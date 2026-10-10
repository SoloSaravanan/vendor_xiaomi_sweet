/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "SweetThermalEngine"

#include <errno.h>
#include <stdlib.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <atomic>
#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <cutils/sockets.h>
#include <log/log.h>

namespace {

constexpr size_t kMessageSize = 0x3ce0;
constexpr size_t kClientNameOffset = 4;
constexpr size_t kClientNameSize = 0x14;
constexpr size_t kValueOffset = 0x18;
constexpr int kCameraMitigation = 10;

constexpr char kThermalRoot[] = "/sys/class/thermal";
constexpr char kQ6Sensor[] = "q6-hvx-usr";
constexpr char kQuietSensor[] = "quiet-therm-usr";
constexpr char kSpeakerSensor[] = "pa_therm0";

std::mutex g_event_clients_lock;
std::vector<int> g_event_clients;
std::atomic<bool> g_speaker_requested{false};
std::atomic<int> g_camera_mitigation{0};

std::string trim(std::string value) {
    const size_t first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    const size_t last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::string normalize(std::string value) {
    value = trim(value);
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char ch) {
                       if (ch == '-') return '_';
                       return static_cast<char>(std::tolower(ch));
                   });
    return value;
}

bool read_text(const std::string& path, std::string* output) {
    if (output == nullptr) return false;
    FILE* file = fopen(path.c_str(), "r");
    if (file == nullptr) return false;
    char buffer[256];
    const size_t count = fread(buffer, 1, sizeof(buffer) - 1, file);
    fclose(file);
    if (count == 0) return false;
    buffer[count] = '\0';
    *output = trim(buffer);
    return !output->empty();
}

bool read_integer(const std::string& path, int* value) {
    std::string text;
    if (value == nullptr || !read_text(path, &text)) return false;
    char* end = nullptr;
    const long parsed = strtol(text.c_str(), &end, 10);
    if (end == text.c_str()) return false;
    *value = static_cast<int>(parsed);
    return true;
}

std::map<std::string, std::string> scan_thermal_zones() {
    std::map<std::string, std::string> sensors;
    for (int index = 0; index < 256; ++index) {
        const std::string base = std::string(kThermalRoot) +
                "/thermal_zone" + std::to_string(index);
        std::string type;
        if (!read_text(base + "/type", &type)) continue;

        const std::string key = normalize(type);
        const std::string temperature_path = base + "/temp";
        sensors.emplace(key, temperature_path);
        const std::string suffix = "_usr";
        if (key.size() > suffix.size() &&
            key.compare(key.size() - suffix.size(), suffix.size(), suffix) == 0) {
            sensors.emplace(key.substr(0, key.size() - suffix.size()), temperature_path);
        }
    }
    return sensors;
}

bool find_sensor_path(const std::map<std::string, std::string>& sensors,
                      const char* name, std::string* path) {
    if (name == nullptr || path == nullptr) return false;
    const std::string key = normalize(name);
    auto it = sensors.find(key);
    if (it == sensors.end()) {
        const std::string suffix = "_usr";
        if (key.size() > suffix.size() &&
            key.compare(key.size() - suffix.size(), suffix.size(), suffix) == 0) {
            it = sensors.find(key.substr(0, key.size() - suffix.size()));
        }
    }
    if (it == sensors.end()) return false;
    *path = it->second;
    return true;
}

int get_platform_id() {
    std::string value;
    if (!read_text("/sys/devices/soc0/hw_platform", &value) &&
        !read_text("/sys/devices/system/soc/soc0/hw_platform", &value)) {
        return 0;
    }
    value = trim(value);
    if (value == "Surf") return 1;
    if (value == "AFF") return 2;
    if (value == "Fluid") return 3;
    if (value == "SVLTE_FFA") return 4;
    if (value == "SVLTE_SURF") return 5;
    if (value == "MTP") return 6;
    if (value == "Liquid") return 7;
    if (value == "Dragonboard") return 8;
    if (value == "QRD") return 9;
    if (value == "IDP") return 10;
    if (value == "ATP") return 11;
    return 0;
}

bool read_exact(int fd, uint8_t* data, size_t size) {
    size_t received = 0;
    while (received < size) {
        const ssize_t count = recv(fd, data + received, size - received, 0);
        if (count < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        if (count == 0) return false;
        received += static_cast<size_t>(count);
    }
    return true;
}

bool send_all(int fd, const uint8_t* data, size_t size) {
    size_t sent = 0;
    while (sent < size) {
        const ssize_t count = send(fd, data + sent, size - sent, MSG_NOSIGNAL);
        if (count < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        if (count == 0) return false;
        sent += static_cast<size_t>(count);
    }
    return true;
}

void make_event_message(const char* name, int value, uint8_t* message) {
    memset(message, 0, kMessageSize);
    memcpy(message + kClientNameOffset, name, strnlen(name, kClientNameSize - 1));
    memcpy(message + kValueOffset, &value, sizeof(value));
}

void broadcast_event(const char* name, int value) {
    uint8_t message[kMessageSize];
    make_event_message(name, value, message);

    std::lock_guard<std::mutex> guard(g_event_clients_lock);
    auto it = g_event_clients.begin();
    while (it != g_event_clients.end()) {
        if (send_all(*it, message, sizeof(message))) {
            ++it;
        } else {
            close(*it);
            it = g_event_clients.erase(it);
        }
    }
}

void handle_request(int socket_kind, int fd) {
    uint8_t message[kMessageSize];
    while (read_exact(fd, message, sizeof(message))) {
        char name[kClientNameSize] = {};
        int value = 0;
        memcpy(name, message + kClientNameOffset, sizeof(name));
        name[sizeof(name) - 1] = '\0';
        memcpy(&value, message + kValueOffset, sizeof(value));

        if (socket_kind == 1 && strcmp(name, "spkr") == 0) {
            g_speaker_requested.store(value != 0);
            ALOGI("speaker thermal callback %s", value != 0 ? "enabled" : "disabled");
        }
        /* Bandwidth and runtime-config APIs are not used by sweet clients. */
    }
    close(fd);
}

void accept_loop(int server_fd, int socket_kind) {
    for (;;) {
        const int client_fd = accept4(server_fd, nullptr, nullptr, SOCK_CLOEXEC);
        if (client_fd < 0) {
            if (errno == EINTR) continue;
            ALOGE("accept failed for thermal socket: %d", errno);
            sleep(1);
            continue;
        }

        if (socket_kind == 0) {
            std::lock_guard<std::mutex> guard(g_event_clients_lock);
            g_event_clients.push_back(client_fd);
            const int level = g_camera_mitigation.load();
            uint8_t message[kMessageSize];
            make_event_message("camera", level, message);
            bool sent = send_all(client_fd, message, sizeof(message));
            make_event_message("camcorder", level, message);
            sent = send_all(client_fd, message, sizeof(message)) && sent;
            if (!sent) {
                close(client_fd);
                g_event_clients.pop_back();
            }
        } else {
            std::thread(handle_request, socket_kind, client_fd).detach();
        }
    }
}

int inherited_socket(const char* name) {
    const int fd = android_get_control_socket(name);
    if (fd < 0) ALOGE("init socket %s is unavailable: %d", name, errno);
    return fd;
}

void monitor_thermal_policy() {
    bool q6_active = false;
    bool skin_active = false;
    int last_mitigation = -1;
    int last_speaker_celsius = 1000;
    auto last_zone_scan = std::chrono::steady_clock::now() - std::chrono::seconds(60);
    std::map<std::string, std::string> sensors;

    const int platform_id = get_platform_id();
    const bool has_skin_rule =
            platform_id > 11 || ((1U << platform_id) & 0xc40U) == 0;

    for (;;) {
        const auto now = std::chrono::steady_clock::now();
        if (now - last_zone_scan > std::chrono::seconds(30)) {
            sensors = scan_thermal_zones();
            last_zone_scan = now;
        }

        std::string path;
        int temperature = 0;
        if (find_sensor_path(sensors, kQ6Sensor, &path) &&
            read_integer(path, &temperature)) {
            if (!q6_active && temperature >= 100000) q6_active = true;
            if (q6_active && temperature <= 95000) q6_active = false;
        }

        if (has_skin_rule && find_sensor_path(sensors, kQuietSensor, &path) &&
            read_integer(path, &temperature)) {
            if (!skin_active && temperature >= 56000) skin_active = true;
            if (skin_active && temperature <= 53000) skin_active = false;
        }

        const int mitigation = (q6_active || skin_active) ? kCameraMitigation : 0;
        if (mitigation != last_mitigation) {
            g_camera_mitigation.store(mitigation);
            broadcast_event("camera", mitigation);
            broadcast_event("camcorder", mitigation);
            ALOGI("camera mitigation level %d (q6=%d skin=%d)",
                  mitigation, q6_active, skin_active);
            last_mitigation = mitigation;
        }

        if (g_speaker_requested.load()) {
            if (find_sensor_path(sensors, kSpeakerSensor, &path) &&
                read_integer(path, &temperature) &&
                temperature >= -40000 && temperature <= 125000) {
                const int celsius = temperature / 1000;
                if (celsius != last_speaker_celsius) {
                    broadcast_event("spkr", celsius);
                    last_speaker_celsius = celsius;
                }
            }
        } else {
            last_speaker_celsius = 1000;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

}  // namespace

int main() {
    const struct {
        const char* name;
        int kind;
    } sockets[] = {
        {"thermal-send-client", 0},
        {"thermal-recv-client", 1},
        {"thermal-recv-passive-client", 1},
        {"thermal-send-rule", 2},
    };

    for (const auto& socket : sockets) {
        const int fd = inherited_socket(socket.name);
        if (fd >= 0) std::thread(accept_loop, fd, socket.kind).detach();
    }

    std::thread(monitor_thermal_policy).detach();
    ALOGI("open Atoll thermal policy started");
    for (;;) std::this_thread::sleep_for(std::chrono::hours(24));
    return 0;
}
