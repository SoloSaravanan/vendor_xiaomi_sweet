/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Sweet thermal policy daemon. This source implements the profile-driven
 * sensor, monitor, step and cooling-device policies used by sweet.
 */
#define LOG_TAG "SweetThermald"
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <cutils/properties.h>
#include <log/log.h>

namespace {
using Clock = std::chrono::steady_clock;
constexpr char kVendorEtc[] = "/vendor/etc";
constexpr char kConfigDir[] = "/data/vendor/thermal/config";
constexpr char kGlobalMode[] = "/data/vendor/thermal/thermal-global-mode";
constexpr char kSconfig[] = "/sys/class/thermal/thermal_message/sconfig";
constexpr char kScreenState[] = "/sys/class/thermal/thermal_message/screen_state";
constexpr char kCpuLimits[] = "/sys/class/thermal/thermal_message/cpu_limits";
constexpr char kThermalZones[] = "/sys/class/thermal";
constexpr int kDefaultPollMs = 1000;

std::string trim(const std::string& value) {
    const size_t first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    const size_t last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}
std::string normalize(std::string value) {
    value = trim(value);
    for (char& ch : value) {
        const unsigned char byte = static_cast<unsigned char>(ch);
        ch = ch == '-' ? '_' : static_cast<char>(std::tolower(byte));
    }
    return value;
}
std::vector<std::string> words(const std::string& value) {
    std::istringstream input(value);
    std::vector<std::string> result;
    std::string word;
    while (input >> word) result.push_back(word);
    return result;
}
std::vector<std::string> split(const std::string& value, char delimiter) {
    std::vector<std::string> result;
    std::stringstream input(value);
    std::string item;
    while (std::getline(input, item, delimiter)) result.push_back(trim(item));
    return result;
}
bool parse_int(const std::string& text, int* value) {
    if (!value) return false;
    const std::string input = trim(text);
    if (input.empty()) return false;
    char* end = nullptr;
    errno = 0;
    const long parsed = std::strtol(input.c_str(), &end, 10);
    if (errno || end == input.c_str() || *end || parsed < INT_MIN || parsed > INT_MAX) {
        return false;
    }
    *value = static_cast<int>(parsed);
    return true;
}
bool read_text(const std::string& path, std::string* value) {
    if (!value) return false;
    std::ifstream input(path);
    std::string line;
    if (!input.is_open() || !std::getline(input, line)) return false;
    *value = trim(line);
    return !value->empty();
}
bool read_int(const std::string& path, int* value) {
    std::string text;
    return read_text(path, &text) && parse_int(text, value);
}
bool write_text(const std::string& path, const std::string& value) {
    const int fd = open(path.c_str(), O_WRONLY | O_CLOEXEC);
    if (fd < 0) return false;
    const std::string output = value + "\n";
    size_t written = 0;
    while (written < output.size()) {
        const ssize_t count = write(fd, output.data() + written, output.size() - written);
        if (count < 0) {
            if (errno == EINTR) continue;
            close(fd);
            return false;
        }
        if (count == 0) {
            close(fd);
            return false;
        }
        written += static_cast<size_t>(count);
    }
    close(fd);
    return true;
}
bool write_int(const std::string& path, int value) {
    return write_text(path, std::to_string(value));
}
long long file_stamp(const std::string& path) {
    struct stat status {};
    if (stat(path.c_str(), &status) != 0) return -1;
    return static_cast<long long>(status.st_mtime) * 1000000000LL + status.st_size;
}

struct SensorDefinition {
    enum class Kind { kVirtual, kSimulated };
    Kind kind = Kind::kSimulated;
    std::string path;
    std::vector<std::string> inputs;
    std::vector<int> weights;
    int weight_sum = 1;
    int compensation = 0;
    int polling_ms = kDefaultPollMs;
};
struct DeviceDefinition {
    std::string name;
    std::string cooling_name;
    std::string path;
    int default_target = 0;
    bool select_higher = true;
};
struct Rule {
    enum class Kind { kMonitor, kStep };
    Kind kind = Kind::kMonitor;
    std::string section;
    std::string sensor;
    std::vector<std::string> devices;
    std::vector<int> trigger;
    std::vector<int> clear;
    std::vector<std::vector<int>> targets;
    std::vector<bool> active_thresholds;
    int polling_ms = kDefaultPollMs;
    bool reverse = false;
    int level = -1;
    Clock::time_point next_poll {};
};
struct Profile {
    std::map<std::string, SensorDefinition> sensors;
    std::vector<Rule> rules;
};
struct SensorSample {
    int value = 0;
    Clock::time_point sampled_at {};
    bool valid = false;
};
std::vector<int> parse_values(const std::vector<std::string>& values) {
    std::vector<int> result;
    for (const std::string& value : values) {
        int parsed = 0;
        if (parse_int(value, &parsed)) result.push_back(parsed);
    }
    return result;
}

bool parse_profile(const std::string& path, Profile* profile) {
    if (!profile) return false;
    std::ifstream input(path);
    if (!input.is_open()) {
        ALOGE("cannot open profile %s", path.c_str());
        return false;
    }
    struct Section {
        std::string name;
        std::map<std::string, std::vector<std::string>> properties;
    } section;
    bool have_section = false;
    size_t accepted = 0;
    auto finish_section = [&]() {
        if (!have_section) return;
        const auto algorithm = section.properties.find("algo_type");
        if (algorithm == section.properties.end() || algorithm->second.empty()) {
            have_section = false;
            section = {};
            return;
        }
        const std::string type = normalize(algorithm->second.front());
        if (type == "virtual" || type == "simulated") {
            SensorDefinition definition;
            definition.kind = type == "virtual" ? SensorDefinition::Kind::kVirtual
                                                : SensorDefinition::Kind::kSimulated;
            auto path_it = section.properties.find("path");
            if (path_it != section.properties.end() && !path_it->second.empty()) {
                definition.path = path_it->second.front();
            }
            auto input_it = section.properties.find("sensors");
            if (input_it != section.properties.end()) definition.inputs = input_it->second;
            auto weight_it = section.properties.find("weight");
            if (weight_it != section.properties.end()) {
                definition.weights = parse_values(weight_it->second);
            }
            auto sum_it = section.properties.find("weight_sum");
            if (sum_it != section.properties.end() && !sum_it->second.empty()) {
                parse_int(sum_it->second.front(), &definition.weight_sum);
            }
            auto compensation_it = section.properties.find("compensation");
            if (compensation_it != section.properties.end() &&
                !compensation_it->second.empty()) {
                parse_int(compensation_it->second.front(), &definition.compensation);
            }
            auto polling_it = section.properties.find("polling");
            if (polling_it != section.properties.end() && !polling_it->second.empty()) {
                parse_int(polling_it->second.front(), &definition.polling_ms);
            }

            const bool valid_sensor =
                    (definition.kind == SensorDefinition::Kind::kSimulated &&
                     !definition.path.empty()) ||
                    (definition.kind == SensorDefinition::Kind::kVirtual &&
                     !definition.inputs.empty() &&
                     definition.inputs.size() == definition.weights.size() &&
                     definition.weight_sum != 0);
            if (!valid_sensor) {
                ALOGE("invalid sensor section [%s] in %s",
                      section.name.c_str(), path.c_str());
            } else {
                profile->sensors[normalize(section.name)] = std::move(definition);
                ++accepted;
            }
        } else if (type == "monitor" || type == "ss") {
            Rule rule;
            rule.kind = type == "ss" ? Rule::Kind::kStep : Rule::Kind::kMonitor;
            rule.section = section.name;
            auto get_first = [&](const char* key) -> std::string {
                const auto it = section.properties.find(key);
                return it == section.properties.end() || it->second.empty()
                           ? std::string() : it->second.front();
            };
            rule.sensor = normalize(get_first("sensor"));
            const auto device_it = section.properties.find("device");
            if (device_it != section.properties.end()) {
                for (const std::string& token : device_it->second) {
                    for (const std::string& device : split(token, '+')) {
                        if (!device.empty()) rule.devices.push_back(normalize(device));
                    }
                }
            }
            const auto trigger_it = section.properties.find("trig");
            if (trigger_it != section.properties.end()) {
                rule.trigger = parse_values(trigger_it->second);
            }
            const auto clear_it = section.properties.find("clr");
            if (clear_it != section.properties.end()) {
                rule.clear = parse_values(clear_it->second);
            }
            const auto target_it = section.properties.find("target");
            if (target_it != section.properties.end()) {
                for (const std::string& token : target_it->second) {
                    std::vector<int> row;
                    for (const std::string& item : split(token, '+')) {
                        int target = 0;
                        if (parse_int(item, &target)) row.push_back(target);
                    }
                    if (!row.empty()) rule.targets.push_back(std::move(row));
                }
            }
            auto polling_it = section.properties.find("polling");
            if (polling_it != section.properties.end() && !polling_it->second.empty()) {
                parse_int(polling_it->second.front(), &rule.polling_ms);
            }
            auto reverse_it = section.properties.find("reverse");
            if (reverse_it != section.properties.end() && !reverse_it->second.empty()) {
                int reverse = 0;
                if (parse_int(reverse_it->second.front(), &reverse)) {
                    rule.reverse = reverse != 0;
                }
            }

            bool valid = !rule.sensor.empty() && !rule.devices.empty() &&
                         !rule.trigger.empty() &&
                         rule.trigger.size() == rule.clear.size() &&
                         rule.trigger.size() == rule.targets.size() &&
                         rule.polling_ms > 0;
            for (const auto& row : rule.targets) {
                valid = valid && row.size() == rule.devices.size();
            }
            if (!valid) {
                ALOGE("invalid policy section [%s] in %s",
                      section.name.c_str(), path.c_str());
            } else {
                rule.active_thresholds.assign(rule.trigger.size(), false);
                profile->rules.push_back(std::move(rule));
                ++accepted;
            }
        }
        have_section = false;
        section = {};
    };

    std::string line;
    while (std::getline(input, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        if (line.front() == '[' && line.back() == ']') {
            finish_section();
            section.name = trim(line.substr(1, line.size() - 2));
            have_section = true;
            continue;
        }
        if (!have_section) continue;
        std::istringstream fields(line);
        std::string key;
        fields >> key;
        std::string value;
        std::getline(fields, value);
        value = trim(value);
        if (!key.empty() && !value.empty()) {
            section.properties[normalize(key)] = words(value);
        }
    }
    finish_section();
    if (accepted == 0) {
        ALOGE("profile %s contained no usable sensors or policies", path.c_str());
        return false;
    }
    ALOGI("loaded %zu source-defined sensors and rules from %s",
          accepted, path.c_str());
    return true;
}

bool parse_device_config(const std::string& path,
                         std::map<std::string, DeviceDefinition>* devices) {
    if (!devices) return false;
    std::ifstream input(path);
    if (!input.is_open()) return false;
    DeviceDefinition current;
    bool in_section = false;
    std::string line;
    while (std::getline(input, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        if (line == "[") {
            if (in_section && !current.name.empty()) {
                (*devices)[normalize(current.name)] = current;
            }
            current = {};
            in_section = true;
            continue;
        }
        if (line == "]") {
            if (in_section && !current.name.empty()) {
                (*devices)[normalize(current.name)] = current;
            }
            current = {};
            in_section = false;
            continue;
        }
        if (!in_section) continue;
        const size_t separator = line.find(':');
        if (separator == std::string::npos) continue;
        const std::string key = normalize(line.substr(0, separator));
        const std::string value = trim(line.substr(separator + 1));
        if (key == "name") current.name = value;
        else if (key == "cooling_name") current.cooling_name = value;
        else if (key == "path") current.path = value;
        else if (key == "def_target") parse_int(value, &current.default_target);
        else if (key == "select_higher") {
            int higher = 1;
            if (parse_int(value, &higher)) current.select_higher = higher != 0;
        }
    }
    if (in_section && !current.name.empty()) {
        (*devices)[normalize(current.name)] = current;
    }
    return !devices->empty();
}

std::string profile_map_path() {
    const std::string override_path = std::string(kConfigDir) + "/thermal-map.conf";
    return access(override_path.c_str(), R_OK) == 0
               ? override_path : std::string(kVendorEtc) + "/thermal-map.conf";
}
std::map<int, std::string> parse_profile_map(const std::string& path) {
    std::map<int, std::string> result;
    std::ifstream input(path);
    if (!input.is_open()) return result;
    std::string line;
    while (std::getline(input, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#' || line.front() != '[' || line.back() != ']') {
            continue;
        }
        const std::string item = line.substr(1, line.size() - 2);
        const size_t separator = item.find(':');
        if (separator == std::string::npos) continue;
        int mode = -1;
        if (!parse_int(item.substr(0, separator), &mode) || mode < 0 || mode > 256) {
            continue;
        }
        const std::string profile = trim(item.substr(separator + 1));
        if (!profile.empty() && profile.find('/') == std::string::npos) {
            result[mode] = profile;
        }
    }
    return result;
}
bool create_data_paths() {
    if (mkdir("/data/vendor/thermal", 0770) != 0 && errno != EEXIST) return false;
    if (mkdir(kConfigDir, 0770) != 0 && errno != EEXIST) return false;
    (void)chown("/data/vendor/thermal", 1000, 1000);
    (void)chown(kConfigDir, 1000, 1000);
    (void)chmod("/data/vendor/thermal", 0770);
    (void)chmod(kConfigDir, 0770);
    if (access(kGlobalMode, F_OK) != 0) {
        const int fd = open(kGlobalMode, O_WRONLY | O_CREAT | O_CLOEXEC, 0664);
        if (fd >= 0) {
            (void)write(fd, "0\n", 2);
            (void)fchown(fd, 1000, 1000);
            (void)fchmod(fd, 0664);
            close(fd);
        }
    }
    (void)chown(kGlobalMode, 1000, 1000);
    (void)chmod(kGlobalMode, 0664);
    (void)property_set("vendor.sys.thermal.data.path", "/data/vendor/thermal/");
    return true;
}
class SweetThermald {
  public:
    bool initialize() {
        if (!create_data_paths()) {
            ALOGE("could not prepare thermal data directory: %d", errno);
            return false;
        }
        if (!parse_device_config(std::string(kVendorEtc) + "/thermald-devices.conf",
                                 &devices_)) {
            ALOGE("could not load thermald-devices.conf");
            return false;
        }
        scan_thermal_zones();
        return reload_profile(true);
    }

    void run() {
        for (;;) {
            const auto now = Clock::now();
            refresh_thermal_zones_if_needed(now);
            maybe_reload_profile();
            int screen_state = 0;
            (void)read_int(kScreenState, &screen_state);

            for (Rule& rule : profile_.rules) {
                if (now < rule.next_poll) continue;
                rule.next_poll = now + std::chrono::milliseconds(rule.polling_ms);
                int temperature = 0;
                if (read_sensor(rule.sensor, now, 0, &temperature)) {
                    update_rule(&rule, temperature);
                }
            }
            apply_profile_targets(screen_state);

            // Recheck screen state at the default sensor polling interval,
            // and wake earlier when a rule or profile reload is due.
            Clock::time_point next_wake =
                    now + std::chrono::milliseconds(kDefaultPollMs);
            if (next_reload_check_ < next_wake) next_wake = next_reload_check_;
            for (const Rule& rule : profile_.rules) {
                if (rule.next_poll < next_wake) next_wake = rule.next_poll;
            }
            std::this_thread::sleep_until(next_wake);
        }
    }

  private:
    std::map<std::string, std::string> thermal_zone_paths_;
    std::map<std::string, SensorSample> sensor_cache_;
    std::map<std::string, DeviceDefinition> devices_;
    std::map<std::string, std::string> cooling_state_paths_;
    std::map<std::string, int> applied_targets_;
    Profile profile_;
    std::string active_profile_path_;
    std::string active_profile_map_path_;
    long long active_profile_stamp_ = -2;
    long long active_map_stamp_ = -2;
    int active_sconfig_ = INT_MIN;
    int active_global_mode_ = INT_MIN;
    bool charger_boot_ = false;
    Clock::time_point last_zone_scan_ {};
    Clock::time_point next_reload_check_ {};

    void scan_thermal_zones() {
        thermal_zone_paths_.clear();
        for (int index = 0; index < 256; ++index) {
            const std::string base = std::string(kThermalZones) +
                    "/thermal_zone" + std::to_string(index);
            std::string type;
            if (!read_text(base + "/type", &type)) continue;
            const std::string key = normalize(type);
            thermal_zone_paths_.emplace(key, base + "/temp");
            constexpr char suffix[] = "_usr";
            constexpr size_t suffix_size = sizeof(suffix) - 1;
            if (key.size() > suffix_size &&
                key.compare(key.size() - suffix_size, suffix_size, suffix) == 0) {
                thermal_zone_paths_.emplace(
                        key.substr(0, key.size() - suffix_size), base + "/temp");
            }
        }
        last_zone_scan_ = Clock::now();
        sensor_cache_.clear();
    }

    void refresh_thermal_zones_if_needed(Clock::time_point now) {
        if (last_zone_scan_ == Clock::time_point{} ||
            now - last_zone_scan_ >= std::chrono::seconds(30)) {
            scan_thermal_zones();
        }
    }

    bool is_charger_boot() const {
        char boot_mode[PROPERTY_VALUE_MAX] = {};
        property_get("ro.bootmode", boot_mode, "");
        return std::string(boot_mode) == "charger";
    }

    std::string resolve_profile(int sconfig, int global_mode,
                                std::string* map_path) {
        *map_path = profile_map_path();
        const auto profiles = parse_profile_map(*map_path);
        std::vector<int> candidates;
        if (sconfig >= 0 && sconfig <= 256) candidates.push_back(sconfig);
        if (global_mode >= 0 && global_mode <= 256 &&
            (candidates.empty() || candidates.front() != global_mode)) {
            candidates.push_back(global_mode);
        }
        if (std::find(candidates.begin(), candidates.end(), 0) == candidates.end()) {
            candidates.push_back(0);
        }

        for (int mode : candidates) {
            const auto it = profiles.find(mode);
            if (it == profiles.end()) continue;
            const std::string override_path = std::string(kConfigDir) + "/" + it->second;
            if (access(override_path.c_str(), R_OK) == 0) return override_path;
            const std::string vendor_path = std::string(kVendorEtc) + "/" + it->second;
            if (access(vendor_path.c_str(), R_OK) == 0) return vendor_path;
        }
        const std::string fallback = std::string(kVendorEtc) + "/thermal-normal.conf";
        return access(fallback.c_str(), R_OK) == 0 ? fallback : std::string();
    }

    bool reload_profile(bool force) {
        std::string new_map_path;
        const bool charger_boot = is_charger_boot();
        int sconfig = 0;
        const bool have_sconfig = read_int(kSconfig, &sconfig);
        int global_mode = 0;
        (void)read_int(kGlobalMode, &global_mode);

        std::string new_path;
        if (charger_boot) {
            const std::string charger_path =
                    std::string(kVendorEtc) + "/thermal-chg-only.conf";
            if (access(charger_path.c_str(), R_OK) == 0) new_path = charger_path;
        }
        if (new_path.empty()) {
            new_path = resolve_profile(have_sconfig ? sconfig : -1,
                                       global_mode, &new_map_path);
        }

        const long long profile_stamp = file_stamp(new_path);
        const long long map_stamp = file_stamp(new_map_path);
        if (!force && new_path == active_profile_path_ &&
            new_map_path == active_profile_map_path_ &&
            profile_stamp == active_profile_stamp_ &&
            map_stamp == active_map_stamp_ &&
            sconfig == active_sconfig_ &&
            global_mode == active_global_mode_ &&
            charger_boot == charger_boot_) {
            return false;
        }

        const std::string normal_override =
                std::string(kConfigDir) + "/thermal-normal.conf";
        const std::string normal_vendor =
                std::string(kVendorEtc) + "/thermal-normal.conf";
        const std::string normal_path =
                access(normal_override.c_str(), R_OK) == 0
                        ? normal_override : normal_vendor;
        Profile baseline;
        bool baseline_loaded = parse_profile(normal_path, &baseline);
        if (!baseline_loaded && normal_path != normal_vendor) {
            baseline_loaded = parse_profile(normal_vendor, &baseline);
        }
        if (!baseline_loaded) {
            ALOGE("could not load base thermal sensor definitions");
            return false;
        }

        Profile next;
        bool loaded = false;
        if (new_path == normal_path) {
            next = baseline;
            loaded = true;
        } else if (!new_path.empty()) {
            loaded = parse_profile(new_path, &next);
        }
        if (!loaded) {
            ALOGW("profile unavailable; falling back to %s", normal_path.c_str());
            new_path = normal_path;
            next = baseline;
            loaded = true;
        }

        /*
         * MI profiles such as mgame/tgame contain policy rules only. Their
         * GAME-VIRTUAL-SENSOR and BAT_SOC definitions remain from the normal
         * sensor configuration loaded at daemon startup.
         */
        for (const auto& sensor : baseline.sensors) {
            if (next.sensors.find(sensor.first) == next.sensors.end()) {
                next.sensors.emplace(sensor.first, sensor.second);
            }
        }

        reset_applied_targets();
        profile_ = std::move(next);
        active_profile_path_ = new_path;
        active_profile_map_path_ = new_map_path;
        active_profile_stamp_ = file_stamp(active_profile_path_);
        active_map_stamp_ = file_stamp(active_profile_map_path_);
        active_sconfig_ = sconfig;
        active_global_mode_ = global_mode;
        charger_boot_ = charger_boot;
        sensor_cache_.clear();
        ALOGI("active policy %s (sconfig=%d global_mode=%d%s)",
              active_profile_path_.c_str(), sconfig, global_mode,
              charger_boot ? ", charger boot" : "");
        return true;
    }

    void maybe_reload_profile() {
        const auto now = Clock::now();
        if (now < next_reload_check_) return;
        next_reload_check_ = now + std::chrono::seconds(1);
        (void)reload_profile(false);
    }

    bool read_sensor(const std::string& name, Clock::time_point now,
                     int depth, int* value) {
        if (!value || depth > 8) return false;
        const std::string key = normalize(name);
        const auto definition = profile_.sensors.find(key);
        const int polling_ms = definition == profile_.sensors.end()
                                   ? kDefaultPollMs
                                   : std::max(1, definition->second.polling_ms);
        const auto cached = sensor_cache_.find(key);
        if (cached != sensor_cache_.end() && cached->second.valid &&
            now - cached->second.sampled_at <
                    std::chrono::milliseconds(polling_ms)) {
            *value = cached->second.value;
            return true;
        }

        int sample = 0;
        bool valid = false;
        if (definition != profile_.sensors.end() &&
            definition->second.kind == SensorDefinition::Kind::kVirtual) {
            const SensorDefinition& virtual_sensor = definition->second;
            int64_t weighted_sum = 0;
            valid = true;
            for (size_t index = 0; index < virtual_sensor.inputs.size(); ++index) {
                int input = 0;
                if (!read_sensor(virtual_sensor.inputs[index], now, depth + 1, &input)) {
                    valid = false;
                    break;
                }
                weighted_sum += static_cast<int64_t>(input) *
                                virtual_sensor.weights[index];
            }
            if (valid) {
                sample = static_cast<int>(weighted_sum / virtual_sensor.weight_sum) +
                         virtual_sensor.compensation;
            }
        } else {
            std::string sensor_path;
            if (definition != profile_.sensors.end() &&
                definition->second.kind == SensorDefinition::Kind::kSimulated) {
                sensor_path = definition->second.path;
            } else {
                auto path = thermal_zone_paths_.find(key);
                if (path == thermal_zone_paths_.end()) {
                    constexpr char suffix[] = "_usr";
                    constexpr size_t suffix_size = sizeof(suffix) - 1;
                    if (key.size() > suffix_size &&
                        key.compare(key.size() - suffix_size, suffix_size, suffix) == 0) {
                        path = thermal_zone_paths_.find(
                                key.substr(0, key.size() - suffix_size));
                    }
                }
                if (path != thermal_zone_paths_.end()) sensor_path = path->second;
            }
            if (!sensor_path.empty() && read_int(sensor_path, &sample)) {
                valid = true;
            }
        }

        if (!valid) return false;
        sensor_cache_[key] = SensorSample{sample, now, true};
        *value = sample;
        return true;
    }
    void update_rule(Rule* rule, int temperature) {
        if (!rule) return;
        const size_t count = rule->trigger.size();
        if (rule->kind == Rule::Kind::kStep) {
            if (rule->level < 0) {
                if (temperature >= rule->trigger[0]) rule->level = 0;
            } else {
                while (static_cast<size_t>(rule->level + 1) < count &&
                       temperature >= rule->trigger[rule->level + 1]) {
                    ++rule->level;
                }
                while (rule->level >= 0 &&
                       temperature <= rule->clear[rule->level]) {
                    --rule->level;
                }
            }
            return;
        }

        for (size_t index = 0; index < count; ++index) {
            if (rule->reverse) {
                if (!rule->active_thresholds[index] &&
                    temperature <= rule->trigger[index]) {
                    rule->active_thresholds[index] = true;
                } else if (rule->active_thresholds[index] &&
                           temperature >= rule->clear[index]) {
                    rule->active_thresholds[index] = false;
                }
            } else {
                if (!rule->active_thresholds[index] &&
                    temperature >= rule->trigger[index]) {
                    rule->active_thresholds[index] = true;
                } else if (rule->active_thresholds[index] &&
                           temperature <= rule->clear[index]) {
                    rule->active_thresholds[index] = false;
                }
            }
        }

        rule->level = -1;
        for (size_t index = 0; index < count; ++index) {
            if (rule->active_thresholds[index]) {
                rule->level = static_cast<int>(index);
            }
        }
    }

    const DeviceDefinition* device_definition(const std::string& name) const {
        const auto it = devices_.find(normalize(name));
        return it == devices_.end() ? nullptr : &it->second;
    }

    int default_target(const std::string& name) const {
        const DeviceDefinition* device = device_definition(name);
        return device ? device->default_target : 0;
    }

    bool select_higher(const std::string& name) const {
        const std::string key = normalize(name);
        if (key.rfind("cpu", 0) == 0 && key.size() > 3 &&
            std::isdigit(static_cast<unsigned char>(key[3]))) {
            return false;
        }
        const DeviceDefinition* device = device_definition(key);
        return !device || device->select_higher;
    }

    bool resolve_cooling_path(const DeviceDefinition& device, std::string* path) {
        if (!path) return false;
        const std::string key = normalize(device.name);
        const auto cached = cooling_state_paths_.find(key);
        if (cached != cooling_state_paths_.end()) {
            *path = cached->second;
            return true;
        }
        if (device.cooling_name.empty()) return false;

        const std::string expected = normalize(device.cooling_name);
        for (int index = 0; index < 256; ++index) {
            const std::string base = std::string(kThermalZones) +
                    "/cooling_device" + std::to_string(index);
            std::string type;
            if (read_text(base + "/type", &type) && normalize(type) == expected) {
                *path = base + "/cur_state";
                cooling_state_paths_[key] = *path;
                return true;
            }
        }
        return false;
    }

    int cpu_max_frequency(const std::string& cpu_name) const {
        int cpu = -1;
        if (cpu_name.size() <= 3 || !parse_int(cpu_name.substr(3), &cpu) ||
            cpu < 0 || cpu > 15) {
            return 0;
        }
        const std::string path = "/sys/devices/system/cpu/" + cpu_name +
                                 "/cpufreq/cpuinfo_max_freq";
        int frequency = 0;
        return read_int(path, &frequency) && frequency > 0 ? frequency : 0;
    }

    bool apply_target(const std::string& name, int target) {
        const std::string key = normalize(name);
        if (key == "boost_limit") {
            /* The sweet device map has no boost_limit control device. */
            return true;
        }
        if (key.rfind("hotplug_cpu", 0) == 0) {
            const std::string cpu = key.substr(std::string("hotplug_").size());
            int cpu_number = -1;
            if (cpu.size() <= 3 || !parse_int(cpu.substr(3), &cpu_number) ||
                cpu_number <= 0 || cpu_number > 15) {
                return false;
            }
            return write_int("/sys/devices/system/cpu/" + cpu + "/online",
                             target != 0 ? 0 : 1);
        }
        if (key.rfind("cpu", 0) == 0 && key.size() > 3 &&
            std::isdigit(static_cast<unsigned char>(key[3]))) {
            const int frequency = target > 0 ? target : cpu_max_frequency(key);
            if (frequency <= 0) return false;
            int cpu_number = -1;
            if (!parse_int(key.substr(3), &cpu_number) || cpu_number < 0) return false;
            if (access(kCpuLimits, W_OK) == 0) {
                return write_text(kCpuLimits,
                                  key + " " + std::to_string(frequency));
            }
            return write_int("/sys/devices/system/cpu/" + key +
                             "/cpufreq/scaling_max_freq", frequency);
        }

        const DeviceDefinition* device = device_definition(key);
        if (!device) {
            ALOGW("profile refers to unknown thermal device %s", key.c_str());
            return false;
        }
        std::string path = device->path;
        if (path.empty() && !resolve_cooling_path(*device, &path)) {
            ALOGW("cooling device %s (%s) is unavailable",
                  key.c_str(), device->cooling_name.c_str());
            return false;
        }
        return write_int(path, target);
    }

    void apply_profile_targets(int screen_state) {
        std::map<std::string, int> desired;
        for (const Rule& rule : profile_.rules) {
            if (rule.level < 0 ||
                static_cast<size_t>(rule.level) >= rule.targets.size()) {
                continue;
            }
            const std::vector<int>& row = rule.targets[rule.level];
            for (size_t index = 0; index < rule.devices.size(); ++index) {
                const std::string& device = rule.devices[index];
                int target = row[index];
                if (device == "battery" && target >= 100) {
                    target = screen_state != 0 ? target / 100 : target % 100;
                }
                const auto current = desired.find(device);
                if (current == desired.end() ||
                    (select_higher(device) && target > current->second) ||
                    (!select_higher(device) && target < current->second)) {
                    desired[device] = target;
                }
            }
        }

        std::set<std::string> affected;
        for (const auto& item : applied_targets_) affected.insert(item.first);
        for (const auto& item : desired) affected.insert(item.first);
        for (const std::string& device : affected) {
            const auto active = desired.find(device);
            const int target = active == desired.end()
                                   ? default_target(device) : active->second;
            const auto previous = applied_targets_.find(device);
            if (previous != applied_targets_.end() && previous->second == target) {
                continue;
            }
            if (apply_target(device, target)) applied_targets_[device] = target;
        }
    }

    void reset_applied_targets() {
        for (const auto& item : applied_targets_) {
            (void)apply_target(item.first, default_target(item.first));
        }
        applied_targets_.clear();
    }
};

}  // namespace

int main() {
    SweetThermald daemon;
    if (!daemon.initialize()) return 1;
    ALOGI("source-defined sweet thermal policies started");
    daemon.run();
    return 0;
}
