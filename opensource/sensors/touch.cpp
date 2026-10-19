// SPDX-License-Identifier: Apache-2.0

#define LOG_TAG "sensors.touch"

#include <errno.h>
#include <fcntl.h>
#include <hardware/hardware.h>
#include <hardware/sensors.h>
#include <log/log.h>
#include <new>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

namespace {

constexpr int32_t kSensorHandle = 0;
constexpr int32_t kSensorType = 0x01fa2657;
constexpr char kPalmSensorPath[] =
        "/sys/devices/virtual/touch/touch_dev/palm_sensor";

struct TouchDevice {
    sensors_poll_device_1_t hal{};
    pthread_mutex_t mutex{};
    pthread_cond_t condition{};
    bool enabled = false;
};

static int write_sensor_enabled(bool enabled) {
    const int fd = open(kPalmSensorPath, O_WRONLY | O_CLOEXEC);
    if (fd < 0) return -errno;

    const char value[] = {enabled ? '1' : '0', '\n'};
    size_t offset = 0;
    while (offset < sizeof(value)) {
        const ssize_t size = write(fd, value + offset, sizeof(value) - offset);
        if (size < 0 && errno == EINTR) continue;
        if (size < 0) {
            const int error = errno;
            close(fd);
            return -error;
        }
        if (size == 0) {
            close(fd);
            return -EIO;
        }
        offset += static_cast<size_t>(size);
    }

    close(fd);
    return 0;
}

static int read_sensor_value(int* value) {
    char buffer[32]{};
    const int fd = open(kPalmSensorPath, O_RDONLY | O_CLOEXEC);
    if (fd < 0) return -errno;

    ssize_t size;
    do {
        size = read(fd, buffer, sizeof(buffer) - 1);
    } while (size < 0 && errno == EINTR);
    const int read_error = size < 0 ? errno : 0;
    close(fd);
    if (size < 0) return -read_error;
    if (size == 0) return -EIO;

    char* end = nullptr;
    errno = 0;
    const long parsed = strtol(buffer, &end, 10);
    if (errno != 0 || end == buffer || parsed < 0 || parsed > 1) return -EINVAL;
    while (*end == '\n' || *end == '\r' || *end == ' ' || *end == '\t') ++end;
    if (*end != '\0') return -EINVAL;

    *value = static_cast<int>(parsed);
    return 0;
}

static int fill_event(sensors_event_t* event, int value) {
    timespec timestamp{};
    if (clock_gettime(CLOCK_BOOTTIME, &timestamp) != 0) return -errno;

    memset(event, 0, sizeof(*event));
    event->version = sizeof(*event);
    event->sensor = kSensorHandle;
    event->type = kSensorType;
    event->timestamp = static_cast<int64_t>(timestamp.tv_sec) * 1000000000LL +
                       timestamp.tv_nsec;
    event->data[0] = static_cast<float>(value);
    return 0;
}

static int activate_sensor(sensors_poll_device_t* common, int handle, int enabled) {
    if (handle != kSensorHandle || (enabled != 0 && enabled != 1)) return -EINVAL;
    TouchDevice* device = reinterpret_cast<TouchDevice*>(common);

    const int lock_result = pthread_mutex_lock(&device->mutex);
    if (lock_result != 0) return -lock_result;

    if (device->enabled == (enabled != 0)) {
        pthread_mutex_unlock(&device->mutex);
        return 0;
    }

    const int result = write_sensor_enabled(enabled != 0);
    if (result == 0) {
        device->enabled = enabled != 0;
        pthread_cond_broadcast(&device->condition);
    }
    pthread_mutex_unlock(&device->mutex);
    return result;
}

static int set_delay(sensors_poll_device_t*, int handle, int64_t) {
    return handle == kSensorHandle ? 0 : -EINVAL;
}

static int batch_sensor(sensors_poll_device_1_t*, int handle, int, int64_t, int64_t) {
    return handle == kSensorHandle ? 0 : -EINVAL;
}

static int flush_sensor(sensors_poll_device_1_t*, int) {
    return -EINVAL;
}

static int inject_sensor_data(sensors_poll_device_1_t*, const sensors_event_t*) {
    return -EINVAL;
}

static int poll_sensor(sensors_poll_device_t* common, sensors_event_t* events, int count) {
    if (events == nullptr || count <= 0) return -EINVAL;
    TouchDevice* device = reinterpret_cast<TouchDevice*>(common);

    for (;;) {
        int lock_result = pthread_mutex_lock(&device->mutex);
        if (lock_result != 0) return -lock_result;
        while (!device->enabled) {
            const int wait_result = pthread_cond_wait(&device->condition, &device->mutex);
            if (wait_result != 0) {
                pthread_mutex_unlock(&device->mutex);
                return -wait_result;
            }
        }
        pthread_mutex_unlock(&device->mutex);

        int value;
        const int result = read_sensor_value(&value);
        if (result == -EINTR) continue;
        if (result < 0) {
            ALOGE("Unable to read palm sensor: %s", strerror(-result));
            return result;
        }

        lock_result = pthread_mutex_lock(&device->mutex);
        if (lock_result != 0) return -lock_result;
        const bool enabled = device->enabled;
        pthread_mutex_unlock(&device->mutex);
        if (!enabled) continue;

        const int event_result = fill_event(&events[0], value);
        return event_result < 0 ? event_result : 1;
    }
}

static int close_device(hw_device_t* common) {
    TouchDevice* device = reinterpret_cast<TouchDevice*>(common);

    pthread_mutex_lock(&device->mutex);
    const bool was_enabled = device->enabled;
    device->enabled = false;
    pthread_cond_broadcast(&device->condition);
    pthread_mutex_unlock(&device->mutex);

    if (was_enabled) write_sensor_enabled(false);
    pthread_cond_destroy(&device->condition);
    pthread_mutex_destroy(&device->mutex);
    delete device;
    return 0;
}

static int open_device(const hw_module_t* module, const char*, hw_device_t** output) {
    if (module == nullptr || output == nullptr) return -EINVAL;

    TouchDevice* device = new (std::nothrow) TouchDevice{};
    if (device == nullptr) return -ENOMEM;

    int result = pthread_mutex_init(&device->mutex, nullptr);
    if (result != 0) {
        delete device;
        return -result;
    }
    result = pthread_cond_init(&device->condition, nullptr);
    if (result != 0) {
        pthread_mutex_destroy(&device->mutex);
        delete device;
        return -result;
    }

    device->hal.v0.common.tag = HARDWARE_DEVICE_TAG;
    device->hal.v0.common.version = SENSORS_DEVICE_API_VERSION_1_3;
    device->hal.v0.common.module = const_cast<hw_module_t*>(module);
    device->hal.v0.common.close = close_device;
    device->hal.v0.activate = activate_sensor;
    device->hal.v0.setDelay = set_delay;
    device->hal.v0.poll = poll_sensor;
    device->hal.batch = batch_sensor;
    device->hal.flush = flush_sensor;
    device->hal.inject_sensor_data = inject_sensor_data;
    *output = &device->hal.v0.common;
    return 0;
}

static int get_sensors_list(sensors_module_t*, const sensor_t** list) {
    if (list == nullptr) return -EINVAL;

    static const sensor_t sensors[] = {
        {"Touch Sensor", "xiaomi", 1, kSensorHandle, kSensorType, 1.0f, 0.0f, 0.01f,
         0, 0, 0, "qti.sensor.touch", "", 0, 0, {nullptr, nullptr}},
    };
    *list = sensors;
    return static_cast<int>(sizeof(sensors) / sizeof(sensors[0]));
}

static int set_operation_mode(unsigned int mode) {
    return mode == 0 ? 0 : -EINVAL;
}

static hw_module_methods_t module_methods = {open_device};

}  // namespace

sensors_module_t HAL_MODULE_INFO_SYM = {
    {HARDWARE_MODULE_TAG, HARDWARE_MODULE_API_VERSION(1, 0), HARDWARE_HAL_API_VERSION,
     SENSORS_HARDWARE_MODULE_ID, "Xiaomi Touch Sensor HAL", "Xiaomi", &module_methods,
     nullptr, {0}},
    get_sensors_list,
    set_operation_mode,
};
