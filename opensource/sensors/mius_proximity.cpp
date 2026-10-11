// SPDX-License-Identifier: Apache-2.0

#define LOG_TAG "sensors.mius.proximity"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <hardware/hardware.h>
#include <hardware/sensors.h>
#include <limits.h>
#include <log/log.h>
#include <new>
#include <poll.h>
#include <pthread.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/eventfd.h>
#include <time.h>
#include <unistd.h>

extern "C" int ultrasound_enable(int mode, int enable);

namespace {

constexpr int32_t kSensorHandle = 0;
constexpr size_t kScanSize = 16;
constexpr size_t kMaxSamples = 8;
constexpr char kIioDirectory[] = "/sys/bus/iio/devices";
constexpr char kIioDeviceName[] = "distance";

// Matches the enabled IIO proximity and timestamp channels in us_prox_el_data.
struct IioProximitySample {
    uint16_t value;
    uint16_t reserved;
    uint32_t alignment;
    int64_t timestamp;
};
static_assert(sizeof(IioProximitySample) == kScanSize, "Unexpected MIUS IIO scan size");
static_assert(offsetof(IioProximitySample, timestamp) == 8, "Unexpected MIUS timestamp offset");

struct ProximityDevice {
    sensors_poll_device_1_t hal{};
    pthread_mutex_t mutex{};
    int iio_fd = -1;
    int wake_fd = -1;
    bool enabled = false;
    uint32_t pending_flushes = 0;
    size_t partial_size = 0;
    uint8_t partial[kScanSize]{};
    char iio_node[PATH_MAX]{};
    char buffer_enable[PATH_MAX]{};
    char proximity_enable[PATH_MAX]{};
    char timestamp_enable[PATH_MAX]{};
};

static int read_text_file(const char* path, char* output, size_t output_size) {
    if (output_size < 2) return -EINVAL;

    int fd = open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0) return -errno;

    ssize_t size;
    do {
        size = read(fd, output, output_size - 1);
    } while (size < 0 && errno == EINTR);
    const int read_error = size < 0 ? errno : 0;
    close(fd);
    if (size < 0) return -read_error;

    while (size > 0 && (output[size - 1] == '\n' || output[size - 1] == '\r' ||
                        output[size - 1] == ' ' || output[size - 1] == '\t')) {
        --size;
    }
    output[size] = '\0';
    return 0;
}

static bool format_path(char* output, size_t output_size, const char* base,
                        const char* suffix) {
    const int size = snprintf(output, output_size, "%s/%s", base, suffix);
    return size >= 0 && static_cast<size_t>(size) < output_size;
}

static int find_iio_device(ProximityDevice* device) {
    DIR* directory = opendir(kIioDirectory);
    if (directory == nullptr) return -errno;

    int result = -ENODEV;
    dirent* entry;
    while ((entry = readdir(directory)) != nullptr) {
        if (strncmp(entry->d_name, "iio:device", 10) != 0) continue;

        char base_path[PATH_MAX];
        char name_path[PATH_MAX];
        char name[64];
        if (!format_path(base_path, sizeof(base_path), kIioDirectory, entry->d_name) ||
            !format_path(name_path, sizeof(name_path), base_path, "name")) {
            continue;
        }
        if (read_text_file(name_path, name, sizeof(name)) < 0 ||
            strcmp(name, kIioDeviceName) != 0) {
            continue;
        }

        if (!format_path(device->iio_node, sizeof(device->iio_node), "/dev", entry->d_name) ||
            !format_path(device->buffer_enable, sizeof(device->buffer_enable), base_path,
                         "buffer/enable") ||
            !format_path(device->proximity_enable, sizeof(device->proximity_enable), base_path,
                         "scan_elements/in_proximity_en") ||
            !format_path(device->timestamp_enable, sizeof(device->timestamp_enable), base_path,
                         "scan_elements/in_timestamp_en")) {
            result = -ENAMETOOLONG;
            break;
        }
        result = 0;
        break;
    }

    closedir(directory);
    return result;
}

static int write_sysfs_int(const char* path, int value) {
    int fd = open(path, O_WRONLY | O_CLOEXEC);
    if (fd < 0) return -errno;

    const char data[2] = {value ? '1' : '0', '\n'};
    size_t offset = 0;
    while (offset < sizeof(data)) {
        ssize_t written = write(fd, data + offset, sizeof(data) - offset);
        if (written < 0 && errno == EINTR) continue;
        if (written < 0) {
            const int error = errno;
            close(fd);
            return -error;
        }
        if (written == 0) {
            close(fd);
            return -EIO;
        }
        offset += static_cast<size_t>(written);
    }
    close(fd);
    return 0;
}

static void drain_wake_fd(int fd) {
    uint64_t value;
    for (;;) {
        const ssize_t size = read(fd, &value, sizeof(value));
        if (size == static_cast<ssize_t>(sizeof(value))) continue;
        if (size < 0 && errno == EINTR) continue;
        break;
    }
}

static void signal_poll_thread(ProximityDevice* device) {
    const uint64_t value = 1;
    ssize_t size;
    do {
        size = write(device->wake_fd, &value, sizeof(value));
    } while (size < 0 && errno == EINTR);
    if (size < 0 && errno != EAGAIN) {
        ALOGE("eventfd write failed: %s", strerror(errno));
    }
}

static void drain_iio(ProximityDevice* device) {
    uint8_t data[kScanSize * kMaxSamples];
    for (;;) {
        const ssize_t size = read(device->iio_fd, data, sizeof(data));
        if (size > 0) continue;
        if (size < 0 && errno == EINTR) continue;
        break;
    }
    device->partial_size = 0;
}

static int configure_scan(ProximityDevice* device) {
    int result = write_sysfs_int(device->buffer_enable, 0);
    if (result < 0) return result;

    result = write_sysfs_int(device->proximity_enable, 1);
    if (result < 0) return result;

    result = write_sysfs_int(device->timestamp_enable, 1);
    if (result < 0) {
        write_sysfs_int(device->proximity_enable, 0);
        return result;
    }
    return 0;
}

static void release_scan(ProximityDevice* device) {
    if (device->buffer_enable[0] != '\0') write_sysfs_int(device->buffer_enable, 0);
    if (device->proximity_enable[0] != '\0') write_sysfs_int(device->proximity_enable, 0);
    if (device->timestamp_enable[0] != '\0') write_sysfs_int(device->timestamp_enable, 0);
}

static int activate_sensor(sensors_poll_device_t* common, int handle, int enabled) {
    if (handle != kSensorHandle || (enabled != 0 && enabled != 1)) return -EINVAL;
    ProximityDevice* device = reinterpret_cast<ProximityDevice*>(common);

    pthread_mutex_lock(&device->mutex);
    if (device->enabled == (enabled != 0)) {
        pthread_mutex_unlock(&device->mutex);
        return 0;
    }

    int result = 0;
    if (enabled) {
        drain_iio(device);
        result = write_sysfs_int(device->buffer_enable, 1);
        if (result == 0) {
            result = ultrasound_enable(0, 1);
            if (result < 0) write_sysfs_int(device->buffer_enable, 0);
        }
        if (result == 0) device->enabled = true;
    } else {
        result = write_sysfs_int(device->buffer_enable, 0);
        const int audio_result = ultrasound_enable(0, 0);
        drain_iio(device);
        device->enabled = false;
        if (result == 0 && audio_result < 0) result = audio_result;
    }
    pthread_mutex_unlock(&device->mutex);

    signal_poll_thread(device);
    return result;
}

static void fill_proximity_event(sensors_event_t* event, uint16_t value, int64_t timestamp) {
    memset(event, 0, sizeof(*event));
    event->version = sizeof(*event);
    event->sensor = kSensorHandle;
    event->type = SENSOR_TYPE_PROXIMITY;
    event->timestamp = timestamp;
    event->distance = value == 0 ? 5.0f : 0.0f;
}

static void fill_flush_event(sensors_event_t* event) {
    memset(event, 0, sizeof(*event));
    event->version = sizeof(*event);
    event->type = SENSOR_TYPE_META_DATA;
    event->meta_data.what = META_DATA_FLUSH_COMPLETE;
    event->meta_data.sensor = kSensorHandle;
}

static int read_sensor_events(ProximityDevice* device, sensors_event_t* events, int count) {
    const size_t max_samples = count < static_cast<int>(kMaxSamples)
                                   ? static_cast<size_t>(count)
                                   : kMaxSamples;
    size_t bytes_to_read = max_samples * kScanSize;
    if (bytes_to_read < kScanSize) bytes_to_read = kScanSize;

    uint8_t buffer[kScanSize * (kMaxSamples + 1)];
    const size_t previous_size = device->partial_size;
    if (previous_size >= kScanSize) return -EIO;
    memcpy(buffer, device->partial, previous_size);

    ssize_t bytes_read;
    do {
        bytes_read = read(device->iio_fd, buffer + previous_size, bytes_to_read);
    } while (bytes_read < 0 && errno == EINTR);
    if (bytes_read < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
        return -errno;
    }
    if (bytes_read == 0) return -EIO;

    const size_t total_size = previous_size + static_cast<size_t>(bytes_read);
    const size_t sample_count = total_size / kScanSize;
    const size_t remaining_size = total_size % kScanSize;
    if (sample_count > max_samples) return -EOVERFLOW;

    if (remaining_size != 0) {
        memcpy(device->partial, buffer + sample_count * kScanSize, remaining_size);
    }
    device->partial_size = remaining_size;

    for (size_t index = 0; index < sample_count; ++index) {
        IioProximitySample sample;
        memcpy(&sample, buffer + index * kScanSize, sizeof(sample));

        if (sample.timestamp <= 0) {
            timespec now{};
            if (clock_gettime(CLOCK_BOOTTIME, &now) == 0) {
                sample.timestamp = static_cast<int64_t>(now.tv_sec) * 1000000000LL + now.tv_nsec;
            }
        }
        fill_proximity_event(&events[index], sample.value, sample.timestamp);
    }
    return static_cast<int>(sample_count);
}

static int poll_sensor(sensors_poll_device_t* common, sensors_event_t* events, int count) {
    if (events == nullptr || count <= 0) return -EINVAL;
    ProximityDevice* device = reinterpret_cast<ProximityDevice*>(common);

    for (;;) {
        pthread_mutex_lock(&device->mutex);
        if (device->pending_flushes != 0) {
            --device->pending_flushes;
            pthread_mutex_unlock(&device->mutex);
            drain_wake_fd(device->wake_fd);
            fill_flush_event(&events[0]);
            return 1;
        }
        const bool enabled = device->enabled;
        const int wake_fd = device->wake_fd;
        const int iio_fd = device->iio_fd;
        pthread_mutex_unlock(&device->mutex);

        pollfd descriptors[2]{};
        descriptors[0].fd = wake_fd;
        descriptors[0].events = POLLIN;
        nfds_t descriptor_count = 1;
        if (enabled) {
            descriptors[1].fd = iio_fd;
            descriptors[1].events = POLLIN;
            descriptor_count = 2;
        }

        int result;
        do {
            result = poll(descriptors, descriptor_count, -1);
        } while (result < 0 && errno == EINTR);
        if (result < 0) return -errno;

        if ((descriptors[0].revents & (POLLIN | POLLERR | POLLHUP | POLLNVAL)) != 0) {
            drain_wake_fd(wake_fd);
            continue;
        }
        if (!enabled) continue;
        if ((descriptors[1].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) return -EIO;
        if ((descriptors[1].revents & POLLIN) == 0) continue;

        pthread_mutex_lock(&device->mutex);
        if (!device->enabled) {
            pthread_mutex_unlock(&device->mutex);
            continue;
        }
        result = read_sensor_events(device, events, count);
        pthread_mutex_unlock(&device->mutex);
        if (result != 0) return result;
    }
}

static int set_delay(sensors_poll_device_t*, int handle, int64_t) {
    return handle == kSensorHandle ? 0 : -EINVAL;
}

static int batch_sensor(sensors_poll_device_1_t*, int handle, int, int64_t, int64_t) {
    return handle == kSensorHandle ? 0 : -EINVAL;
}

static int flush_sensor(sensors_poll_device_1_t* hal, int handle) {
    if (handle != kSensorHandle) return -EINVAL;
    ProximityDevice* device = reinterpret_cast<ProximityDevice*>(hal);

    pthread_mutex_lock(&device->mutex);
    if (!device->enabled) {
        pthread_mutex_unlock(&device->mutex);
        return -EINVAL;
    }
    if (device->pending_flushes == UINT32_MAX) {
        pthread_mutex_unlock(&device->mutex);
        return -EOVERFLOW;
    }
    ++device->pending_flushes;
    pthread_mutex_unlock(&device->mutex);

    signal_poll_thread(device);
    return 0;
}

static int inject_sensor_data(sensors_poll_device_1_t*, const sensors_event_t*) {
    return -EINVAL;
}

static int close_device(hw_device_t* common) {
    ProximityDevice* device = reinterpret_cast<ProximityDevice*>(common);
    if (device->enabled) activate_sensor(&device->hal.v0, kSensorHandle, 0);
    release_scan(device);
    if (device->iio_fd >= 0) close(device->iio_fd);
    if (device->wake_fd >= 0) close(device->wake_fd);
    pthread_mutex_destroy(&device->mutex);
    delete device;
    return 0;
}

static int open_device(const hw_module_t* module, const char*, hw_device_t** output) {
    if (output == nullptr) return -EINVAL;
    ProximityDevice* device = new (std::nothrow) ProximityDevice{};
    if (device == nullptr) return -ENOMEM;

    int result = pthread_mutex_init(&device->mutex, nullptr);
    if (result != 0) {
        delete device;
        return -result;
    }

    result = find_iio_device(device);
    if (result < 0) {
        pthread_mutex_destroy(&device->mutex);
        delete device;
        return result;
    }

    result = configure_scan(device);
    if (result < 0) {
        release_scan(device);
        pthread_mutex_destroy(&device->mutex);
        delete device;
        return result;
    }

    device->iio_fd = open(device->iio_node, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (device->iio_fd < 0) {
        result = -errno;
        release_scan(device);
        pthread_mutex_destroy(&device->mutex);
        delete device;
        return result;
    }

    device->wake_fd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (device->wake_fd < 0) {
        result = -errno;
        close(device->iio_fd);
        release_scan(device);
        pthread_mutex_destroy(&device->mutex);
        delete device;
        return result;
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
    static const sensor_t sensors[] = {
        {"Xiaomi Proximity", "Xiaomi", 1, kSensorHandle, SENSOR_TYPE_PROXIMITY,
         5.0f, 1.0f, 0.5f, 0, 0, 0, "android.sensor.proximity", "", 0,
         SENSOR_FLAG_WAKE_UP | SENSOR_FLAG_ON_CHANGE_MODE, {nullptr, nullptr}},
    };
    *list = sensors;
    return sizeof(sensors) / sizeof(sensors[0]);
}

static int set_operation_mode(unsigned int mode) {
    return mode == 0 ? 0 : -EINVAL;
}

static hw_module_methods_t module_methods = {open_device};

}  // namespace

sensors_module_t HAL_MODULE_INFO_SYM = {
    {HARDWARE_MODULE_TAG, HARDWARE_MODULE_API_VERSION(1, 0), HARDWARE_HAL_API_VERSION,
     SENSORS_HARDWARE_MODULE_ID, "Xiaomi MIUS Proximity Sensor HAL", "Xiaomi",
     &module_methods, nullptr, {0}},
    get_sensors_list,
    set_operation_mode,
};
