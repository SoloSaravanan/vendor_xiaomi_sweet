/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal 32-bit thermal-client ABI compatibility layer for libspkrprot.
 * The Qualcomm thermal client uses fixed-size local-socket messages:
 * client name at byte 4, integer request/event value at byte 24, total
 * message size 0x3ce0. Speaker requests use the passive receive socket;
 * speaker temperature events arrive on the send-client socket.
 */

#define LOG_TAG "ThermalClientCompat32"

#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <log/log.h>
#include <thermal_client.h>

#define THERMAL_MESSAGE_SIZE 0x3ce0U
#define THERMAL_CLIENT_NAME_OFFSET 4U
#define THERMAL_VALUE_OFFSET 0x18U
#define THERMAL_CLIENT_NAME_SIZE 0x14U
#define MAX_CALLBACKS 31

#define THERMAL_SEND_CLIENT_SOCKET "/dev/socket/thermal-send-client"
#define THERMAL_RECV_CLIENT_SOCKET "/dev/socket/thermal-recv-client"
#define THERMAL_RECV_PASSIVE_SOCKET "/dev/socket/thermal-recv-passive-client"

struct callback_entry {
    int handle;
    char client_name[THERMAL_CLIENT_NAME_SIZE];
    int (*callback)(int, void *, void *);
    void *data;
};

struct request_context {
    const char *socket_path;
    uint8_t message[THERMAL_MESSAGE_SIZE];
};

static pthread_mutex_t g_lifecycle_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t g_callbacks_lock = PTHREAD_MUTEX_INITIALIZER;
static struct callback_entry g_callbacks[MAX_CALLBACKS];
static uint32_t g_handle_mask;
static pthread_t g_listener_thread;
static int g_listener_running;
static int g_listener_stopping;
static int g_wakeup_pipe[2] = {-1, -1};

static int copy_client_name(char destination[THERMAL_CLIENT_NAME_SIZE],
                            const char *source) {
    size_t length;

    if (source == NULL) {
        return -EINVAL;
    }

    length = strnlen(source, THERMAL_CLIENT_NAME_SIZE);
    if (length == 0 || length >= THERMAL_CLIENT_NAME_SIZE) {
        return -EINVAL;
    }

    memset(destination, 0, THERMAL_CLIENT_NAME_SIZE);
    memcpy(destination, source, length);
    return 0;
}

static const char *request_socket_for_client(const char *client_name) {
    if (strcmp(client_name, "spkr") == 0 ||
        strcmp(client_name, "camera_bw") == 0 ||
        strcmp(client_name, "display_bw") == 0) {
        return THERMAL_RECV_PASSIVE_SOCKET;
    }

    if (strcmp(client_name, "override") == 0 ||
        strcmp(client_name, "config_set") == 0 ||
        strcmp(client_name, "config_query") == 0) {
        return THERMAL_RECV_CLIENT_SOCKET;
    }

    return NULL;
}

static bool callback_client_supported(const char *client_name) {
    return strcmp(client_name, "camera") == 0 ||
           strcmp(client_name, "camcorder") == 0 ||
           strcmp(client_name, "spkr") == 0 ||
           strcmp(client_name, "config_query") == 0;
}

static bool event_level_valid(const char *client_name, int level) {
    /*
     * Bounds recovered from the shipped client library's per-client table:
     * camera/camcorder 0..10, speaker temperature -30..150, config_query 0..3.
     */
    if (strcmp(client_name, "camera") == 0 ||
        strcmp(client_name, "camcorder") == 0) {
        return level >= 0 && level <= 10;
    }
    if (strcmp(client_name, "spkr") == 0) {
        return level >= -30 && level <= 150;
    }
    if (strcmp(client_name, "config_query") == 0) {
        return level >= 0 && level <= 3;
    }

    return false;
}

static int connect_local_socket(const char *path) {
    struct sockaddr_un address;
    size_t path_length;
    socklen_t address_length;
    int fd;

    path_length = strlen(path);
    if (path_length >= sizeof(address.sun_path)) {
        errno = ENAMETOOLONG;
        return -1;
    }

    fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
        return -1;
    }

    memset(&address, 0, sizeof(address));
    address.sun_family = AF_UNIX;
    memcpy(address.sun_path, path, path_length + 1);
    address_length = (socklen_t)(offsetof(struct sockaddr_un, sun_path) +
                                 path_length + 1);

    if (connect(fd, (struct sockaddr *)&address, address_length) < 0) {
        int saved_errno = errno;
        close(fd);
        errno = saved_errno;
        return -1;
    }

    return fd;
}

static int send_all(int fd, const uint8_t *data, size_t length) {
    size_t sent = 0;

    while (sent < length) {
        ssize_t result = send(fd, data + sent, length - sent, MSG_NOSIGNAL);
        if (result < 0) {
            if (errno == EINTR) {
                continue;
            }
            return -errno;
        }
        if (result == 0) {
            return -EIO;
        }
        sent += (size_t)result;
    }

    return 0;
}

static void *request_worker(void *opaque) {
    struct request_context *context = opaque;

    for (;;) {
        int fd = connect_local_socket(context->socket_path);
        if (fd < 0) {
            int saved_errno = errno;
            ALOGW("thermal request socket connect failed: %d", saved_errno);
            sleep(5);
            continue;
        }

        int result = send_all(fd, context->message, sizeof(context->message));
        if (result < 0) {
            ALOGW("thermal request send failed: %d", result);
        }
        close(fd);
        break;
    }

    free(context);
    return NULL;
}

int thermal_client_request(char *client_name, int request_data) {
    char name[THERMAL_CLIENT_NAME_SIZE];
    const char *socket_path;
    struct request_context *context;
    pthread_attr_t attributes;
    pthread_t thread;
    int result;

    if (copy_client_name(name, client_name) < 0 ||
        (socket_path = request_socket_for_client(name)) == NULL) {
        return -EINVAL;
    }

    context = calloc(1, sizeof(*context));
    if (context == NULL) {
        return -EINVAL;
    }

    context->socket_path = socket_path;
    memcpy(context->message + THERMAL_CLIENT_NAME_OFFSET, name, sizeof(name));
    memcpy(context->message + THERMAL_VALUE_OFFSET, &request_data,
           sizeof(request_data));

    result = pthread_attr_init(&attributes);
    if (result == 0) {
        result = pthread_attr_setdetachstate(&attributes, PTHREAD_CREATE_DETACHED);
        if (result == 0) {
            result = pthread_create(&thread, &attributes, request_worker, context);
        }
        pthread_attr_destroy(&attributes);
    }

    if (result != 0) {
        free(context);
        return -EINVAL;
    }

    return 0;
}

static bool listener_should_stop(void) {
    bool should_stop;

    pthread_mutex_lock(&g_callbacks_lock);
    should_stop = g_listener_stopping != 0;
    pthread_mutex_unlock(&g_callbacks_lock);
    return should_stop;
}

static void consume_wakeup(void) {
    char byte;

    if (g_wakeup_pipe[0] >= 0) {
        (void)read(g_wakeup_pipe[0], &byte, sizeof(byte));
    }
}

static int wait_for_wakeup(int timeout_ms) {
    struct pollfd descriptor;
    int result;

    if (g_wakeup_pipe[0] < 0) {
        (void)poll(NULL, 0, timeout_ms);
        return 0;
    }

    descriptor.fd = g_wakeup_pipe[0];
    descriptor.events = POLLIN;
    descriptor.revents = 0;

    do {
        result = poll(&descriptor, 1, timeout_ms);
    } while (result < 0 && errno == EINTR);

    if (result > 0 && (descriptor.revents & POLLIN) != 0) {
        consume_wakeup();
        return 1;
    }

    return result < 0 ? -1 : 0;
}

static int receive_message(int fd, uint8_t *message) {
    size_t received = 0;

    while (received < THERMAL_MESSAGE_SIZE) {
        struct pollfd descriptors[2] = {
            {
                .fd = fd,
                .events = POLLIN,
                .revents = 0,
            },
            {
                .fd = g_wakeup_pipe[0],
                .events = POLLIN,
                .revents = 0,
            },
        };
        int result;

        do {
            result = poll(descriptors, 2, -1);
        } while (result < 0 && errno == EINTR);

        if (result < 0) {
            return -1;
        }

        if ((descriptors[1].revents & POLLIN) != 0) {
            consume_wakeup();
            if (listener_should_stop()) {
                return 0;
            }
        }

        if ((descriptors[0].revents & POLLIN) != 0) {
            ssize_t count = recv(fd, message + received,
                                 THERMAL_MESSAGE_SIZE - received, 0);
            if (count < 0) {
                if (errno == EINTR) {
                    continue;
                }
                return -1;
            }
            if (count == 0) {
                return -1;
            }
            received += (size_t)count;
            continue;
        }

        if ((descriptors[0].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            return -1;
        }
    }

    return 1;
}

static void dispatch_message(const uint8_t *message) {
    char client_name[THERMAL_CLIENT_NAME_SIZE];
    int level;

    memcpy(client_name, message + THERMAL_CLIENT_NAME_OFFSET,
           sizeof(client_name));
    client_name[sizeof(client_name) - 1] = '\0';
    memcpy(&level, message + THERMAL_VALUE_OFFSET, sizeof(level));

    if (!callback_client_supported(client_name) ||
        !event_level_valid(client_name, level)) {
        return;
    }

    pthread_mutex_lock(&g_callbacks_lock);
    for (size_t i = 0; i < (size_t)MAX_CALLBACKS; ++i) {
        struct callback_entry *entry = &g_callbacks[i];
        if (entry->handle != 0 && entry->callback != NULL &&
            strcmp(entry->client_name, client_name) == 0) {
            (void)entry->callback(level, entry->data, NULL);
        }
    }
    pthread_mutex_unlock(&g_callbacks_lock);
}

static void *listener_worker(void *unused) {
    uint8_t message[THERMAL_MESSAGE_SIZE];

    (void)unused;

    while (!listener_should_stop()) {
        int fd = connect_local_socket(THERMAL_SEND_CLIENT_SOCKET);
        if (fd < 0) {
            int saved_errno = errno;
            if (!listener_should_stop()) {
                ALOGW("thermal event socket connect failed: %d", saved_errno);
                (void)wait_for_wakeup(5000);
            }
            continue;
        }

        while (!listener_should_stop()) {
            int result = receive_message(fd, message);
            if (result == 1) {
                dispatch_message(message);
                continue;
            }
            if (result < 0 && !listener_should_stop()) {
                ALOGW("thermal event socket closed or returned an invalid message");
            }
            break;
        }

        close(fd);
        if (!listener_should_stop()) {
            (void)wait_for_wakeup(5000);
        }
    }

    return NULL;
}

static int allocate_callback_handle(void) {
    for (int handle = 1; handle <= MAX_CALLBACKS; ++handle) {
        if ((g_handle_mask & (1U << handle)) == 0) {
            return handle;
        }
    }
    return 0;
}

int thermal_client_register_callback(char *client_name,
                                     int (*callback)(int, void *, void *),
                                     void *data) {
    char name[THERMAL_CLIENT_NAME_SIZE];
    pthread_attr_t attributes;
    int handle;
    int result;

    if (copy_client_name(name, client_name) < 0 || callback == NULL ||
        !callback_client_supported(name)) {
        return 0;
    }

    pthread_mutex_lock(&g_lifecycle_lock);
    pthread_mutex_lock(&g_callbacks_lock);

    handle = allocate_callback_handle();
    if (handle == 0) {
        pthread_mutex_unlock(&g_callbacks_lock);
        pthread_mutex_unlock(&g_lifecycle_lock);
        return 0;
    }

    struct callback_entry *entry = &g_callbacks[handle - 1];
    entry->handle = handle;
    memcpy(entry->client_name, name, sizeof(entry->client_name));
    entry->callback = callback;
    entry->data = data;
    g_handle_mask |= 1U << handle;

    if (!g_listener_running) {
        if (pipe(g_wakeup_pipe) < 0) {
            memset(entry, 0, sizeof(*entry));
            g_handle_mask &= ~(1U << handle);
            pthread_mutex_unlock(&g_callbacks_lock);
            pthread_mutex_unlock(&g_lifecycle_lock);
            return 0;
        }

        g_listener_stopping = 0;
        result = pthread_attr_init(&attributes);
        if (result == 0) {
            result = pthread_attr_setdetachstate(&attributes,
                                                 PTHREAD_CREATE_JOINABLE);
            if (result == 0) {
                result = pthread_create(&g_listener_thread, &attributes,
                                        listener_worker, NULL);
            }
            pthread_attr_destroy(&attributes);
        }

        if (result != 0) {
            close(g_wakeup_pipe[0]);
            close(g_wakeup_pipe[1]);
            g_wakeup_pipe[0] = -1;
            g_wakeup_pipe[1] = -1;
            g_listener_stopping = 0;
            memset(entry, 0, sizeof(*entry));
            g_handle_mask &= ~(1U << handle);
            pthread_mutex_unlock(&g_callbacks_lock);
            pthread_mutex_unlock(&g_lifecycle_lock);
            return 0;
        }

        g_listener_running = 1;
    }

    pthread_mutex_unlock(&g_callbacks_lock);
    pthread_mutex_unlock(&g_lifecycle_lock);
    return handle;
}

void thermal_client_unregister_callback(int client_callback_handle) {
    int should_join = 0;
    pthread_t thread;

    if (client_callback_handle < 1 || client_callback_handle > MAX_CALLBACKS) {
        return;
    }

    pthread_mutex_lock(&g_lifecycle_lock);
    pthread_mutex_lock(&g_callbacks_lock);

    if ((g_handle_mask & (1U << client_callback_handle)) == 0) {
        pthread_mutex_unlock(&g_callbacks_lock);
        pthread_mutex_unlock(&g_lifecycle_lock);
        return;
    }

    memset(&g_callbacks[client_callback_handle - 1], 0,
           sizeof(g_callbacks[client_callback_handle - 1]));
    g_handle_mask &= ~(1U << client_callback_handle);

    if (g_handle_mask == 0 && g_listener_running) {
        g_listener_stopping = 1;
        thread = g_listener_thread;
        should_join = 1;
    }

    pthread_mutex_unlock(&g_callbacks_lock);

    if (should_join) {
        char byte = 1;
        if (g_wakeup_pipe[1] >= 0) {
            ssize_t ignored;
            do {
                ignored = write(g_wakeup_pipe[1], &byte, sizeof(byte));
            } while (ignored < 0 && errno == EINTR);
        }

        (void)pthread_join(thread, NULL);

        pthread_mutex_lock(&g_callbacks_lock);
        close(g_wakeup_pipe[0]);
        close(g_wakeup_pipe[1]);
        g_wakeup_pipe[0] = -1;
        g_wakeup_pipe[1] = -1;
        g_listener_running = 0;
        g_listener_stopping = 0;
        pthread_mutex_unlock(&g_callbacks_lock);
    }

    pthread_mutex_unlock(&g_lifecycle_lock);
}
