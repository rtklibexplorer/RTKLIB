// Stream-server lifecycle tests: real threads, local files and controlled
// scheduling at worker entry and inside peek. No network is required.
#define _DEFAULT_SOURCE
#include <errno.h>
#include <unistd.h>

#include "../../src/rtklib.h"
int test_create(pthread_t *, const pthread_attr_t *, void *(*)(void *), void *);
int test_join(pthread_t, void **);
void test_lock(pthread_mutex_t *);
#define pthread_create test_create
#define pthread_join test_join
#undef rtklib_lock
#define rtklib_lock test_lock
// streamsvr.c defines its own feature-test macro. Headers are loaded above.
#undef _POSIX_C_SOURCE
#include "../../src/streamsvr.c"
#undef pthread_create
#undef pthread_join

static pthread_mutex_t gate = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t event = PTHREAD_COND_INITIALIZER;
static int release_worker, fail_create, pause_peek, peek_locked, allow_peek;
static int stop_started, stop_done, observer_stop, peek_count;
static pthread_t peek_thread;
static void *(*worker_function)(void *);
static void *worker_argument;
static char directory[] = "/tmp/rtklib-strsvr-test-XXXXXX";
static char input[1024], output[1024], logfile[1024];
static const uint8_t payload[] = "RTKLIB stream relay regression\n";
static uint8_t peek_data[sizeof(payload)];

static void require(int condition, const char *message) {
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

void test_lock(pthread_mutex_t *lock) {
    pthread_mutex_lock(&gate);
    int pause = pause_peek && pthread_equal(pthread_self(), peek_thread);
    pthread_mutex_unlock(&gate);
    pthread_mutex_lock(lock);
    if (pause) {
        pthread_mutex_lock(&gate);
        peek_locked = 1;
        pthread_cond_broadcast(&event);
        while (!allow_peek) pthread_cond_wait(&event, &gate);
        pthread_mutex_unlock(&gate);
    }
}

static void release(void) {
    pthread_mutex_lock(&gate);
    release_worker = 1;
    pthread_cond_broadcast(&event);
    pthread_mutex_unlock(&gate);
}

static void *delayed_worker(void *unused) {
    (void)unused;
    pthread_mutex_lock(&gate);
    while (!release_worker) pthread_cond_wait(&event, &gate);
    pthread_mutex_unlock(&gate);
    return worker_function(worker_argument);
}

int test_create(pthread_t *thread, const pthread_attr_t *attr, void *(*start)(void *), void *arg) {
    if (fail_create) return EAGAIN;
    worker_function = start;
    worker_argument = arg;
    return pthread_create(thread, attr, delayed_worker, NULL);
}

int test_join(pthread_t thread, void **result) {
    release();
    return pthread_join(thread, result);
}

static int start_server(strsvr_t *svr, int files) {
    int options[8] = {1000, 1000, 1000, 4096, 1, 0, 0, 0};
    int types[2] = {STR_NONE, STR_NONE};
    const char *paths[2] = {"", ""}, *logs[2] = {"", ""};
    const char *commands[2] = {NULL, NULL};
    strconv_t *converters[1] = {NULL};
    if (files) {
        types[0] = types[1] = STR_FILE;
        paths[0] = input;
        paths[1] = output;
    }
    if (fail_create) {
        types[0] = types[1] = STR_MEMBUF;
        paths[0] = "4096";
        paths[1] = "8192";
        logs[0] = logfile;
    }
    return strsvrstart(svr, options, types, paths, logs, converters, commands, commands, NULL);
}

static void stop_server(strsvr_t *svr) {
    const char *commands[2] = {NULL, NULL};
    strsvrstop(svr, commands);
}

static void require_cleanup(strsvr_t *svr) {
    require(!svr->state && !svr->buff && !svr->pbuf && !svr->npb, "stopped buffers");
    for (int i = 0; i < svr->nstr; i++)
        require(!svr->stream[i].port && !svr->strlog[i].port, "closed streams and logs");
    require(strsvrpeek(svr, peek_data, sizeof(peek_data)) == 0, "peek after stop");
}

static void *peek_once(void *arg) {
    pthread_mutex_lock(&gate);
    peek_thread = pthread_self();
    pause_peek = 1;
    pthread_mutex_unlock(&gate);
    peek_count = strsvrpeek((strsvr_t *)arg, peek_data, sizeof(peek_data));
    return NULL;
}

static void *stop_async(void *arg) {
    pthread_mutex_lock(&gate);
    stop_started = 1;
    pthread_cond_broadcast(&event);
    pthread_mutex_unlock(&gate);
    stop_server((strsvr_t *)arg);
    pthread_mutex_lock(&gate);
    stop_done = 1;
    pthread_cond_broadcast(&event);
    pthread_mutex_unlock(&gate);
    return NULL;
}

static void *observe(void *arg) {
    strsvr_t *svr = (strsvr_t *)arg;
    uint8_t data[64];
    int status[2], logs[2], bytes[2], bps[2];
    char msg[2 * MAXSTRMSG];
    for (;;) {
        pthread_mutex_lock(&gate);
        if (observer_stop) {
            pthread_mutex_unlock(&gate);
            break;
        }
        pthread_mutex_unlock(&gate);
        strsvrpeek(svr, data, sizeof(data));
        strsvrstat(svr, status, logs, bytes, bps, msg);
    }
    return NULL;
}

int main(int argc, char **argv) {
    require(argc == 2 && mkdtemp(directory) != NULL, "test directory");
    snprintf(input, sizeof(input), "%s/input", directory);
    snprintf(output, sizeof(output), "%s/output", directory);
    snprintf(logfile, sizeof(logfile), "%s/log", directory);
    FILE *fp = fopen(input, "wb");
    require(fp != NULL, "input file");
    require(fwrite(payload, 1, sizeof(payload), fp) == sizeof(payload), "input contents");
    fclose(fp);
    strsvr_t svr;
    strsvrinit(&svr, 1);
    if (!strcmp(argv[1], "create-failure")) {
        fail_create = 1;
        require(!start_server(&svr, 0), "creation failure reported");
        require_cleanup(&svr);
        fail_create = 0;
    } else if (!strcmp(argv[1], "peek-stop")) {
        require(start_server(&svr, 0), "start peek test");
        pthread_mutex_lock(&svr.lock);
        memcpy(svr.pbuf, payload, sizeof(payload));
        svr.npb = sizeof(payload);
        pthread_mutex_unlock(&svr.lock);
        pthread_t peeker;
        require(!pthread_create(&peeker, NULL, peek_once, &svr), "peek thread");
        pthread_mutex_lock(&gate);
        while (!peek_locked) pthread_cond_wait(&event, &gate);
        pthread_mutex_unlock(&gate);
        pthread_t stopper;
        require(!pthread_create(&stopper, NULL, stop_async, &svr), "stop thread");
        pthread_mutex_lock(&gate);
        while (!stop_started) pthread_cond_wait(&event, &gate);
        struct timespec deadline;
        clock_gettime(CLOCK_REALTIME, &deadline);
        deadline.tv_sec++;
        while (!stop_done)
            if (pthread_cond_timedwait(&event, &gate, &deadline) == ETIMEDOUT) break;
        int early = stop_done;
        allow_peek = 1;
        pthread_cond_broadcast(&event);
        pthread_mutex_unlock(&gate);
        pthread_join(peeker, NULL);
        pthread_join(stopper, NULL);
        require(!early, "stop cannot free buffers while peek holds the mutex");
        require(peek_count == sizeof(payload) && !memcmp(peek_data, payload, sizeof(payload)),
                "peek retains buffered bytes across concurrent stop");
        pause_peek = 0;
        require_cleanup(&svr);
    } else
        require(!strcmp(argv[1], "relay") || !strcmp(argv[1], "immediate-stop") ||
                    !strcmp(argv[1], "polling"),
                "known scenario");

    for (int i = 0; i < 10; i++) {
        pthread_t observer;
        release_worker = 0;
        require(start_server(&svr, !strcmp(argv[1], "relay")), "start/restart");
        require(!start_server(&svr, 0), "duplicate start rejected");
        if (!strcmp(argv[1], "relay")) {
            uint8_t data[sizeof(payload)];
            int n = 0;
            release();
            for (int attempt = 0; attempt < 2000 && n < sizeof(payload); attempt++) {
                n += strsvrpeek(&svr, data + n, sizeof(data) - n);
                if (n < sizeof(payload)) sleepms(1);
            }
            require(n == sizeof(payload) && !memcmp(data, payload, n), "relayed peek data");
        }
        if (!strcmp(argv[1], "polling")) {
            observer_stop = 0;
            require(!pthread_create(&observer, NULL, observe, &svr), "observer thread");
            release();
            sleepms(5);
        }
        stop_server(&svr);
        if (!strcmp(argv[1], "polling")) {
            pthread_mutex_lock(&gate);
            observer_stop = 1;
            pthread_mutex_unlock(&gate);
            pthread_join(observer, NULL);
        }
        require_cleanup(&svr);
        if (!strcmp(argv[1], "relay")) {
            uint8_t data[sizeof(payload) + 1];
            FILE *fp = fopen(output, "rb");
            require(fp != NULL, "output file");
            require(fread(data, 1, sizeof(data), fp) == sizeof(payload) &&
                        !memcmp(data, payload, sizeof(payload)),
                    "output contents");
            fclose(fp);
        }
    }
    pthread_mutex_destroy(&svr.lock);
    for (int i = 0; i < svr.nstr; i++) {
        pthread_mutex_destroy(&svr.stream[i].lock);
        pthread_mutex_destroy(&svr.strlog[i].lock);
    }
    unlink(input);
    unlink(output);
    unlink(logfile);
    rmdir(directory);
    return 0;
}
