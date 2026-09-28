// Deterministic RTK server lifecycle tests with real worker threads.
// Include the implementation only to inject startup failures and delay worker
// entry. Streams are disabled, so no receiver data or network is needed.
#define _DEFAULT_SOURCE
#include <errno.h>
#include <pthread.h>
#include <stdlib.h>

#include "../../src/rtklib.h"
int test_create(pthread_t *, const pthread_attr_t *, void *(*)(void *), void *);
int test_join(pthread_t, void **);
void *test_calloc(size_t, size_t);
void *test_malloc(size_t);
int test_raw(raw_t *, int);
int test_rtcm(rtcm_t *);
#define pthread_create test_create
#define pthread_join test_join
#define calloc test_calloc
#define malloc test_malloc
#define init_raw test_raw
#define init_rtcm test_rtcm
#include "../../src/rtksvr.c"
#undef init_raw
#undef init_rtcm
#undef malloc
#undef calloc
#undef pthread_join
#undef pthread_create

static pthread_mutex_t gate = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t event = PTHREAD_COND_INITIALIZER;
static int release_worker, worker_done, fail_create, fail_allocation, observer_stop;
static int fail_malloc, malloc_count, fail_raw, fail_rtcm, fail_open;
static void *(*worker_function)(void *);
static void *worker_argument;

static void require(int condition, const char *message) {
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

static int running(rtksvr_t *svr) {
    rtksvrlock(svr);
    int state = svr->state;
    rtksvrunlock(svr);
    return state;
}

static void *delayed_worker(void *unused) {
    (void)unused;
    pthread_mutex_lock(&gate);
    while (!release_worker) pthread_cond_wait(&event, &gate);
    pthread_mutex_unlock(&gate);
    void *result = worker_function(worker_argument);
    pthread_mutex_lock(&gate);
    worker_done = 1;
    pthread_cond_broadcast(&event);
    pthread_mutex_unlock(&gate);
    return result;
}

int test_create(pthread_t *thread, const pthread_attr_t *attr, void *(*start)(void *), void *arg) {
    if (fail_create) return EAGAIN;
    worker_function = start;
    worker_argument = arg;
    return pthread_create(thread, attr, delayed_worker, NULL);
}

int test_join(pthread_t thread, void **result) {
    // An immediate stop must publish state zero before the worker can run.
    pthread_mutex_lock(&gate);
    release_worker = 1;
    pthread_cond_broadcast(&event);
    pthread_mutex_unlock(&gate);
    return pthread_join(thread, result);
}

void *test_calloc(size_t n, size_t size) {
    if (fail_allocation && n == MAXOBS * 2 && size == sizeof(obsd_t)) return NULL;
    return calloc(n, size);
}

void *test_malloc(size_t size) {
    if (fail_malloc && ++malloc_count == fail_malloc) return NULL;
    return malloc(size);
}

int test_raw(raw_t *raw, int format) { return fail_raw ? 0 : init_raw(raw, format); }

int test_rtcm(rtcm_t *rtcm) { return fail_rtcm ? 0 : init_rtcm(rtcm); }

static int start_server(rtksvr_t *svr) {
    int streams[MAXSTRRTK] = {0}, formats[3] = {STRFMT_RTCM3, STRFMT_RTCM3, STRFMT_RTCM3};
    const char *paths[MAXSTRRTK] = {"", "", "", "", "", "", "", ""};
    const char *commands[3] = {NULL, NULL, NULL}, *options[3] = {"", "", ""};
    double position[3] = {0};
    prcopt_t processing = prcopt_default;
    solopt_t solutions[2] = {solopt_default, solopt_default};
    char error[2048] = {0};

    solutions[0].posf = solutions[1].posf = SOLF_STAT;
    if (fail_open) {
        streams[1] = STR_FILE;
        paths[1] = "/dev/null/not-a-directory";
    }
    int ret = rtksvrstart(svr, 1, 4096, streams, paths, formats, 0, commands, commands, options,
                          1000, 0, position, &processing, solutions, NULL, error);
    if (!ret && !fail_create) fprintf(stderr, "start error: %s\n", error);
    return ret;
}

static void require_cleanup(rtksvr_t *svr) {
    require(!running(svr), "server stopped");
    for (int i = 0; i < 3; i++) {
        require(!svr->buff[i] && !svr->pbuf[i], "input buffers released");
        require(!svr->raw[i].obs.data && !svr->rtcm[i].obs.data, "decoders released");
    }
    for (int i = 0; i < 2; i++) require(!svr->sbuf[i], "output buffers released");
}

static void *observe(void *arg) {
    rtksvr_t *svr = (rtksvr_t *)arg;
    gtime_t time;
    int sat[MAXSAT], vsat[MAXSAT][NFREQ], status[MAXSTRRTK];
    double az[MAXSAT], el[MAXSAT], snr[MAXSAT][NFREQ];
    char msg[MAXSTRRTK * MAXSTRMSG] = {0};
    for (;;) {
        pthread_mutex_lock(&gate);
        if (observer_stop) {
            pthread_mutex_unlock(&gate);
            break;
        }
        pthread_mutex_unlock(&gate);
        running(svr);
        rtksvrostat(svr, 0, &time, sat, az, el, snr, vsat);
        rtksvrsstat(svr, status, msg);
    }
    return NULL;
}

int main(int argc, char **argv) {
    const char *commands[3] = {NULL, NULL, NULL};
    require(argc == 2, "scenario argument");
    rtksvr_t *svr = (rtksvr_t *)calloc(1, sizeof(*svr));
    require(svr != NULL && rtksvrinit(svr), "server initialization");

    if (!strcmp(argv[1], "buffer-allocation-failure")) {
        for (int i = 1; i <= 8; i++) {
            fail_malloc = i;
            malloc_count = 0;
            require(!start_server(svr), "buffer allocation failure reported");
            require_cleanup(svr);
        }
        fail_malloc = 0;
    } else if (!strcmp(argv[1], "raw-init-failure") || !strcmp(argv[1], "rtcm-init-failure")) {
        fail_raw = !strcmp(argv[1], "raw-init-failure");
        fail_rtcm = !strcmp(argv[1], "rtcm-init-failure");
        require(!start_server(svr), "decoder failure reported");
        require_cleanup(svr);
        fail_raw = fail_rtcm = 0;
    } else if (!strcmp(argv[1], "stream-open-failure")) {
        fail_open = 1;
        require(!start_server(svr), "stream open failure reported");
        require_cleanup(svr);
        fail_open = 0;
    } else if (!strcmp(argv[1], "create-failure")) {
        fail_create = 1;
        require(!start_server(svr), "creation failure reported");
        require_cleanup(svr);
        fail_create = 0;
    } else if (!strcmp(argv[1], "worker-allocation-failure")) {
        fail_allocation = 1;
        require(start_server(svr), "worker created");
        pthread_mutex_lock(&gate);
        release_worker = 1;
        pthread_cond_broadcast(&event);
        while (!worker_done) pthread_cond_wait(&event, &gate);
        pthread_mutex_unlock(&gate);
        require(!running(svr), "failed worker clears running state");
        rtksvrstop(svr, commands);
        require_cleanup(svr);
        fail_allocation = 0;
    } else
        require(!strcmp(argv[1], "immediate-stop") || !strcmp(argv[1], "polling"),
                "known scenario");

    for (int i = 0; i < 10; i++) {
        pthread_t observer;
        pthread_mutex_lock(&gate);
        release_worker = worker_done = 0;
        pthread_mutex_unlock(&gate);
        require(start_server(svr), "start/restart");
        if (strcmp(argv[1], "polling")) {
            require(running(svr), "running state published before worker entry");
        } else {
            pthread_mutex_lock(&gate);
            release_worker = 1;
            pthread_cond_broadcast(&event);
            pthread_mutex_unlock(&gate);
            for (int attempts = 0; attempts < 2000 && !running(svr); attempts++) sleepms(1);
            require(running(svr), "worker running");
        }
        require(!start_server(svr), "duplicate start rejected");
        if (!strcmp(argv[1], "polling")) {
            observer_stop = 0;
            require(!pthread_create(&observer, NULL, observe, svr), "observer created");
            pthread_mutex_lock(&gate);
            release_worker = 1;
            pthread_cond_broadcast(&event);
            pthread_mutex_unlock(&gate);
            sleepms(5);
        }
        rtksvrstop(svr, commands);
        if (!strcmp(argv[1], "polling")) {
            pthread_mutex_lock(&gate);
            observer_stop = 1;
            pthread_mutex_unlock(&gate);
            pthread_join(observer, NULL);
        }
        require_cleanup(svr);
    }
    rtksvrfree(svr);
    pthread_mutex_destroy(&svr->lock);
    for (int i = 0; i < MAXSTRRTK; i++) pthread_mutex_destroy(&svr->stream[i].lock);
    free(svr);
    return 0;
}
