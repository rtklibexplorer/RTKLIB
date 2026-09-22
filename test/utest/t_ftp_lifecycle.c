/* Exercise private download lifecycle with real threads and a controlled
 * command executor. No network, wget or wall-clock scheduling is required.
 */
#define _DEFAULT_SOURCE
#include <pthread.h>
int test_thread_create(pthread_t *, const pthread_attr_t *,
                       void *(*)(void *), void *);
#define execcmd test_download
#define pthread_create test_thread_create
#include "../../src/stream.c"
#undef pthread_create
#undef execcmd
#include <errno.h>

static pthread_mutex_t gate=PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t event=PTHREAD_COND_INITIALIZER;
static int entered,released,download_error,closing,closed,create_error,calls;
static char directory[]="/tmp/rtklib-ftp-test-XXXXXX",localfile[1024];
static pthread_t worker;

int test_thread_create(pthread_t *thread, const pthread_attr_t *attr,
                       void *(*start)(void *), void *arg)
{
    return create_error?EAGAIN:pthread_create(thread,attr,start,arg);
}

int test_download(const char *cmd)
{
    FILE *fp;
    int error;
    (void)cmd;
    pthread_mutex_lock(&gate);
    worker=pthread_self();
    entered=1;
    calls++;
    pthread_cond_broadcast(&event);
    while (!released) pthread_cond_wait(&event,&gate);
    error=download_error;
    pthread_mutex_unlock(&gate);
    if (error) return error;
    fp=fopen(localfile,"wb");
    if (!fp) return 1;
    fputs("downloaded product\n",fp);
    return fclose(fp);
}

static int check(int condition, const char *message)
{
    if (!condition) fprintf(stderr,"FAIL: %s\n",message);
    return condition;
}

static void release_download(void)
{
    pthread_mutex_lock(&gate);
    released=1;
    pthread_cond_broadcast(&event);
    pthread_mutex_unlock(&gate);
}

static void begin_download(ftp_t *ftp)
{
    uint8_t buff[2048];
    char msg[2048];
    pthread_mutex_lock(&gate);
    entered=released=0;
    pthread_mutex_unlock(&gate);
    ftp->tnext.time=0;
    ftp->tnext.sec=0;
    readftp(ftp,buff,sizeof(buff),msg);
    pthread_mutex_lock(&gate);
    while (!entered) pthread_cond_wait(&event,&gate);
    pthread_mutex_unlock(&gate);
}

static int finish_download(ftp_t *ftp, int error)
{
    uint8_t buff[2048]={0};
    char msg[2048]={0},expected[2048];
    int i,n,state;
    release_download();
    for (i=0;i<2000;i++) {
        state=stateftp(ftp);
        statexftp(ftp,msg);
        if (state==(error?-1:3)) {
            /* State 3 also denotes an active download; read until result. */
            n=readftp(ftp,buff,sizeof(buff),msg);
            if (error&&strstr(msg,"error (7)")) return 1;
            if (n>0) {
                snprintf(expected,sizeof(expected),"%s\r\n",localfile);
                return check(!error&&n==(int)strlen(expected)&&!memcmp(buff,expected,n),
                             "download result path");
            }
        }
        sleepms(1);
    }
    return check(0,"download completion timeout");
}

static void *close_download(void *arg)
{
    pthread_mutex_lock(&gate);
    closing=1;
    pthread_cond_broadcast(&event);
    pthread_mutex_unlock(&gate);
    closeftp((ftp_t *)arg);
    pthread_mutex_lock(&gate);
    closed=1;
    pthread_cond_broadcast(&event);
    pthread_mutex_unlock(&gate);
    return NULL;
}

static int run_case(const char *name, int proto)
{
    char msg[2048]={0};
    uint8_t buff[2048];
    ftp_t *ftp=openftp("example.invalid/product",proto,msg);
    int ok=1,i;
    if (!check(ftp!=NULL,"openftp")) return 0;
    if (!strcmp(name,"inactive")) {
        ok=check(stateftp(ftp)==2,"initial state");
    }
    else if (!strcmp(name,"cached")) {
        FILE *fp=fopen(localfile,"wb");
        if (!fp) abort();
        fputs("cached product\n",fp);
        fclose(fp);
        ftp->tnext.time=0;
        readftp(ftp,buff,sizeof(buff),msg);
        ok=finish_download(ftp,0)&&check(calls==0,"cache bypasses downloader");
    }
    else if (!strcmp(name,"missing-directory")) {
        strsetdir("");
        ftp->tnext.time=0;
        readftp(ftp,buff,sizeof(buff),msg);
        for (i=0;i<2000&&stateftp(ftp)!=-1;i++) sleepms(1);
        ok=check(stateftp(ftp)==-1,"missing directory error state");
        readftp(ftp,buff,sizeof(buff),msg);
        ok=check(strstr(msg,"error (11)")!=NULL,"missing directory diagnostic")&&ok;
    }
    else if (!strcmp(name,"create-error")) {
        create_error=1;
        ftp->tnext.time=0;
        readftp(ftp,buff,sizeof(buff),msg);
        ok=check(!strcmp(msg,"ftp thread error"),"thread creation failure");
        create_error=0;
    }
    else if (!strcmp(name,"close-active")) {
        pthread_t closer;
        struct timespec deadline;
        int early;
        begin_download(ftp);
        if (pthread_create(&closer,NULL,close_download,ftp)) abort();
        pthread_mutex_lock(&gate);
        while (!closing) pthread_cond_wait(&event,&gate);
        clock_gettime(CLOCK_REALTIME,&deadline);
        deadline.tv_sec++;
        while (!closed) {
            if (pthread_cond_timedwait(&event,&gate,&deadline)==ETIMEDOUT) break;
        }
        early=closed;
        pthread_mutex_unlock(&gate);
        release_download();
        pthread_join(closer,NULL);
        /* Clean up the old implementation's still-running worker when testing
         * the regression baseline; the fixed close already joins it. */
        if (early) pthread_join(worker,NULL);
        return check(!early,"close must wait for active worker");
    }
    else if (!strcmp(name,"success")||!strcmp(name,"error")) {
        download_error=!strcmp(name,"error")?7:0;
        for (i=0;i<3;i++) {
            unlink(localfile);
            begin_download(ftp);
            ok=finish_download(ftp,download_error)&&ok;
            if (!ok) break;
        }
        ok=check(calls==3,"three separate workers")&&ok;
    }
    else ok=check(0,"unknown scenario");
    closeftp(ftp);
    return ok;
}

int main(int argc, char **argv)
{
    int ok;
    if (argc!=3||!mkdtemp(directory)) return 1;
    snprintf(localfile,sizeof(localfile),"%s/product",directory);
    strsetdir(directory);
    ok=run_case(argv[1],!strcmp(argv[2],"http"));
    unlink(localfile);
    rmdir(directory);
    return ok?0:1;
}
