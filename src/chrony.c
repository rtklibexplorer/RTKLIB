#define _POSIX_C_SOURCE 199506
#include <time.h>        		/* for timespec */
#include <sys/time.h>
#include <stdio.h>              /* for printf() */
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <unistd.h>
#include <math.h>
#include <ctype.h>              /* for isdigit() */
#include <libgen.h>		/* for basename() */
#include <sys/un.h>
#include "rtklib.h"  
#include <stdint.h>		/* for intmax_t	*/

/* NOTE Definitions */
#define GPS_PATH_MAX	128	/* for names like /dev/serial/by-id/... */
#define MONTHSPERYEAR	12	/* months per calendar year 		*/
#define LEAP_NOWARNING  0x0     /* normal, no leap second warning 	*/
#define CLIENT_DATE_MAX	24
#define NS_IN_SEC	1000000000LL     /* nanoseconds in a second 	*/
#define US_IN_SEC	1000000LL        /* microseconds in a second 	*/
#define MS_IN_SEC	1000LL           /* milliseconds in a second 	*/


/* netlib_connectsock() errno return values 				*/
#define NL_NOSERVICE	-1	/* can't get service entry 		*/
#define NL_NOHOST	-2	/* can't get host entry 		*/
#define NL_NOPROTO	-3	/* can't get protocol entry 		*/
#define NL_NOSOCK	-4	/* can't create socket 			*/
#define NL_NOSOCKOPT	-5	/* error SETSOCKOPT SO_REUSEADDR 	*/
#define NL_NOCONNECT	-6	/* can't connect to host/socket pair 	*/
#define SHM_NOSHARED	-7	/* shared-memory segment not available 	*/
#define SHM_NOATTACH	-8	/* shared-memory attach failed 		*/
#define DBUS_FAILURE	-9	/* DBUS initialization failure 		*/

/* Chrony Socket definitions 						*/
int fd_chrony = -1;
int debug = 0;
#define SOCK_MAGIC 0x534f434b
struct sock_sample {
    struct timeval tv;
    double offset;
    int pulse;
    int leap;    		/* notify that a leap second is upcoming */
    int _pad;
    int magic;      		/* must be SOCK_MAGIC */
};

#define SUN_LEN(ptr) ((size_t) (((struct sockaddr_un *) 0)->sun_path)        \
              + strlen ((ptr)->sun_path))

/* NOTE Type definitions */
typedef struct timespec timespec_t;	/* Unix time as sec, nsec */
typedef int socket_t;
#define BAD_SOCKET(s)		((s) == -1)
#define INVALIDATE_SOCKET(s)	do { s = -1; } while (0)

#ifndef TIMEDELTA_DEFINED
#define TIMEDELTA_DEFINED
#define TIMESPEC_LEN	22	/* required length of a timespec buffer */

struct timedelta_t {
    timespec_t	real;
    timespec_t	clock;
};
#endif /* TIMEDELTA_DEFINED */

/* NOTE Data structures */
struct gpsd_errout_t {
    int debug;				/* lexer debug level */
    void (*report)(const char *);	/* reporting hook for lexer errors */
    char *label;
};

struct gps_context_t {
    int leap_notify;			/* notification state from subframe */
    struct gpsd_errout_t errout;	/* debug verbosity level and hook */
};

struct devconfig_t   { char *path[GPS_PATH_MAX]; };
struct gps_data_t    { struct devconfig_t dev;   };

struct gps_device_t  {
/* session object, encapsulates all global state */
    struct gps_data_t		gpsdata; 
    struct gps_context_t	*context; 
    int chronyfd;			/* for talking to chrony */
};

/* normalize a timeval */
#define TV_NORM(tv)  \
    do { \
	if ( US_IN_SEC <= (tv)->tv_usec ) { \
	    (tv)->tv_usec -= US_IN_SEC; \
	    (tv)->tv_sec++; \
	} else if ( 0 > (tv)->tv_usec ) { \
	    (tv)->tv_usec += US_IN_SEC; \
	    (tv)->tv_sec--; \
	} \
    } while (0)

/* convert timespec to timeval, with rounding */
#define TSTOTV(tv, ts) \
    do { \
	(tv)->tv_sec = (ts)->tv_sec; \
	(tv)->tv_usec = ((ts)->tv_nsec + 500)/1000; \
        TV_NORM( tv ); \
    } while (0)

/* subtract two timespec, return a double */
#define TS_SUB_D(ts1, ts2) \
    ((double)((ts1)->tv_sec - (ts2)->tv_sec) + \
    ((double)((ts1)->tv_nsec - (ts2)->tv_nsec) * 1e-9))

/* true if normalized timespec equal or greater than zero */
#define TS_GEZ(ts) (0 <= (ts)->tv_sec && 0 <= (ts)->tv_nsec)

#ifndef HAVE_STRLCPY
/*
 * '_cups_strlcpy()' - Safely copy two strings.
 */
size_t                  	/* O - Length of string */
strlcpy(char 	   *dst,	/* O - Destination string */
        const char *src,  	/* I - Source string */
        size_t     size) 	/* I - Size of destination string buffer */
{
        size_t     srclen;     	/* Length of source string */
 /*
  * Figure out how much room is needed...
  */
  size --;
  srclen = strlen(src);

 /*
  * Copy the appropriate amount...
  */
  if (srclen > size)
    srclen = size;

  memcpy(dst, src, srclen);
  dst[srclen] = '\0';

  return (srclen);
}
#endif /* !HAVE_STRLCPY */

/* Convert a normalized timespec to a nice string
 * put in it *buf, buf should be at least 22 bytes
 *
 * the returned buffer will look like, shortest case:
 *    sign character ' ' or '-'
 *    one digit of seconds
 *    decmal point '.'
 *    9 digits of nanoSec
 *
 * So 12 chars, like this: "-0.123456789"
 *
 * Probable worst case is 10 digits of seconds,
 * but standards do not provide hard limits to time_t
 * So 21 characters like this: "-2147483647.123456789"
 *
 * date --date='@2147483647' is: Mon Jan 18 19:14:07 PST 2038
 * date --date='@9999999999' is: Sat Nov 20 09:46:39 PST 2286
 *
 */
const char *timespec_str(const struct timespec *ts, char *buf, size_t buf_size)
{
    char sign = ' ';

    if (!TS_GEZ(ts)) {
        sign = '-';
    }

    /* %lld and (long long) because some time_t is bigger than a long
     * mostly on 32-bit systems. */
    (void)snprintf(buf, buf_size, "%c%lld.%09ld",
                   sign,
                   (long long)llabs(ts->tv_sec),
                   (long)labs(ts->tv_nsec));
    return  buf;
}

/* acquire a connection to an existing Unix-domain socket */
socket_t netlib_localsocket(const char *sockfile, int socktype)
{
    int sock;

    if ((sock = socket(AF_UNIX, socktype, 0)) < 0) {
	return -1;
    } else {
	struct sockaddr_un saddr;

	memset(&saddr, 0, sizeof(struct sockaddr_un));
	saddr.sun_family = AF_UNIX;
	(void)strlcpy(saddr.sun_path,
		      sockfile,
		      sizeof(saddr.sun_path));

	if (connect(sock, (struct sockaddr *)&saddr, SUN_LEN(&saddr)) < 0) {
	    (void)close(sock);
	    return -2;
	}

	return sock;
    }
}

/* for chrony SOCK interface, which allows nSec timekeeping */
static void init_chrony_socket(struct gps_device_t *session, rtk_t *rtk)
{
    /* open the chrony socket */
    char chrony_path[GPS_PATH_MAX];

    session->chronyfd = -1;

    if ( 0 == getuid() ) {
	/* this case will fire on command-line devices;
	 * they're opened before priv-dropping.  Matters because
         * only root can use /var/run.
	 */
	(void)snprintf(chrony_path, sizeof (chrony_path),
		"/var/run/chrony.%s.sock", basename(*session->gpsdata.dev.path)); 
    } else {
	(void)snprintf(chrony_path, sizeof (chrony_path),
		"/tmp/chrony.%s.sock", 	basename(*session->gpsdata.dev.path)); 
    }

    if (access(chrony_path, F_OK) != 0) {
        printf(	"PPS:%s chrony socket %s doesn't exist\n",
		*session->gpsdata.dev.path, chrony_path); 
	printf("PPS: Chrony RTK socket doesn't exist\n");
    } else {
	session->chronyfd = netlib_localsocket(chrony_path, SOCK_DGRAM);
	if (session->chronyfd < 0)
            printf("PPS:%s connect chrony socket failed: %s, error: %d, errno: %d/%s\n",
		     *session->gpsdata.dev.path, 
		     chrony_path, session->chronyfd, errno, strerror(errno));
	else {
            printf(  "PPS:%s using chrony socket: %s\n",
                     *session->gpsdata.dev.path, 
		     chrony_path);
            rtk->chrony_delta = -100;
        }
    }
}

/* td is the real time and clock time of the edge */
/* offset is actual_ts - clock_ts */
static void update_chrony(struct gps_device_t *session, struct timedelta_t *td, rtk_t *rtk)
{
    char real_str[TIMESPEC_LEN];
    char clock_str[TIMESPEC_LEN];
    struct sock_sample sample;
    struct tm tm;
    int r;
    int leap_notify = session->context->leap_notify; 

    /* printf("td.tvsec=%ld",td->real.tv_sec); */

    /*
     * insist that leap seconds only happen in june and december
     * GPS emits leap pending for 3 months prior to insertion
     * NTP expects leap pending for only 1 month prior to insertion
     * Per http://bugs.ntp.org/1090
     *
     * ITU-R TF.460-6, Section 2.1, says leap seconds can be primarily
     * in Jun/Dec but may be in March or September
     */
    (void)gmtime_r( &(td->real.tv_sec), &tm);
    if ( 5 != tm.tm_mon && 11 != tm.tm_mon ) {
        /* Not june, not December, no way */
        leap_notify = LEAP_NOWARNING; 
    }

    /* chrony expects tv-sec since Jan 1970 */
    sample.pulse = 0;
    sample.leap = leap_notify;
    sample.magic = SOCK_MAGIC;
    /* chronyd wants a timeval, not a timspec, not to worry, it is
     * just the top of the second */
    TSTOTV(&sample.tv, &td->clock);
    /* calculate the offset as a timespec to not lose precision */
    /* if tv_sec greater than 2 then tv_nsec loses precision, but
     * not a big deal as slewing will be required */
    sample.offset = TS_SUB_D(&td->real, &td->clock);
    sample._pad = 0;
    if (debug) printf(  "PPS chrony_send %s @ %s Offset: %0.9f\n",
             timespec_str(&td->real, real_str, sizeof(real_str)),
             timespec_str(&td->clock, clock_str, sizeof(clock_str)),
             sample.offset); 
    r = send(session->chronyfd, &sample, sizeof(sample), 0);
    rtk->chrony_delta = sample.offset;
    if (r < 0) { 
        if (debug) printf("Could not send data fd=%d error=%d : %s", session->chronyfd, errno, strerror(errno)); 
        fd_chrony = -1;
        rtk->chrony_delta = -100;
        }
}

/* Main chrony update function */
void upd_chrony(gtime_t RcvClk, double tick, int func_debug, rtk_t *rtk)
{
    /* variable definition */
    struct gps_device_t*  session = NULL;
    struct gps_context_t gps_context;
    session = (struct gps_device_t*) 
        malloc(sizeof(struct gps_device_t));
    gps_context.leap_notify = LEAP_NOWARNING;
    session->context = &gps_context; 
    *session->gpsdata.dev.path = "ttyRTK";
    debug=func_debug; 

    double tt;
    struct timedelta_t* td;
    td = (struct timedelta_t*) 
        malloc(sizeof(struct timedelta_t)); 

/*    tt=(int)(tickget()-tick)/1000.0+DTTOL;			/* default: 0.025 */
/*    printf("ticks=%d %f\n",tick,tt); */

    if ( fd_chrony < 0 ) { 
        /* Initialize UNIX socket connection 	*/
        session->chronyfd = -1;
        init_chrony_socket(session, rtk);
        fd_chrony = session->chronyfd;
    } 

    /* Update Chrony socket 		*/
    session->chronyfd = fd_chrony;
    clock_gettime(CLOCK_REALTIME, &td->clock);
    td->real.tv_sec = (intmax_t)RcvClk.time;
    td->real.tv_nsec = RcvClk.sec*NS_IN_SEC;
    update_chrony(session, td, rtk); 
}
