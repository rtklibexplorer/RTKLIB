/* temporarily move contents of this file to rtklib.h until build issues for
   RTKCONV and RTKNAVI resolved */

/* #ifndef RTKLIB_TRACE_H
#define RTKLIB_TRACE_H   */

#ifdef __cplusplus
extern "C" {
#endif

/* debug trace functions -----------------------------------------------------*/

/* #ifdef TRACE
#include "rtklib.h"

#define trace(level, ...) do { if (level <= gettracelevel()) trace_impl(level, __VA_ARGS__); } while (0)
#define tracet(level, ...) do { if (level <= gettracelevel()) tracet_impl(level, __VA_ARGS__); } while (0)
#define tracemat(level, ...) do { if (level <= gettracelevel()) tracemat_impl(level, __VA_ARGS__); } while (0)
#define traceobs(level, ...) do { if (level <= gettracelevel()) traceobs_impl(level, __VA_ARGS__); } while (0)
#define tracenav(level, ...) do { if (level <= gettracelevel()) tracenav_impl(level, __VA_ARGS__); } while (0)
#define tracegnav(level, ...) do { if (level <= gettracelevel()) tracegnav_impl(level, __VA_ARGS__); } while (0)
#define tracehnav(level, ...) do { if (level <= gettracelevel()) tracehnav_impl(level, __VA_ARGS__); } while (0)
#define tracepeph(level, ...) do { if (level <= gettracelevel()) tracepeph_impl(level, __VA_ARGS__); } while (0)
#define tracepclk(level, ...) do { if (level <= gettracelevel()) tracepclk_impl(level, __VA_ARGS__); } while (0)
#define traceb(level, ...) do { if (level <= gettracelevel()) traceb_impl(level, __VA_ARGS__); } while (0)

EXPORT void traceopen(const char *file);
EXPORT void traceclose(void);
EXPORT void tracelevel(int level);
EXPORT int gettracelevel(void);

EXPORT void trace_impl    (int level, const char *format, ...);
EXPORT void tracet_impl   (int level, const char *format, ...);
EXPORT void tracemat_impl (int level, const double *A, int n, int m, int p, int q);
EXPORT void traceobs_impl (int level, const obsd_t *obs, int n);
EXPORT void tracenav_impl (int level, const nav_t *nav);
EXPORT void tracegnav_impl(int level, const nav_t *nav);
EXPORT void tracehnav_impl(int level, const nav_t *nav);
EXPORT void tracepeph_impl(int level, const nav_t *nav);
EXPORT void tracepclk_impl(int level, const nav_t *nav);
EXPORT void traceb_impl   (int level, const uint8_t *p, int n);

#else

#define traceopen(file)       ((void)0)
#define traceclose()          ((void)0)
#define tracelevel(level)     ((void)0)
#define gettracelevel() 0

#define trace(level, ...)     ((void)0)
#define tracet(level, ...)    ((void)0)
#define tracemat(level, ...)  ((void)0)
#define traceobs(level, ...)  ((void)0)
#define tracenav(level, ...)  ((void)0)
#define tracegnav(level, ...) ((void)0)
#define tracehnav(level, ...) ((void)0)
#define tracepeph(level, ...) ((void)0)
#define tracepclk(level, ...) ((void)0)
#define traceb(level, ...)    ((void)0)    */

/* #endif  */ /* TRACE */

#ifdef __cplusplus
}
#endif
  
/* #endif  */ /* RTKLIB_TRACE_H */

