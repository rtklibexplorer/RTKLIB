/* debug trace functions -----------------------------------------------------*/
#ifdef TRACE
#include "rtklib.h"

static FILE *fp_trace = NULL;    /* file pointer of trace */
static char file_trace[1024];    /* trace file */
static int level_trace = 0;      /* level of trace */
static uint32_t tick_trace = 0;  /* tick time at traceopen (ms) */
static gtime_t time_trace = {0}; /* time at traceopen */
static rtklib_lock_t lock_trace; /* lock for trace */

static void traceswap(void)
{
    gtime_t time = utc2gpst(timeget());
    char path[1024];

    rtklib_lock(&lock_trace);

    if ((int)(time2gpst(time,       NULL) / INT_SWAP_TRAC) ==
        (int)(time2gpst(time_trace, NULL) / INT_SWAP_TRAC)) {
        rtklib_unlock(&lock_trace);
        return;
    }
    time_trace = time;

    if (!reppath(file_trace, path, time, "", "")) {
        rtklib_unlock(&lock_trace);
        return;
    }
    if (fp_trace && fp_trace != stderr) fclose(fp_trace);

    if (!(fp_trace = fopen(path, "w"))) {
        fp_trace = stderr;
    }
    rtklib_unlock(&lock_trace);
}
extern void traceopen(const char *file)
{
    gtime_t time = utc2gpst(timeget());
    char path[1024];

    reppath(file, path, time, "", "");
    if (!*path || !(fp_trace = fopen(path, "w"))) fp_trace = stderr;
    strcpy(file_trace, file);
    tick_trace = tickget();
    time_trace = time;
    rtklib_initlock(&lock_trace);
}
extern void traceclose(void)
{
    if (fp_trace && fp_trace != stderr) fclose(fp_trace);
    fp_trace = NULL;
    file_trace[0] = '\0';
}
extern void tracelevel(int level) { level_trace = level; }
extern int gettracelevel(void) { return level_trace; }
extern void trace_impl(int level, const char *format, ...)
{
    va_list ap;

    /* print error message to stderr */
    if (level <= 1) {
        va_start(ap, format);
        vfprintf(stderr, format, ap);
        va_end(ap);
    }
    if (!fp_trace || level > level_trace) return;
    traceswap();
    fprintf(fp_trace, "%d ", level);
    va_start(ap, format);
    vfprintf(fp_trace, format, ap);
    va_end(ap);
    fflush(fp_trace);
}
extern void tracet_impl(int level, const char *format, ...)
{
    va_list ap;

    if (!fp_trace || level > level_trace) return;
    traceswap();
    fprintf(fp_trace, "%d %9.3f: ", level, (tickget() - tick_trace) / 1000.0);
    va_start(ap, format);
    vfprintf(fp_trace, format, ap);
    va_end(ap);
    fflush(fp_trace);
}
extern void tracemat_impl(int level, const double *A, int n, int m, int p,
                          int q)
{
    if (!fp_trace || level > level_trace) return;
    matfprint(A, n, m, p, q, fp_trace);
    fflush(fp_trace);
}
extern void traceobs_impl(int level, const obsd_t *obs, int n)
{
    char str[40], id[8];
    int i;

    if (!fp_trace || level > level_trace) return;
    for (i = 0; i < n; i++) {
        time2str(obs[i].time, str, 3);
        satno2id(obs[i].sat, id);
        fprintf(fp_trace,
                " (%2d) %s %-3s rcv%d %13.3f %13.3f %13.3f %13.3f %d %d %d %d "
                "%3.4f %3.3f %3.1f %3.1f\n",
                i + 1, str, id, obs[i].rcv, obs[i].L[0], obs[i].L[1],
                obs[i].P[0], obs[i].P[1], obs[i].LLI[0], obs[i].LLI[1],
                obs[i].code[0], obs[i].code[1], obs[i].Lstd[0], obs[i].Pstd[0],
                obs[i].SNR[0], obs[i].SNR[1]);
    }
    fflush(fp_trace);
}
extern void tracenav_impl(int level, const nav_t *nav)
{
    char s1[40], s2[40], id[8];
    int i;

    if (!fp_trace || level > level_trace) return;
    for (i = 0; i < nav->n; i++) {
        time2str(nav->eph[i].toe, s1, 0);
        time2str(nav->eph[i].ttr, s2, 0);
        satno2id(nav->eph[i].sat, id);
        fprintf(fp_trace, "(%3d) %-3s : %s %s %3d %3d %02x\n", i + 1, id, s1,
                s2, nav->eph[i].iode, nav->eph[i].iodc, nav->eph[i].svh);
    }
    fprintf(fp_trace, "(ion) %9.4e %9.4e %9.4e %9.4e\n", nav->ion_gps[0],
            nav->ion_gps[1], nav->ion_gps[2], nav->ion_gps[3]);
    fprintf(fp_trace, "(ion) %9.4e %9.4e %9.4e %9.4e\n", nav->ion_gps[4],
            nav->ion_gps[5], nav->ion_gps[6], nav->ion_gps[7]);
    fprintf(fp_trace, "(ion) %9.4e %9.4e %9.4e %9.4e\n", nav->ion_gal[0],
            nav->ion_gal[1], nav->ion_gal[2], nav->ion_gal[3]);
}
extern void tracegnav_impl(int level, const nav_t *nav)
{
    char s1[40], s2[40], id[8];
    int i;

    if (!fp_trace || level > level_trace) return;
    for (i = 0; i < nav->ng; i++) {
        time2str(nav->geph[i].toe, s1, 0);
        time2str(nav->geph[i].tof, s2, 0);
        satno2id(nav->geph[i].sat, id);
        fprintf(fp_trace, "(%3d) %-3s : %s %s %2d %2d %8.3f\n", i + 1, id, s1,
                s2, nav->geph[i].frq, nav->geph[i].svh,
                nav->geph[i].taun * 1E6);
    }
}
extern void tracehnav_impl(int level, const nav_t *nav)
{
    char s1[40], s2[40], id[8];
    int i;

    if (!fp_trace || level > level_trace) return;
    for (i = 0; i < nav->ns; i++) {
        time2str(nav->seph[i].t0, s1, 0);
        time2str(nav->seph[i].tof, s2, 0);
        satno2id(nav->seph[i].sat, id);
        fprintf(fp_trace, "(%3d) %-3s : %s %s %2d %2d\n", i + 1, id, s1, s2,
                nav->seph[i].svh, nav->seph[i].sva);
    }
}
extern void tracepeph_impl(int level, const nav_t *nav)
{
    char s[40], id[8];
    int i, j;

    if (!fp_trace || level > level_trace) return;

    for (i = 0; i < nav->ne; i++) {
        time2str(nav->peph[i].time, s, 0);
        for (j = 0; j < MAXSAT; j++) {
            satno2id(j + 1, id);
            fprintf(fp_trace,
                    "%-3s %d %-3s %13.3f %13.3f %13.3f %13.3f %6.3f %6.3f "
                    "%6.3f %6.3f\n",
                    s, nav->peph[i].index, id, nav->peph[i].pos[j][0],
                    nav->peph[i].pos[j][1], nav->peph[i].pos[j][2],
                    nav->peph[i].pos[j][3] * 1E9, nav->peph[i].std[j][0],
                    nav->peph[i].std[j][1], nav->peph[i].std[j][2],
                    nav->peph[i].std[j][3] * 1E9);
        }
    }
}
extern void tracepclk_impl(int level, const nav_t *nav)
{
    char s[40], id[8];
    int i, j;

    if (!fp_trace || level > level_trace) return;

    for (i = 0; i < nav->nc; i++) {
        time2str(nav->pclk[i].time, s, 0);
        for (j = 0; j < MAXSAT; j++) {
            satno2id(j + 1, id);
            fprintf(fp_trace, "%-3s %d %-3s %13.3f %6.3f\n", s,
                    nav->pclk[i].index, id, nav->pclk[i].clk[j][0] * 1E9,
                    nav->pclk[i].std[j][0] * 1E9);
        }
    }
}
extern void traceb_impl(int level, const uint8_t *p, int n)
{
    int i;
    if (!fp_trace || level > level_trace) return;
    for (i = 0; i < n; i++)
        fprintf(fp_trace, "%02X%s", *p++, i % 8 == 7 ? " " : "");
    fprintf(fp_trace, "\n");
}

#endif /* TRACE */
