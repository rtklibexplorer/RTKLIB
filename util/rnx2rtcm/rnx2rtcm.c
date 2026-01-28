//-----------------------------------------------------------------------------
// rnx2rtcm.c : RINEX to RTCM converter
//
//          Copyright (C) 2012 by T.TAKASU, All rights reserved.
//
// Version : $Revision: 1.1 $ $Date: 2008/07/17 21:55:16 $
// History : 2012/12/12  1.0 new
//-----------------------------------------------------------------------------
#include "rtklib.h"

#define TRACEFILE "rnx2rtcm.trace"  // Debug trace file

// Print usage ---------------------------------------------------------------
static const char *help[] = {"",
                             "usage: rnx2rtcm [options] [infile ...]",
                             "",
                             "options:",
                             "  -ts  y/m/d h:m:s    start time (gpst)",
                             "  -te  y/m/d h:m:s    end time (gpst)",
                             "  -ti  tint           time interval (s)",
                             "  -sta staid          station id",
                             "  -out outfile        output RTCM file",
                             "  -typ type[,type...] RTCM message types",
                             "  -opt opt            RTCM options",
                             "  -rnxopt opt         RINEX options",
                             "  -x   level          debug trace level",
                             ""};
static void print_help(void) {
  for (int i = 0; i < sizeof(help) / sizeof(*help); i++) fprintf(stderr, "%s\n", help[i]);
  exit(0);
}
// Test RTCM nav data --------------------------------------------------------
static int is_nav(int type) {
  return type == 1019 || type == 1041 || type == 1042 || type == 1044 || type == 1045 ||
         type == 1046;
}
// Test RTCM gnav data -------------------------------------------------------
static int is_gnav(int type) { return type == 1020; }
// Test RTCM ant info --------------------------------------------------------
static int is_ant(int type) {
  return type == 1005 || type == 1006 || type == 1007 || type == 1008 || type == 1033;
}
// Code adapted from streamsvr.c write_rtcm3_msm()
static void write_rtcm3_msm(FILE *fp, rtcm_t *rtcm, int msg, int sync) {
  int sys;
  if (1071 <= msg && msg <= 1077)
    sys = SYS_GPS;
  else if (1081 <= msg && msg <= 1087)
    sys = SYS_GLO;
  else if (1091 <= msg && msg <= 1097)
    sys = SYS_GAL;
  else if (1101 <= msg && msg <= 1107)
    sys = SYS_SBS;
  else if (1111 <= msg && msg <= 1117)
    sys = SYS_QZS;
  else if (1121 <= msg && msg <= 1127)
    sys = SYS_CMP;
  else if (1131 <= msg && msg <= 1137)
    sys = SYS_IRN;
  else
    return;

  // Count the number of satellites and signals
  obsd_t *data = rtcm->obs.data;
  int nobs = rtcm->obs.n, nsat = 0, nsig = 0, mask[MAXCODE] = {0};
  for (int i = 0; i < nobs && i < MAXOBS; i++) {
    if (satsys(data[i].sat, NULL) != sys) continue;
    nsat++;
    for (int j = 0; j < NFREQ + NEXOBS; j++) {
      int code = data[i].code[j];
      if (!code || mask[code - 1]) continue;
      mask[code - 1] = 1;
      nsig++;
    }
  }
  if (nsig > 64) return;

  // Pack data into multiple messages if nsat x nsig > 64
  int ns, nmsg;
  if (nsig > 0) {
    ns = 64 / nsig;              // Max number of sats in a message
    nmsg = (nsat - 1) / ns + 1;  // Number of messages
  } else {
    ns = 0;
    nmsg = 1;
  }
  obsd_t buff[MAXOBS];
  rtcm->obs.data = buff;

  for (int i = 0, j = 0; i < nmsg; i++) {
    int n;
    for (n = 0; n < ns && j < nobs && j < MAXOBS; j++) {
      if (satsys(data[j].sat, NULL) != sys) continue;
      rtcm->obs.data[n++] = data[j];
    }
    rtcm->obs.n = n;

    if (gen_rtcm3(rtcm, msg, 0, (i < nmsg - 1) || sync)) {
      if (fwrite(rtcm->buff, rtcm->nbyte, 1, fp) < 1) break;
    }
  }
  rtcm->obs.data = data;
  rtcm->obs.n = nobs;
}
static void gen_rtcm_obs(rtcm_t *rtcm, const int *type, int n, FILE *fp) {
  int j = 0;

  for (int i = 0; i < n; i++) {
    if (is_nav(type[i]) || is_gnav(type[i]) || is_ant(type[i]) || type[i] == 1013 ||
        type[i] == 1230)
      continue;
    j = i;  // Index of last message
  }
  for (int i = 0; i < n; i++) {
    if (is_nav(type[i]) || is_gnav(type[i]) || is_ant(type[i]) || type[i] == 1013 ||
        type[i] == 1230)
      continue;

    int msg = type[i];
    int sync = i != j;
    if (msg <= 1012) {
      if (!gen_rtcm3(rtcm, msg, 0, sync)) continue;
      if (fwrite(rtcm->buff, rtcm->nbyte, 1, fp) < 1) break;
    } else {  // RTCM3 MSM
      write_rtcm3_msm(fp, rtcm, msg, sync);
    }
  }
}
// Generate RTCM nav data messages -------------------------------------------
static void gen_rtcm_nav(gtime_t time, rtcm_t *rtcm, const nav_t *nav, int index[3],
                         const int *type, int n, FILE *fp) {
  for (int i = index[0]; i < nav->n; i++) {
    if (time.time && timediff(nav->eph[i].ttr, time) > -0.1) continue;
    int sat = nav->eph[i].sat;
    int sys = satsys(sat, NULL);
    int set = (sys == SYS_GAL && (nav->eph[i].code & ((1 << 8) | (1 << 1)))) ? 1 : 0;
    rtcm->time = nav->eph[i].ttr;
    rtcm->nav.eph[sat - 1 + set * MAXSAT] = nav->eph[i];
    // Clear the other set, to not trigger an unexpected message below.
    eph_t eph0 = {0};
    rtcm->nav.eph[sat - 1 + (set == 0 ? 1 : 0) * MAXSAT] = eph0;
    rtcm->ephsat = sat;
    rtcm->ephset = set;

    for (int j = 0; j < n; j++) {
      if (!is_nav(type[j])) continue;
      if (!gen_rtcm3(rtcm, type[j], 0, 0)) continue;
      if (fwrite(rtcm->buff, rtcm->nbyte, 1, fp) < 1) break;
    }
    index[0] = i + 1;
  }
  for (int i = index[1]; i < nav->ng; i++) {
    if (time.time && timediff(nav->geph[i].tof, time) > -0.1) continue;
    int sat = nav->geph[i].sat;
    int prn;
    if (satsys(sat, &prn) != SYS_GLO) continue;
    rtcm->time = nav->geph[i].tof;
    rtcm->nav.geph[prn - 1] = nav->geph[i];
    rtcm->ephsat = sat;
    rtcm->ephset = 0;

    for (int j = 0; j < n; j++) {
      if (!is_gnav(type[j])) continue;
      if (!gen_rtcm3(rtcm, type[j], 0, 0)) continue;
      if (fwrite(rtcm->buff, rtcm->nbyte, 1, fp) < 1) break;
    }
    index[1] = i + 1;
  }
}

// Generate RTCM antenna info messages ---------------------------------------
static void gen_rtcm_ant(rtcm_t *rtcm, const int *type, int n, FILE *fp) {
  for (int i = 0; i < n; i++) {
    if (!is_ant(type[i])) continue;

    if (!gen_rtcm3(rtcm, type[i], 0, 0)) continue;
    if (fwrite(rtcm->buff, rtcm->nbyte, 1, fp) < 1) break;
  }
}
// Generate RTCM system parameters -------------------------------------------
static void gen_rtcm_sysparams(rtcm_t *rtcm, const int *type, int n, FILE *fp) {
  for (int i = 0; i < n; i++) {
    if (type[i] != 1013) continue;
    if (!gen_rtcm3(rtcm, type[i], 0, 0)) continue;
    if (fwrite(rtcm->buff, rtcm->nbyte, 1, fp) < 1) break;
  }
}
// Generate RTCM GLONASS L1 and L2 code-phase biases -------------------------
static void gen_glo_biases(rtcm_t *rtcm, const int *type, int n, FILE *fp) {
  for (int i = 0; i < n; i++) {
    if (type[i] != 1230) continue;
    if (!gen_rtcm3(rtcm, type[i], 0, 0)) continue;
    if (fwrite(rtcm->buff, rtcm->nbyte, 1, fp) < 1) break;
  }
}
// Convert to RTCM messages --------------------------------------------------
static int conv_rtcm(const int *type, const double *tint, int n, const char *opt,
                     const char *outfile, const obs_t *obs, const nav_t *nav, const sta_t *sta,
                     int staid) {
  rtcm_t rtcm = {0};

  strcpy(rtcm.opt, opt);

  eph_t eph0 = {0};
  rtcm.nav.eph = (eph_t *)malloc(sizeof(eph_t) * MAXSAT * 2);
  if (!rtcm.nav.eph) return 0;
  rtcm.nav.n = rtcm.nav.nmax = 2;
  for (int i = 0; i < MAXSAT * 2; i++) rtcm.nav.eph[i] = eph0;

  rtcm.nav.geph = NULL;
  rtcm.nav.ng = rtcm.nav.ngmax = 0;
  if (MAXPRNGLO > 0) {
    geph_t geph0 = {0};
    rtcm.nav.geph = (geph_t *)malloc(sizeof(geph_t) * MAXPRNGLO);
    if (!rtcm.nav.geph) return 0;
    rtcm.nav.ng = rtcm.nav.ngmax = 1;
    for (int i = 0; i < MAXPRNGLO; i++) rtcm.nav.geph[i] = geph0;

    // Update GLONASS freq channel number
    for (int i = 0; i < nav->ng; i++) {
      int prn;
      if (satsys(nav->geph[i].sat, &prn) != SYS_GLO) continue;
      rtcm.nav.geph[prn - 1] = nav->geph[i];
    }
    for (int i = 0; i < MAXPRNGLO; i++) {
      rtcm.nav.glo_fcn[i] = nav->glo_fcn[i];
    }
  }

  rtcm.staid = staid;
  rtcm.sta = *sta;

  rtcm.nmsg = n;
  for (int i = 0; i < n; i++) {
    rtcm.msgs[i] = type[i];
    rtcm.tint[i] = tint[i];
  }

  FILE *fp = stdout;
  if (*outfile && !(fp = fopen(outfile, "wb"))) {
    fprintf(stderr, "file open error: %s\n", outfile);
    return 0;
  }

  rtcm.time = obs->data[0].time;

  // Generate RTCM system paramaters message.
  gen_rtcm_sysparams(&rtcm, type, n, fp);
  // Generate RTCM antenna info messages
  gen_rtcm_ant(&rtcm, type, n, fp);
  // Generate RTCM GLONASS L1 and L2 code-phase biases.
  gen_glo_biases(&rtcm, type, n, fp);

  int index[3] = {0};
  int j = -1;
  for (int i = 0; i < obs->n; i = j) {
    // Extract epoch obs data
    for (j = i + 1; j < obs->n; j++) {
      if (timediff(obs->data[j].time, obs->data[i].time) > DTTOL) break;
    }
    rtcm.time = obs->data[i].time;
    rtcm.seqno++;
    rtcm.obs.data = obs->data + i;
    rtcm.obs.n = j - i;

    // Generate RTCM obs data messages
    gen_rtcm_obs(&rtcm, type, n, fp);

    // Generate RTCM nav data messages
    gen_rtcm_nav(rtcm.time, &rtcm, nav, index, type, n, fp);

    char tstr[40];
    fprintf(stderr, "%s: NOBS=%2d\r", time2str(rtcm.time, tstr, 0), rtcm.obs.n);
  }
  // Generate RTCM nav data messages
  gtime_t time0 = {0};
  gen_rtcm_nav(time0, &rtcm, nav, index, type, n, fp);

  fclose(fp);

  // Print statistics
  fprintf(stderr, "\n  MT  # OF MSGS\n");

  for (int i = 1; i < 299; i++) {
    if (!rtcm.nmsg3[i]) continue;
    fprintf(stderr, "%04d %10u\n", 1000 + i, rtcm.nmsg3[i]);
  }
  fprintf(stderr, "\n");
  free(rtcm.nav.eph);
  free(rtcm.nav.geph);
  return 1;
}
// Main ----------------------------------------------------------------------
int main(int argc, char **argv) {
  double es[6] = {0}, ee[6] = {0}, tint = 0.0;
  char *infile[16] = {0}, *outfile = "";
  int n = 0, trlevel = 0, staid = 0;
  int type[32], m = 0;
  char *rnxopt = "", *rtcmopt = "";
  double otint[32];

  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "-ts") && i + 2 < argc) {
      sscanf(argv[++i], "%lf/%lf/%lf", es, es + 1, es + 2);
      sscanf(argv[++i], "%lf:%lf:%lf", es + 3, es + 4, es + 6);
    } else if (!strcmp(argv[i], "-te") && i + 2 < argc) {
      sscanf(argv[++i], "%lf/%lf/%lf", ee, ee + 1, ee + 2);
      sscanf(argv[++i], "%lf:%lf:%lf", ee + 3, ee + 4, ee + 5);
    } else if (!strcmp(argv[i], "-ti") && i + 1 < argc)
      tint = atof(argv[++i]);
    else if (!strcmp(argv[i], "-sta") && i + 1 < argc)
      staid = atoi(argv[++i]);
    else if (!strcmp(argv[i], "-out") && i + 1 < argc)
      outfile = argv[++i];
    else if (!strcmp(argv[i], "-typ") && i + 1 < argc) {
      char buff[1024];
      strcpy(buff, argv[++i]);
      char *p;
      for (p = strtok(buff, ","); p && m < 32; p = strtok(NULL, ",")) {
        double interval = 0.0;
        int msg, r = sscanf(p, "%d(%lf)", &msg, &interval);
        if (r < 1) continue;
        if (r < 2) {
          // Default the interval.
          if (is_nav(msg) || is_gnav(msg))
            interval = 60;
          else if (is_ant(msg) || msg == 1013 || msg == 1230)
            interval = 10;
          else
            interval = tint;
        }
        type[m] = msg;
        otint[m++] = interval;
      }
    } else if (!strcmp(argv[i], "-opt") && i + 1 < argc) {
      rtcmopt = argv[++i];
    } else if (!strcmp(argv[i], "-rnxopt") && i + 1 < argc) {
      rnxopt = argv[++i];
    } else if (!strcmp(argv[i], "-x") && i + 1 < argc)
      trlevel = atoi(argv[++i]);
    else if (!strncmp(argv[i], "-", 1))
      print_help();
    else
      infile[n++] = argv[i];
  }
  if (trlevel > 0) {
    traceopen(TRACEFILE);
    tracelevel(trlevel);
  }
  gtime_t ts = {0}, te = {0};
  if (es[0] > 0.0) ts = epoch2time(es);
  if (ee[0] > 0.0) te = epoch2time(ee);

  // Read RINEX files
  obs_t obs = {0};
  nav_t nav = {0};
  sta_t sta = {{0}};
  for (int i = 0; i < n; i++) {
    readrnxt(infile[i], 0, ts, te, tint, rnxopt, &obs, &nav, &sta);
  }
  sortobs(&obs);
  uniqnav(&nav);  // To sort them for gen_rtcm_nav()

  // Convert to RTCM messages
  int ret = 0;
  if (!conv_rtcm(type, otint, m, rtcmopt, outfile, &obs, &nav, &sta, staid)) ret = -1;

  free(obs.data);
  freenav(&nav, 0xFF);

  if (trlevel > 0) {
    traceclose();
  }
  return ret;
}
