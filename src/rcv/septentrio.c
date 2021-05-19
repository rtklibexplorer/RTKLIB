/*------------------------------------------------------------------------------
 * septentrio.c : Septentrio Binary Format (SBF) decoder
 *
 *          Copyright (C) 2020 by Tomoji TAKASU
 *
 * reference :
 *     [1] Septentrio, mosaic-X5 reference guide applicable to version 4.8.0 of
 *         the firmware, June 4, 2020
 *
 * version : $Revision:$
 *
 * history : 2013/07/17  1.0  begin writing
 *           2013/10/24  1.1  GPS L1 working
 *           2013/11/02  1.2  modified by TTAKASU
 *           2015/01/26  1.3  fix some problems by Jens Reimann
 *           2016/02/04  1.4  by Jens Reimann
 *                           - added more sanity checks
 *                           - added galileon raw decoding
 *                           - added usage of decoded SBAS messages for testing
 *                           - add QZSS and Compass/Beidou navigation messages
 *                           - fixed code and Doppler for 2nd and following
 *frequency
 *                           - fixed bug in glonass ephemeris
 *                           - fixed decoding of galileo ephemeris
 *                           - fixed lost lock indicator
 *                           - fixed sbas decoding
 *                           - cleanups
 *           2016/03/03  1.5 - fixed TOW in SBAS messages
 *           2016/03/12  1.6 - respect code priorities
 *                           - fixed bug in carrier phase calculation of type2
 *data
 *                           - unify frequency determination
 *                           - improve lock handling
 *                           - various bug fixes
 *           2016/05/25  1.7  rtk_crc24q() -> crc24q() by T.T
 *           2016/07/29  1.8  crc24q() -> rtk_crc24q() by T.T
 *           2017/04/11  1.9  (char *) -> (signed char *) by T.T
 *           2017/09/01  1.10 suppress warnings
 *
 *           2020/11/30  1.11 rewritten from scratch to support mosaic-X5 [1]
 *           2021/05/19  1.12 Bring across all the removed code in 1.11
 *-----------------------------------------------------------------------------*/

#include "rtklib.h"

#define SBF_SYNC1 0x24 /* SBF block header 1 */
#define SBF_SYNC2 0x40 /* SBF block header 2 */
#define SBF_MAXSIG 36  /* SBF max signal number */

#define SBF_MEASEPOCH 4027  /* SBF GNSS measurements */
#define SBF_MEASEXTRA 4000  /* SBF GNSS measurements extra info */
#define SBF_GPSRAWCA 4017   /* SBF GPS C/A subframe */
#define SBF_GLORAWCA 4026   /* SBF GLONASS L1CA or L2CA navigation string */
#define SBF_GALRAWFNAV 4022 /* SBF Galileo F/NAV navigation page */
#define SBF_GALRAWINAV 4023 /* SBF Galileo I/NAV navigation page */
#define SBF_GEORAWL1 4020   /* SBF SBAS L1 navigation frame */
#define SBF_BDSRAW 4047     /* SBF BDS navigation page */
#define SBF_QZSRAWL1CA 4066 /* SBF QZSS C/A subframe */
#define SBF_NAVICRAW 4093   /* SBF NavIC/IRNSS subframe */
#define SBF_PVTGEOD                                                            \
  4007 /* SBF message id: Rx Position Velocity and Time data in Geodetic       \
          coordinates */
#define SBF_DOP 4001 /* SBF message id: Dilution of Precision data */
#define SBF_MEASEPOCH_END                                                      \
  5922 /* SBF message id: end of SBF range measurememts */

#define SBF_ENDOFPVT 5921 /* SBF message id: End of any PVT block */
#define SBF_CHNSTATUS                                                          \
  4013 /* SBF message id: Status of the receiver channels                      \
        */
#define SBF_BASEVECGEOD                                                        \
  4028 /* SBF message id: Base station vector                                  \
        */
#define SBF_ATTEULER                                                           \
  5938 /* SBF message id: Euler Angles of attitude                             \
        */
#define SBF_POSCOVGEO                                                          \
  5906 /* SBF message id: Position covariance matrix                           \
        */

#define SBF_GEONAV 5896          /* SBF message id:  SBAS navigation message */
#define SBF_GEOALM 5897          /* SBF message id:  SBAS satellite almanac */
#define SBF_GEOSERVICELEVEL 5917 /* SBF message id:  SBAS Service Message */
#define SBF_GEONETWORKTIME                                                     \
  5918 /* SBF message id:  SBAS Network Time/UTC offset parameters */
#define SBF_GEOMT00                                                            \
  5925 /* SBF message id:  SBAS: Don't use for safety application */
#define SBF_GEOPRNMASK 5926   /* SBF message id:  PRN Mask assignments */
#define SBF_GEOFASTCORR 5927  /* SBF message id:  Fast Corrections */
#define SBF_GEOINTEGRITY 5928 /* SBF message id:  Integrity information */
#define SBF_GEOFASTCORRDEGR                                                    \
  5929 /* SBF message id:  fast correction degradation factor */
#define SBF_GEODEGRFACTORS 5930 /* SBF message id:  Degration factors */
#define SBF_GEOIGPMASK 5931 /* SBF message id:  Ionospheric grid point mask */
#define SBF_GEOLONGTERMCOR                                                     \
  5932 /* SBF message id:  Long term satellite error corrections */
#define SBF_GEOIONODELAY                                                       \
  5933 /* SBF message id:  Inospheric delay correction                         \
        */
#define SBF_GEOCLOCKEPHCOVMATRIX                                               \
  5934 /* SBF message id:  Clock-Ephemeris Covariance Matrix l*/

#define SBF_QUALITY_IND 4082 /* SBF message id:  Quality Indicators*/
#define SBF_PVTSUPPORT 4076  /* SBF message id: PVT support params*/
#define SBF_GPSRAWL2C                                                          \
  4018 /* SBF message id: GPS raw navigation page or frame */

#define SBF_GPSNAV 5891 /* SBF message id: GPS navigation data */
#define SBF_GPSALM 5892 /* SBF message id: GPS almanac */
#define SBF_GPSION                                                             \
  5893 /* SBF message id: GPS ionosphere data, Klobuchar coefficients */
#define SBF_GPSUTC 5894 /* SBF message id: GPS UTC data */

#define SBF_GLONAV 4004  /* SBF message id: GLONASS navigation data */
#define SBF_GLOALM 4005  /* SBF message id: GLONASS almanac */
#define SBF_GLOTIME 4036 /* SBF message id: GLONASS time data */

#define SBF_GALNAV 4002 /* SBF message id: Galileo navigation data */
#define SBF_GALALM 4003 /* SBF message id: Galileo almanac */
#define SBF_GALION                                                             \
  4030 /* SBF message id: Galileo ionosphere data, Klobuchar coefficients */
#define SBF_GALUTC 4031 /* SBF message id: Galileo UTC data */

#define SBF_CMPNAV 4081  /* SBF message id: Compass navigation data */
#define SBF_QZSSNAV 4095 /* SBF message id: QZSS navigation data */

#define SBF_GALGSTGPS 4032 /* SBF message id: Galileo GPS time offset */
#define SBF_GPSRAWL5                                                           \
  4019 /* SBF message id: GPS raw navigation page or frame                     \
        */

/* get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((uint8_t *)(p)))
#define I1(p) (*((int8_t *)(p)))
static uint16_t U2(uint8_t *p) {
  uint16_t a;
  memcpy(&a, p, 2);
  return a;
}
static uint32_t U4(uint8_t *p) {
  uint32_t a;
  memcpy(&a, p, 4);
  return a;
}
static int32_t I4(uint8_t *p) {
  int32_t a;
  memcpy(&a, p, 4);
  return a;
}

static float R4(uint8_t *p) {
  float value;
  uint8_t *q = (uint8_t *)&value + 3;
  int i;
  for (i = 0; i < 4; i++)
    *q-- = *p++;
  return value;
}
static double R8(uint8_t *p) {
  double value;
  uint8_t *q = (uint8_t *)&value + 7;
  int i;
  for (i = 0; i < 8; i++)
    *q-- = *p++;
  return value;
}
/* svid to satellite number ([1] 4.1.9) --------------------------------------*/
static int svid2sat(int svid) {
  if (svid <= 37)
    return satno(SYS_GPS, svid);
  if (svid <= 61)
    return satno(SYS_GLO, svid - 37);
  if (svid <= 62)
    return 0; /* glonass unknown slot */
  if (svid <= 68)
    return satno(SYS_GLO, svid - 38);
  if (svid <= 70)
    return 0;
  if (svid <= 106)
    return satno(SYS_GAL, svid - 70);
  if (svid <= 119)
    return 0;
  if (svid <= 140)
    return satno(SYS_SBS, svid);
  if (svid <= 180)
    return satno(SYS_CMP, svid - 140);
  if (svid <= 187)
    return satno(SYS_QZS, svid - 180 + 192);
  if (svid <= 190)
    return 0;
  if (svid <= 197)
    return satno(SYS_IRN, svid - 190);
  if (svid <= 215)
    return satno(SYS_SBS, svid - 57);
  if (svid <= 222)
    return satno(SYS_IRN, svid - 208);
  if (svid <= 245)
    return satno(SYS_CMP, svid - 182);
  return 0; /* error */
}
/* signal number table ([1] 4.1.10) ------------------------------------------*/
static uint8_t sig_tbl[SBF_MAXSIG + 1][2] = {
    /* system, obs-code */
    {SYS_GPS, CODE_L1C}, /*  0: GPS L1C/A */
    {SYS_GPS, CODE_L1W}, /*  1: GPS L1P */
    {SYS_GPS, CODE_L2W}, /*  2: GPS L2P */
    {SYS_GPS, CODE_L2L}, /*  3: GPS L2C */
    {SYS_GPS, CODE_L5Q}, /*  4: GPS L5 */
    {SYS_GPS, CODE_L1L}, /*  5: GPS L1C */
    {SYS_QZS, CODE_L1C}, /*  6: QZS L1C/A */
    {SYS_QZS, CODE_L2L}, /*  7: QZS L2C */
    {SYS_GLO, CODE_L1C}, /*  8: GLO L1C/A */
    {SYS_GLO, CODE_L1P}, /*  9: GLO L1P */
    {SYS_GLO, CODE_L2P}, /* 10: GLO L2P */
    {SYS_GLO, CODE_L2C}, /* 11: GLO L2C/A */
    {SYS_GLO, CODE_L3Q}, /* 12: GLO L3 */
    {SYS_CMP, CODE_L1P}, /* 13: BDS B1C */
    {SYS_CMP, CODE_L5P}, /* 14: BDS B2a */
    {SYS_IRN, CODE_L5A}, /* 15: IRN L5 */
    {0, 0},              /* 16: reserved */
    {SYS_GAL, CODE_L1C}, /* 17: GAL E1(L1BC) */
    {0, 0},              /* 18: reserved */
    {SYS_GAL, CODE_L6C}, /* 19: GAL E6(E6BC) */
    {SYS_GAL, CODE_L5Q}, /* 20: GAL E5a */
    {SYS_GAL, CODE_L7Q}, /* 21: GAL E5b */
    {SYS_GAL, CODE_L8Q}, /* 22: GAL E5 AltBoc */
    {0, 0},              /* 23: LBand */
    {SYS_SBS, CODE_L1C}, /* 24: SBS L1C/A */
    {SYS_SBS, CODE_L5I}, /* 25: SBS L5 */
    {SYS_QZS, CODE_L5Q}, /* 26: QZS L5 */
    {SYS_QZS, CODE_L6L}, /* 27: QZS L6 */
    {SYS_CMP, CODE_L2I}, /* 28: BDS B1I */
    {SYS_CMP, CODE_L7I}, /* 29: BDS B2I */
    {SYS_CMP, CODE_L6I}, /* 30: BDS B3I */
    {0, 0},              /* 31: reserved */
    {SYS_QZS, CODE_L1L}, /* 32: QZS L1C */
    {SYS_QZS, CODE_L1Z}, /* 33: QZS L1S */
    {SYS_CMP, CODE_L7D}, /* 34: BDS B2b */
    {0, 0},              /* 35: reserved */
    {SYS_IRN, CODE_L9A}  /* 36: IRN S */
};
/* signal number to freq-index and code --------------------------------------*/
static int sig2idx(int sat, int sig, const char *opt, uint8_t *code) {
  int idx, sys = satsys(sat, NULL), nex = NEXOBS;

  if (sig < 0 || sig > SBF_MAXSIG || sig_tbl[sig][0] != sys)
    return -1;
  *code = sig_tbl[sig][1];
  idx = code2idx(sys, *code);

  /* resolve code priority in a freq-index */
  if (sys == SYS_GPS) {
    if (strstr(opt, "-GL1W") && idx == 0)
      return (*code == CODE_L1W) ? 0 : -1;
    if (strstr(opt, "-GL1L") && idx == 0)
      return (*code == CODE_L1L) ? 0 : -1;
    if (strstr(opt, "-GL2L") && idx == 1)
      return (*code == CODE_L2L) ? 1 : -1;
    if (*code == CODE_L1W)
      return (nex < 1) ? -1 : NFREQ;
    if (*code == CODE_L2L)
      return (nex < 2) ? -1 : NFREQ + 1;
    if (*code == CODE_L1L)
      return (nex < 3) ? -1 : NFREQ + 2;
  } else if (sys == SYS_GLO) {
    if (strstr(opt, "-RL1P") && idx == 0)
      return (*code == CODE_L1P) ? 0 : -1;
    if (strstr(opt, "-RL2C") && idx == 1)
      return (*code == CODE_L2C) ? 1 : -1;
    if (*code == CODE_L1P)
      return (nex < 1) ? -1 : NFREQ;
    if (*code == CODE_L2C)
      return (nex < 2) ? -1 : NFREQ + 1;
  } else if (sys == SYS_QZS) {
    if (strstr(opt, "-JL1L") && idx == 0)
      return (*code == CODE_L1L) ? 0 : -1;
    if (strstr(opt, "-JL1Z") && idx == 0)
      return (*code == CODE_L1Z) ? 0 : -1;
    if (*code == CODE_L1L)
      return (nex < 1) ? -1 : NFREQ;
    if (*code == CODE_L1Z)
      return (nex < 2) ? -1 : NFREQ + 1;
  } else if (sys == SYS_CMP) {
    if (strstr(opt, "-CL1P") && idx == 0)
      return (*code == CODE_L1P) ? 0 : -1;
    if (*code == CODE_L1P)
      return (nex < 1) ? -1 : NFREQ;
  }
  return (idx < NFREQ) ? idx : -1;
}
/* initialize obs data fields ------------------------------------------------*/
static void init_obsd(gtime_t time, int sat, obsd_t *data) {
  int i;

  data->time = time;
  data->sat = (uint8_t)sat;

  for (i = 0; i < NFREQ + NEXOBS; i++) {
    data->L[i] = data->P[i] = 0.0;
    data->D[i] = 0.0f;
    data->SNR[i] = (uint16_t)0;
    data->LLI[i] = (uint8_t)0;
    data->code[i] = CODE_NONE;
  }
}
/* decode SBF GNSS measurements ----------------------------------------------*/
static int decode_measepoch(raw_t *raw) {
  uint8_t *p = raw->buff + 14, code;
  double P1, P2, L1, L2, D1, D2, S1, S2, freq1, freq2;
  int i, j, idx, n, n1, n2, len1, len2, sig, ant, svid, info, sat, sys, lock,
      fcn, LLI;
  int ant_sel = 0; /* antenna selection (0:main) */

  if (strstr(raw->opt, "-AUX1"))
    ant_sel = 1;
  else if (strstr(raw->opt, "-AUX2"))
    ant_sel = 2;

  if (raw->len < 20) {
    trace(2, "sbf measepoch length error: len=%d\n", raw->len);
    return -1;
  }
  n1 = U1(p);
  len1 = U1(p + 1);
  len2 = U1(p + 2);

  if (U1(p + 3) & 0x80) {
    trace(2, "sbf measepoch scrambled\n");
    return -1;
  }
  if (raw->outtype) {
    sprintf(raw->msgtype + strlen(raw->msgtype), " nsat=%d", n1);
  }
  for (i = n = 0, p += 6;
       i < n1 && n < MAXOBS && p + 20 <= raw->buff + raw->len; i++) {
    svid = U1(p + 2);
    ant = U1(p + 1) >> 5;
    sig = U1(p + 1) & 0x1f;
    info = U1(p + 18);
    n2 = U1(p + 19);
    fcn = 0;
    if (sig == 31)
      sig += (info >> 3) * 32;
    else if (sig >= 8 && sig <= 11)
      fcn = (info >> 3) - 8;

    if (ant != ant_sel) {
      trace(3, "sbf measepoch ant error: svid=%d ant=%d\n", svid, ant);
      p += len1 + len2 * n2;
      continue;
    }
    if (!(sat = svid2sat(svid))) {
      trace(4, "sbf measepoch svid error: svid=%d\n", svid);
      p += len1 + len2 * n2;
      continue;
    }
    if ((idx = sig2idx(sat, sig, raw->opt, &code)) < 0) {
      trace(2, "sbf measepoch sig error: sat=%d sig=%d\n", sat, sig);
      p += len1 + len2 * n2;
      continue;
    }
    init_obsd(raw->time, sat, raw->obs.data + n);
    P1 = D1 = 0.0;
    sys = satsys(sat, NULL);
    freq1 = code2freq(sys, code, fcn);

    if ((U1(p + 3) & 0x1f) != 0 || U4(p + 4) != 0) {
      P1 = (U1(p + 3) & 0x0f) * 4294967.296 + U4(p + 4) * 0.001;
      raw->obs.data[n].P[idx] = P1;
    }
    if (I4(p + 8) != -2147483648) {
      D1 = I4(p + 8) * 0.0001;
      raw->obs.data[n].D[idx] = (float)D1;
    }
    lock = U2(p + 16);
    if (P1 != 0.0 && freq1 > 0.0 && lock != 65535 &&
        (I1(p + 14) != -128 || U2(p + 12) != 0)) {
      L1 = I1(p + 14) * 65.536 + U2(p + 12) * 0.001;
      raw->obs.data[n].L[idx] = P1 * freq1 / CLIGHT + L1;
      LLI = (lock < raw->lockt[sat - 1][idx] ? 1 : 0) +
            ((info & (1 << 2)) ? 2 : 0);
      raw->obs.data[n].LLI[idx] = (uint8_t)LLI;
      raw->lockt[sat - 1][idx] = lock;
    }
    if (U1(p + 15) != 255) {
      S1 = U1(p + 15) * 0.25 + ((sig == 1 || sig == 2) ? 0.0 : 10.0);
      raw->obs.data[n].SNR[idx] = (uint16_t)(S1 / SNR_UNIT + 0.5);
    }
    raw->obs.data[n].code[idx] = code;

    for (j = 0, p += len1; j < n2 && p + 12 <= raw->buff + raw->len;
         j++, p += len2) {
      sig = U1(p) & 0x1f;
      ant = U1(p) >> 5;
      info = U1(p + 5);
      if (sig == 31)
        sig += (info >> 3) * 32;

      if (ant != ant_sel) {
        trace(3, "sbf measepoch ant error: sat=%d ant=%d\n", sat, ant);
        continue;
      }
      if ((idx = sig2idx(sat, sig, raw->opt, &code)) < 0) {
        trace(3, "sbf measepoch sig error: sat=%d sig=%d\n", sat, sig);
        continue;
      }
      P2 = 0.0;
      freq2 = code2freq(sys, code, fcn);

      if (P1 != 0.0 && (getbits(p + 3, 5, 3) != -4 || U2(p + 6) != 0)) {
        P2 = P1 + getbits(p + 3, 5, 3) * 65.536 + U2(p + 6) * 0.001;
        raw->obs.data[n].P[idx] = P2;
      }
      if (P2 != 0.0 && freq2 > 0.0 && (I1(p + 4) != -128 || U2(p + 8) != 0)) {
        L2 = I1(p + 4) * 65.536 + U2(p + 8) * 0.001;
        raw->obs.data[n].L[idx] = P2 * freq2 / CLIGHT + L2;
      }
      if (D1 != 0.0 && freq1 > 0.0 && freq2 > 0.0 &&
          (getbits(p + 3, 0, 5) != -16 || U2(p + 10) != 0)) {
        D2 = getbits(p + 3, 0, 5) * 6.5536 + U2(p + 10) * 0.0001;
        raw->obs.data[n].D[idx] = (float)(D1 * freq2 / freq1) + D2;
      }
      lock = U1(p + 1);
      if (lock != 255) {
        LLI = (lock < raw->lockt[sat - 1][idx] ? 1 : 0) +
              ((info & (1 << 2)) ? 2 : 0);
        raw->obs.data[n].LLI[idx] = (uint8_t)LLI;
        raw->lockt[sat - 1][idx] = lock;
      }
      if (U1(p + 2) != 255) {
        S2 = U1(p + 2) * 0.25 + ((sig == 1 || sig == 2) ? 0.0 : 10.0);
        raw->obs.data[n].SNR[idx] = (uint16_t)(S2 / SNR_UNIT + 0.5);
      }
      raw->obs.data[n].code[idx] = code;
    }
    n++;
  }
  raw->obs.n = n;
  return 1;
}
/* decode SBF GNSS measurements extra info -----------------------------------*/
static int decode_measextra(raw_t *raw) {
  /* not yet supported */
  return 0;
}
/* decode ephemeris ----------------------------------------------------------*/
static int decode_eph(raw_t *raw, int sat) {
  eph_t eph = {0};

  if (!decode_frame(raw->subfrm[sat - 1], &eph, NULL, NULL, NULL))
    return 0;

  if (!strstr(raw->opt, "-EPHALL")) {
    if (eph.iode == raw->nav.eph[sat - 1].iode &&
        eph.iodc == raw->nav.eph[sat - 1].iodc &&
        timediff(eph.toe, raw->nav.eph[sat - 1].toe) == 0.0 &&
        timediff(eph.toc, raw->nav.eph[sat - 1].toc) == 0.0)
      return 0;
  }
  eph.sat = sat;
  raw->nav.eph[sat - 1] = eph;
  raw->ephsat = sat;
  raw->ephset = 0;
  return 2;
}
/* UTC 8-bit week -> full week -----------------------------------------------*/
static void adj_utcweek(gtime_t time, double *utc) {
  int week;

  time2gpst(time, &week);
  utc[3] += week / 256 * 256;
  if (utc[3] < week - 127)
    utc[3] += 256.0;
  else if (utc[3] > week + 127)
    utc[3] -= 256.0;
  utc[5] += utc[3] / 256 * 256;
  if (utc[5] < utc[3] - 127)
    utc[5] += 256.0;
  else if (utc[5] > utc[3] + 127)
    utc[5] -= 256.0;
}
/* decode ION/UTC parameters -------------------------------------------------*/
static int decode_ionutc(raw_t *raw, int sat) {
  double ion[8], utc[8];
  int sys = satsys(sat, NULL);

  if (!decode_frame(raw->subfrm[sat - 1], NULL, NULL, ion, utc))
    return 0;

  adj_utcweek(raw->time, utc);
  if (sys == SYS_QZS) {
    matcpy(raw->nav.ion_qzs, ion, 8, 1);
    matcpy(raw->nav.utc_qzs, utc, 8, 1);
  } else {
    matcpy(raw->nav.ion_gps, ion, 8, 1);
    matcpy(raw->nav.utc_gps, utc, 8, 1);
  }
  return 1;
}
/* decode SBF raw C/A subframe -----------------------------------------------*/
static int decode_rawca(raw_t *raw, int sys) {
  uint8_t *p = raw->buff + 14, buff[30];
  int i, svid, sat, prn, id, ret;

  if (raw->len < 60) {
    trace(2, "sbf rawca length error: sys=%d len=%d\n", sys, raw->len);
    return -1;
  }
  svid = U1(p);
  if (!(sat = svid2sat(svid)) || satsys(sat, &prn) != sys) {
    trace(2, "sbf rawca svid error: sys=%d svid=%d\n", sys, svid);
    return -1;
  }
  if (!U1(p + 1)) {
    trace(3, "sbf rawca parity/crc error: sys=%d prn=%d\n", sys, prn);
    return 0;
  }
  if (raw->outtype) {
    sprintf(raw->msgtype + strlen(raw->msgtype), " prn=%d", prn);
  }
  for (i = 0, p += 6; i < 10; i++, p += 4) { /* 24 x 10 bits w/o parity */
    setbitu(buff, 24 * i, 24, U4(p) >> 6);
  }
  id = getbitu(buff, 43, 3);

  if (id < 1 || id > 5) {
    trace(2, "sbf rawca subframe id error: sys=%d prn=%d id=%d\n", sys, prn,
          id);
    return -1;
  }
  memcpy(raw->subfrm[sat - 1] + (id - 1) * 30, buff, 30);

  if (id == 3) {
    return decode_eph(raw, sat);
  }
  if (id == 4 || id == 5) {
    ret = decode_ionutc(raw, sat);
    memset(raw->subfrm[sat - 1] + id * 30, 0, 30);
    return ret;
  }
  return 0;
}
/* decode SBF GPS C/A subframe -----------------------------------------------*/
static int decode_gpsrawca(raw_t *raw) { return decode_rawca(raw, SYS_GPS); }
/* decode SBF GLONASS L1CA or L2CA navigation string -------------------------*/
static int decode_glorawca(raw_t *raw) {
  geph_t geph = {0};
  gtime_t *time;
  double utc[8] = {0};
  uint8_t *p = raw->buff + 14, buff[12];
  int i, svid, sat, prn, m;

  if (raw->len < 32) {
    trace(2, "sbf glorawca length error: len=%d\n", raw->len);
    return -1;
  }
  svid = U1(p);
  if (!(sat = svid2sat(svid)) || satsys(sat, &prn) != SYS_GLO) {
    trace(3, "sbf glorawca svid error: svid=%d\n", svid);
    return (svid == 62) ? 0 : -1; /* svid=62: slot unknown */
  }
  /*if (!U1(p + 1)) {
    trace(3, "sbf glorawca parity/crc error: prn=%d\n", prn);
    return 0;
  }*/
  if (raw->outtype) {
    sprintf(raw->msgtype + strlen(raw->msgtype), " prn=%d", prn);
  }
  for (i = 0; i < 3; i++) {
    setbitu(buff, 32 * i, 32, U4(p + 6 + 4 * i)); /* 85 bits */
  }
  m = getbitu(buff, 1, 4);
  if (m < 1 || m > 15) {
    trace(2, "sbf glorawca string number error: prn=%d m=%d\n", prn, m);
    return -1;
  }
  time = (gtime_t *)(raw->subfrm[sat - 1] + 150);
  if (fabs(timediff(raw->time, *time)) > 30.0) {
    memset(raw->subfrm[sat - 1], 0, 40);
    memcpy(time, &raw->time, sizeof(gtime_t));
  }
  memcpy(raw->subfrm[sat - 1] + (m - 1) * 10, buff, 10);
  if (m != 4)
    return 0;

  geph.tof = raw->time;
  if (!decode_glostr(raw->subfrm[sat - 1], &geph, utc))
    return 0;

  matcpy(raw->nav.utc_glo, utc, 8, 1);

  if (geph.sat != sat) {
    trace(2, "sbf glorawca satellite error: sat=%d %d\n", sat, geph.sat);
    return -1;
  }
  geph.frq = (int)U1(p + 4) - 8;

  if (!strstr(raw->opt, "-EPHALL")) {
    if (geph.iode == raw->nav.geph[prn - 1].iode &&
        timediff(geph.toe, raw->nav.geph[prn - 1].toe) == 0.0)
      return 0;
  }
  raw->nav.geph[prn - 1] = geph;
  raw->ephsat = sat;
  raw->ephset = 0;
  return 2;
}
/* decode SBF Galileo F/NAV navigation page ----------------------------------*/
static int decode_galrawfnav(raw_t *raw) {
  eph_t eph = {0};
  double ion[4] = {0}, utc[8] = {0};
  uint8_t *p = raw->buff + 14, buff[32];
  int i, svid, src, sat, prn, type;

  if (strstr(raw->opt, "-GALINAV"))
    return 0;

  if (raw->len < 52) {
    trace(2, "sbf galrawfnav length error: len=%d\n", raw->len);
    return -1;
  }
  svid = U1(p);
  src = U1(p + 3) & 0x1f;

  if (!(sat = svid2sat(svid)) || satsys(sat, &prn) != SYS_GAL) {
    trace(2, "sbf galrawfnav svid error: svid=%d src=%d\n", svid, src);
    return -1;
  }
  if (!U1(p + 1)) {
    trace(3, "sbf galrawfnav parity/crc error: prn=%d src=%d\n", prn, src);
    return 0;
  }
  if (raw->outtype) {
    sprintf(raw->msgtype + strlen(raw->msgtype), " prn=%d src=%d", prn, src);
  }
  if (src != 20 && src != 22) { /* E5a or E5 AltBOC */
    trace(2, "sbf galrawfnav source error: prn=%d src=%d\n", prn, src);
    return -1;
  }
  for (i = 0; i < 8; i++) {
    setbitu(buff, 32 * i, 32, U4(p + 6 + 4 * i)); /* 244 bits page */
  }
  type = getbitu(buff, 0, 6); /* page type */

  if (type == 63)
    return 0; /* dummy page */
  if (type < 1 || type > 6) {
    trace(2, "sbf galrawfnav page type error: prn=%d type=%d\n", prn, type);
    return -1;
  }
  /* save 244 bits page (31 bytes * 6 page) */
  memcpy(raw->subfrm[sat - 1] + 128 + (type - 1) * 31, buff, 31);

  if (type != 4)
    return 0;
  if (!decode_gal_fnav(raw->subfrm[sat - 1] + 128, &eph, ion, utc))
    return 0;

  if (eph.sat != sat) {
    trace(2, "sbf galrawfnav satellite error: sat=%d %d\n", sat, eph.sat);
    return -1;
  }
  eph.code |= (1 << 1); /* data source: E5a */

  adj_utcweek(raw->time, utc);
  matcpy(raw->nav.ion_gal, ion, 4, 1);
  matcpy(raw->nav.utc_gal, utc, 8, 1);

  if (!strstr(raw->opt, "-EPHALL")) {
    if (eph.iode == raw->nav.eph[sat - 1 + MAXSAT].iode &&
        timediff(eph.toe, raw->nav.eph[sat - 1 + MAXSAT].toe) == 0.0 &&
        timediff(eph.toc, raw->nav.eph[sat - 1 + MAXSAT].toc) == 0.0)
      return 0;
  }
  raw->nav.eph[sat - 1 + MAXSAT] = eph;
  raw->ephsat = sat;
  raw->ephset = 1; /* 1:F/NAV */
  return 2;
}
/* decode SBF Galileo I/NAV navigation page ----------------------------------*/
static int decode_galrawinav(raw_t *raw) {
  eph_t eph = {0};
  double ion[4] = {0}, utc[8] = {0};
  uint8_t *p = raw->buff + 14, buff[32], type, part1, part2, page1, page2;
  int i, j, svid, src, sat, prn;

  if (strstr(raw->opt, "-GALFNAV"))
    return 0;

  if (raw->len < 52) {
    trace(2, "sbf galrawinav length error: len=%d\n", raw->len);
    return -1;
  }
  svid = U1(p);
  src = U1(p + 3) & 0x1f;

  if (!(sat = svid2sat(svid)) || satsys(sat, &prn) != SYS_GAL) {
    trace(2, "sbf galrawinav svid error: svid=%d src=%d\n", svid, src);
    return -1;
  }
  if (!U1(p + 1)) {
    trace(3, "sbf galrawinav parity/crc error: prn=%d src=%d\n", prn, src);
    return 0;
  }
  if (raw->outtype) {
    sprintf(raw->msgtype + strlen(raw->msgtype), " prn=%d src=%d", prn, src);
  }
  if (src != 17 && src != 21 && src != 22) { /* E1, E5b or E5 AltBOC */
    trace(2, "sbf galrawinav source error: prn=%d src=%d\n", prn, src);
    return -1;
  }
  for (i = 0, p += 6; i < 8; i++, p += 4) {
    setbitu(buff, 32 * i, 32, U4(p)); /* 114(even) + 120(odd) bits */
  }
  part1 = getbitu(buff, 0, 1);
  page1 = getbitu(buff, 1, 1);
  part2 = getbitu(buff, 114, 1);
  page2 = getbitu(buff, 115, 1);

  if (part1 != 0 || part2 != 1) {
    trace(3, "sbf galrawinav part error: prn=%d even/odd=%d %d\n", prn, part1,
          part2);
    return -1;
  }
  if (page1 == 1 || page2 == 1)
    return 0; /* alert page */

  type = getbitu(buff, 2, 6); /* word type */

  if (type > 6)
    return 0;

  /* save 128 (112:even+16:odd) bits word (16 bytes * 7 word) */
  for (i = 0, j = 2; i < 14; i++, j += 8) {
    raw->subfrm[sat - 1][type * 16 + i] = getbitu(buff, j, 8);
  }
  for (i = 14, j = 116; i < 16; i++, j += 8) {
    raw->subfrm[sat - 1][type * 16 + i] = getbitu(buff, j, 8);
  }
  if (type != 5)
    return 0;
  if (!decode_gal_inav(raw->subfrm[sat - 1], &eph, ion, utc))
    return 0;

  if (eph.sat != sat) {
    trace(2, "sbf galrawinav satellite error: sat=%d %d\n", sat, eph.sat);
    return -1;
  }
  eph.code |= (src == 17) ? (1 << 0) : (1 << 2); /* data source: E1 or E5b */

  adj_utcweek(raw->time, utc);
  matcpy(raw->nav.ion_gal, ion, 4, 1);
  matcpy(raw->nav.utc_gal, utc, 8, 1);

  if (!strstr(raw->opt, "-EPHALL")) {
    if (eph.iode == raw->nav.eph[sat - 1].iode &&
        timediff(eph.toe, raw->nav.eph[sat - 1].toe) == 0.0 &&
        timediff(eph.toc, raw->nav.eph[sat - 1].toc) == 0.0)
      return 0;
  }
  raw->nav.eph[sat - 1] = eph;
  raw->ephsat = sat;
  raw->ephset = 0; /* 0:I/NAV */
  return 2;
}
/* decode SBF SBAS L1 navigation frame ---------------------------------------*/
static int decode_georawl1(raw_t *raw) {
  uint8_t *p = raw->buff + 14, buff[32];
  int i, svid, sat, prn;

  if (raw->len < 52) {
    trace(2, "sbf georawl1 length error: len=%d\n", raw->len);
    return -1;
  }
  svid = U1(p);
  if (!(sat = svid2sat(svid)) || satsys(sat, &prn) != SYS_SBS) {
    trace(2, "sbf georawl1 svid error: svid=%d\n", svid);
    return -1;
  }
  if (!U1(p + 1)) {
    trace(3, "sbf georawl1 parity/crc error: prn=%d err=%d\n", prn, U1(p + 2));
    return 0;
  }
  if (raw->outtype) {
    sprintf(raw->msgtype + strlen(raw->msgtype), " prn=%d", prn);
  }
  raw->sbsmsg.tow = (int)time2gpst(raw->time, &raw->sbsmsg.week);
  raw->sbsmsg.prn = prn;

  for (i = 0; i < 8; i++) {
    setbitu(buff, 32 * i, 32, U4(p + 6 + 4 * i));
  }
  memcpy(raw->sbsmsg.msg, buff, 29); /* 226 bits w/o CRC */
  raw->sbsmsg.msg[28] &= 0xC0;
  return 3;
}
/* decode SBF BDS navigation frame -------------------------------------------*/
static int decode_bdsraw(raw_t *raw) {
  eph_t eph = {0};
  double ion[8], utc[8];
  uint8_t *p = raw->buff + 14, buff[40];
  int i, id, svid, sat, prn, pgn;

  if (raw->len < 52) {
    trace(2, "sbf bdsraw length error: len=%d\n", raw->len);
    return -1;
  }
  svid = U1(p);
  if (!(sat = svid2sat(svid)) || satsys(sat, &prn) != SYS_CMP) {
    trace(2, "sbf bdsraw svid error: svid=%d\n", svid);
    return -1;
  }
  if (!U1(p + 1)) {
    trace(3, "sbf bdsraw parity/crc error: prn=%d\n", prn);
    return 0;
  }
  if (raw->outtype) {
    sprintf(raw->msgtype + strlen(raw->msgtype), " prn=%d", prn);
  }
  for (i = 0, p += 6; i < 10; i++, p += 4) {
    setbitu(buff, 32 * i, 32, U4(p));
  }
  id = getbitu(buff, 15, 3); /* subframe ID */
  if (id < 1 || id > 5) {
    trace(2, "sbf bdsraw id error: prn=%d id=%d\n", prn, id);
    return -1;
  }
  if (prn >= 6 && prn <= 58) { /* IGSO/MEO */
    memcpy(raw->subfrm[sat - 1] + (id - 1) * 38, buff, 38);

    if (id == 3) {
      if (!decode_bds_d1(raw->subfrm[sat - 1], &eph, NULL, NULL))
        return 0;
    } else if (id == 5) {
      if (!decode_bds_d1(raw->subfrm[sat - 1], NULL, ion, utc))
        return 0;
      matcpy(raw->nav.ion_cmp, ion, 8, 1);
      matcpy(raw->nav.utc_cmp, utc, 8, 1);
      return 9;
    } else
      return 0;
  } else {                      /* GEO */
    pgn = getbitu(buff, 42, 4); /* page number */

    if (id == 1 && pgn >= 1 && pgn <= 10) {
      memcpy(raw->subfrm[sat - 1] + (pgn - 1) * 38, buff, 38);
      if (pgn != 10)
        return 0;
      if (!decode_bds_d2(raw->subfrm[sat - 1], &eph, NULL))
        return 0;
    } else if (id == 1 && pgn == 102) {
      memcpy(raw->subfrm[sat - 1] + 10 * 38, buff, 38);
      if (!decode_bds_d2(raw->subfrm[sat - 1], NULL, utc))
        return 0;
      matcpy(raw->nav.utc_cmp, utc, 8, 1);
      return 9;
    } else
      return 0;
  }
  if (!strstr(raw->opt, "-EPHALL")) {
    if (timediff(eph.toe, raw->nav.eph[sat - 1].toe) == 0.0)
      return 0;
  }
  eph.sat = sat;
  raw->nav.eph[sat - 1] = eph;
  raw->ephsat = sat;
  raw->ephset = 0;
  return 2;
}
/* decode SBF QZS C/A subframe -----------------------------------------------*/
static int decode_qzsrawl1ca(raw_t *raw) { return decode_rawca(raw, SYS_QZS); }
/* decode SBF NavIC/IRNSS subframe -------------------------------------------*/
static int decode_navicraw(raw_t *raw) {
  eph_t eph = {0};
  double ion[8], utc[9];
  uint8_t *p = raw->buff + 14, buff[40];
  int i, id, svid, sat, prn, ret = 0;

  if (raw->len < 52) {
    trace(2, "sbf navicraw length error: len=%d\n", raw->len);
    return -1;
  }
  svid = U1(p);
  if (!(sat = svid2sat(svid)) || satsys(sat, &prn) != SYS_IRN) {
    trace(2, "sbf navicraw svid error: svid=%d\n", svid);
    return -1;
  }
  if (!U1(p + 1)) {
    trace(3, "sbf navicraw parity/crc error: prn=%d err=%d\n", prn, U1(p + 2));
    return 0;
  }
  if (raw->outtype) {
    sprintf(raw->msgtype + strlen(raw->msgtype), " prn=%d", prn);
  }
  for (i = 0, p += 6; i < 10; i++, p += 4) {
    setbitu(buff, 32 * i, 32, U4(p));
  }
  id = getbitu(buff, 27, 2); /* subframe ID (0-3) */

  memcpy(raw->subfrm[sat - 1] + id * 37, buff, 37);

  if (id == 1) { /* subframe 2 */
    if (!decode_irn_nav(raw->subfrm[sat - 1], &eph, NULL, NULL))
      return 0;

    if (!strstr(raw->opt, "-EPHALL")) {
      if (eph.iode == raw->nav.eph[sat - 1].iode &&
          timediff(eph.toe, raw->nav.eph[sat - 1].toe) == 0.0) {
        return 0;
      }
    }
    eph.sat = sat;
    raw->nav.eph[sat - 1] = eph;
    raw->ephsat = sat;
    raw->ephset = 0;
    return 2;
  } else if (id == 2 || id == 3) { /* subframe 3 or 4 */
    if (decode_irn_nav(raw->subfrm[sat - 1], NULL, ion, NULL)) {
      matcpy(raw->nav.ion_irn, ion, 8, 1);
      ret = 9;
    }
    if (decode_irn_nav(raw->subfrm[sat - 1], NULL, NULL, utc)) {
      adj_utcweek(raw->time, utc);
      matcpy(raw->nav.utc_irn, utc, 9, 1);
      ret = 9;
    }
    memset(raw->subfrm[sat - 1] + id * 37, 0, 37);
    return ret;
  }
  return 0;
}
/* decode SBF galalm --------------------------------------------------------*/
static int decode_galalm(raw_t *raw) {
  uint8_t *p = (raw->buff) + 8; /* points at TOW location */
  alm_t alm;

  trace(4, "SBF decode_galalm: len=%d\n", raw->len);

  if (raw->len < 62) {
    trace(1, "SBF decode_galalm: Block too short\n");
    return -1;
  }

  alm.sat = satno(SYS_GAL, U1(p + 49) - 70);
  alm.e = R4(p + 8);
  alm.toas = U4(p + 12);
  alm.i0 = R4(p + 16) + 0.3;
  alm.OMGd = R4(p + 20);
  alm.A = pow(R4(p + 24), 2);
  alm.OMG0 = R4(p + 28);
  alm.omg = R4(p + 32);
  alm.M0 = R4(p + 36);
  alm.f1 = R4(p + 40);
  alm.f0 = R4(p + 44);
  alm.week = U1(p + 48);
  alm.toa = gpst2time(alm.week, alm.toas);
  alm.svconf = 0;
  alm.svh = 0;

  if (alm.sat == 0)
    return -1;
  raw->nav.alm[alm.sat - 1] = alm;

  return 9;
}
  return 9;
}
/* decode SBF gpsalm --------------------------------------------------------*/
static int decode_gpsalm(raw_t *raw) {
  uint8_t *p = (raw->buff) + 8; /* points at TOW location */
  alm_t alm;

  trace(4, "SBF decode_gpsalm: len=%d\n", raw->len);

  if (raw->len < 60) {
    trace(1, "SBF decode_gpsalm: Block too short\n");
    return -1;
  }

  alm.sat = satno(SYS_GPS, U1(p + 6));
  alm.e = R4(p + 8);
  alm.toas = U4(p + 12);
  alm.i0 = R4(p + 16);
  alm.OMGd = R4(p + 20);
  alm.A = pow(R4(p + 24), 2);
  alm.OMG0 = R4(p + 28);
  alm.omg = R4(p + 32);
  alm.M0 = R4(p + 36);
  alm.f1 = R4(p + 40);
  alm.f0 = R4(p + 44);
  alm.week = U1(p + 48);
  alm.toa = gpst2time(alm.week, alm.toas);
  alm.svconf = U1(p + 49);
  alm.svh = U1(p + 50);

  if (alm.sat == 0)
    return -1;

  raw->nav.alm[alm.sat - 1] = alm;

  return 9;
}

/* decode SBF nav message for Galileo (navigation data)
 * --------------------------*/
static int decode_galnav(raw_t *raw) {

  uint8_t *puiTmp = (raw->buff) + 6;
  eph_t eph = {0};
  double toc;
  int prn, sat;
  uint16_t week_oe, week_oc;
  uint32_t tow;

  trace(4, "SBF decode_galnav: len=%d\n", raw->len);

  if ((raw->len) < 152) {
    trace(2, "SBF decode_galnav frame length error: len=%d\n", raw->len);
    return -1;
  }

  prn = U1(puiTmp + 8) - 70;
  sat = satno(SYS_GAL, prn);

  if (sat == 0)
    return -1;

  if (!((prn >= 1) && (prn <= 36))) {
    trace(2, "SBF decode_galnav prn error: sat=%d\n", prn);
    return -1;
  }

  tow = U4(puiTmp + 2);
  eph.week = U2(puiTmp + 6);              /* GAL week number */
  eph.code = U1(puiTmp + 9) == 2 ? 0 : 1; /* 0:INAV,1:FNAV */
  eph.A = pow(R8(puiTmp + 10), 2);
  eph.M0 = R8(puiTmp + 18) * PI;
  eph.e = R8(puiTmp + 26);
  eph.i0 = R8(puiTmp + 34) * PI;
  eph.omg = R8(puiTmp + 42) * PI;
  eph.OMG0 = R8(puiTmp + 50) * PI;
  eph.OMGd = R4(puiTmp + 58) * PI;
  eph.idot = R4(puiTmp + 62) * PI;
  eph.deln = R4(puiTmp + 66) * PI;
  eph.cuc = R4(puiTmp + 70);
  eph.cus = R4(puiTmp + 74);
  eph.crc = R4(puiTmp + 78);
  eph.crs = R4(puiTmp + 82);
  eph.cic = R4(puiTmp + 86);
  eph.cis = R4(puiTmp + 90);
  eph.toes = U4(puiTmp + 94);
  toc = U4(puiTmp + 98);
  eph.f2 = R4(puiTmp + 102);
  eph.f1 = R4(puiTmp + 106);
  eph.f0 = R8(puiTmp + 110);
  week_oe = U2(puiTmp + 118); /* WNt_oc */
  week_oc = U2(puiTmp + 120);
  eph.iode = eph.iodc = U2(puiTmp + 122);
  if (eph.code == 0) /* INAV */
  {
    eph.sva = U1(puiTmp + 128);
    eph.svh = (U2(puiTmp + 124) & 0x00ff) ^ 0x0011;
  } else { /* FNAV */
    eph.sva = U1(puiTmp + 127);
    eph.svh = (U2(puiTmp + 124) & 0x0f0f) ^ 0x0101;
  }

  eph.tgd[0] = R4(puiTmp + 130);
  eph.tgd[1] = R4(puiTmp + 134);
  eph.fit = 0;

  week_oe = adjgpsweek(week_oe);
  week_oc = adjgpsweek(week_oc);
  eph.toe = gpst2time(week_oe, eph.toes);
  eph.toc = gpst2time(week_oc, toc);
  eph.ttr = gpst2time(eph.week, tow / 1000);

  if (!strstr(raw->opt, "-EPHALL")) {
    if (eph.iode == raw->nav.eph[sat - 1].iode)
      return 0;
  }

  if (sat == 0)
    return -1;

  eph.sat = sat;
  raw->nav.eph[sat - 1] = eph;
  raw->ephsat = sat;
  return 2;
}

/* decode SBF nav message for glonass (navigation data) ----------------------*/
static int decode_glonav(raw_t *raw) {

  uint8_t *puiTmp = (raw->buff) + 6;
  geph_t eph = {0};
  int prn, sat;
  uint16_t week;

  trace(4, "SBF decode_glonav: len=%d\n", raw->len);

  if ((raw->len) < 96) {
    trace(2, "SBF decode_glonav frame length error: len=%d\n", raw->len);
    return -1;
  }
  prn = U1(puiTmp + 8) - 37;
  sat = satno(SYS_GLO, prn);

  if (sat == 0)
    return -1;

  if (!((prn >= 1) && (prn <= 24))) {
    trace(2, "SBF decode_glonav prn error: sat=%d\n", prn);
    return -1;
  }

  eph.frq = U1(puiTmp + 9) - 8;
  eph.pos[0] = R8(puiTmp + 10) * 1000;
  eph.pos[1] = R8(puiTmp + 18) * 1000;
  eph.pos[2] = R8(puiTmp + 26) * 1000;
  eph.vel[0] = R4(puiTmp + 34) * 1000;
  eph.vel[1] = R4(puiTmp + 38) * 1000;
  eph.vel[2] = R4(puiTmp + 42) * 1000;
  eph.acc[0] = R4(puiTmp + 46) * 1000;
  eph.acc[1] = R4(puiTmp + 50) * 1000;
  eph.acc[2] = R4(puiTmp + 54) * 1000;
  eph.gamn = R4(puiTmp + 58);
  eph.taun = R4(puiTmp + 62);
  eph.dtaun = R4(puiTmp + 66);
  week = U2(puiTmp + 74); /* WN_toe modulo 1024 */
  week = adjgpsweek(week);

  eph.toe = gpst2time(week, U4(puiTmp + 70));
  eph.tof = raw->time;
  eph.age = U1(puiTmp + 78);
  eph.svh = U1(puiTmp + 79);
  eph.iode = U2(puiTmp + 80);
  eph.sva = U2(puiTmp + 88);

  if (!strstr(raw->opt, "-EPHALL")) {
    if (eph.iode == raw->nav.geph[prn - 1].iode)
      return 0;
  }

  eph.sat = sat;
  raw->nav.geph[prn - 1] = eph;
  raw->ephsat = sat;
  raw->nav.glo_fcn[prn - 1] = eph.frq + 8; /* savbe frequency number */

  return 2;
}

/* adjust daily rollover of time ---------------------------------------------*/
static gtime_t adjday(gtime_t time, double tod) {
  double ep[6], tod_p;
  time2epoch(time, ep);
  tod_p = ep[3] * 3600.0 + ep[4] * 60.0 + ep[5];
  if (tod < tod_p - 43200.0)
    tod += 86400.0;
  else if (tod > tod_p + 43200.0)
    tod -= 86400.0;
  ep[3] = ep[4] = ep[5] = 0.0;
  return timeadd(epoch2time(ep), tod);
}

/* decode SBF nav message for sbas (navigation data) ----------------------*/
static int decode_sbasnav(raw_t *raw) {

  uint8_t *puiTmp = (raw->buff) + 6;
  seph_t eph = {0};
  int prn, sat;
  uint16_t week;
  uint32_t tod, tow;

  trace(4, "SBF decode_sbasnav: len=%d\n", raw->len);

  if ((raw->len) < 104) {
    trace(2, "SBF decode_sbasnav frame length error: len=%d\n", raw->len);
    return -1;
  }
  prn = U1(puiTmp + 8);
  sat = satno(SYS_SBS, prn);

  if (!((prn >= 120) && (prn <= 140))) {
    trace(2, "SBF decode_sbasnav prn error: sat=%d\n", prn);
    return -1;
  }

  if (sat == 0)
    return -1;

  week = U2(puiTmp + 6);
  tow = U4(puiTmp + 2) / 1000;
  tod = U4(puiTmp + 14);
  eph.tof = gpst2time(adjgpsweek(week), tow);
  eph.t0 = adjday(eph.tof, tod);
  eph.sva = U2(puiTmp + 12);
  eph.svh = eph.sva == 15 ? 1 : 0;
  eph.pos[0] = R8(puiTmp + 18);
  eph.pos[1] = R8(puiTmp + 26);
  eph.pos[2] = R8(puiTmp + 34);
  eph.vel[0] = R8(puiTmp + 42);
  eph.vel[1] = R8(puiTmp + 50);
  eph.vel[2] = R8(puiTmp + 58);
  eph.acc[0] = R8(puiTmp + 66);
  eph.acc[1] = R8(puiTmp + 74);
  eph.acc[2] = R8(puiTmp + 82);
  eph.af0 = R4(puiTmp + 90);
  eph.af1 = R4(puiTmp + 94);

  /* debug */
  trace(2, "sat=%2d, week=%d, tow=%f\n", sat, week, U4(puiTmp + 2) / 1000);

  if (!strstr(raw->opt, "-EPHALL")) {
    if (fabs(timediff(eph.t0, raw->nav.seph[prn - 120].t0)) < 1.0 &&
        eph.sva == raw->nav.seph[prn - 120].sva)
      return 0;
  }

  eph.sat = sat;
  raw->nav.seph[prn - 120] = eph;
  raw->ephsat = eph.sat;
  return 2;
}

/* decode SBF raw nav message (raw navigation data) --------------------------*/
static int decode_rawnav(raw_t *raw, int sys) {

  /* NOTE. This function works quite well but it somestimes fails in line:
   * if (resp>5 || resp<=0){
   * To debug the problem an understanding of the whole RTK code is needed
   */

  uint8_t *p = (raw->buff) + 6, id;
  eph_t eph = {0};
  int sat, prn;
  uint8_t _buf[30] = {0};
  int i = 0, ii = 0;

  trace(3, "SBF decode_gpsrawcanav: len=%d\n", raw->len);

  if (raw->len < 58) {
    trace(2, "SBF decode_gpsrawcanav block length error: len=%d\n", raw->len);
    return -1;
  }

  /* get GPS satellite number */
  prn = U1(p + 8);
  if (sys == SYS_QZS)
    prn -= 180;

  sat = satno(sys, prn);
  if (sat == 0)
    return -1;

  /* clean up subframe from Septentrio. This is a little bit of work because
   * Septentrio Rx add some parity bits to this message.
   * We have to throw away the reserved bits as well as the parity bits.
   */

  /*   | 2bits |         24bits        |  6bits  |       <- SBF 32-bit word
      ------------------------------------------
               | byte1 | bite2 | byte3 |                 <- sat nav message
  */

  for (i = 0; i < 40; i += 4) {
    _buf[ii] = ((U4(p + 14 + i) >> 22) & 0x000000FF);     /* take first byte  */
    _buf[1 + ii] = ((U4(p + 14 + i) >> 14) & 0x000000FF); /* take second byte */
    _buf[2 + ii] = ((U4(p + 14 + i) >> 6) & 0x000000FF);  /* take third byte  */
    ii = ii + 3;
  }

  /* Now that we have a classic subframe we call the generic function */
  id = getbitu(_buf, 43, 3); /* get subframe id */
  if ((id < 1) || (id > 5))
    return -1;

  memcpy(raw->subfrm[sat - 1] + (id - 1) * 30, _buf, 30);

  if (decode_frame(raw->subfrm[sat - 1], &eph, NULL, NULL, NULL) == 1 &&
      decode_frame(raw->subfrm[sat - 1] + 30, &eph, NULL, NULL, NULL) == 2 &&
      decode_frame(raw->subfrm[sat - 1] + 60, &eph, NULL, NULL, NULL) == 3) {

    if (!strstr(raw->opt, "-EPHALL")) {
      if ((eph.iode == raw->nav.eph[sat - 1].iode) &&
          (eph.iodc == raw->nav.eph[sat - 1].iodc))
        return 0;
    }
  if (id == 4) {
    if (sys == SYS_GPS) {
      decode_frame(raw->subfrm[sat - 1] + 90, NULL, raw->nav.alm,
                   raw->nav.ion_gps, raw->nav.utc_gps);
      adj_utcweek(raw->time, raw->nav.utc_gps);
    } else if (sys == SYS_QZS) {
      decode_frame(raw->subfrm[sat - 1] + 90, NULL, raw->nav.alm,
                   raw->nav.ion_qzs, raw->nav.utc_qzs);
      adj_utcweek(raw->time, raw->nav.utc_qzs);
    }
    }
static int decode_sbf(raw_t *raw) {
  uint8_t *p = raw->buff;
  uint32_t week, tow;
  char tstr[32];
  int type = U2(p + 4) & 0x1fff;
  uint16_t crc;

  crc = rtk_crc16(p + 4, raw->len - 4);
  if (crc != U2(p + 2) || crc == 0) {
    trace(2, "sbf crc error: type=%d len=%d\n", type, raw->len);
    return -1;
  }
  if (raw->len < 14) {
    trace(2, "sbf length error: type=%d len=%d\n", type, raw->len);
    return -1;
  }
  tow = U4(p + 8);
  week = U2(p + 12);
  if (tow == 4294967295u || week == 65535u) {
    trace(2, "sbf tow/week error: type=%d len=%d\n", type, raw->len);
    return -1;
  }
  raw->time = gpst2time(week, tow * 0.001);

  if (raw->outtype) {
    time2str(raw->time, tstr, 2);
    sprintf(raw->msgtype, "SBF %4d (%4d): %s", type, raw->len, tstr);
  }
  switch (type) {
  case SBF_MEASEPOCH:
    return decode_measepoch(raw);
  case SBF_MEASEXTRA:
    return decode_measextra(raw);
  case SBF_GPSRAWCA:
    return decode_gpsrawca(raw);
  case SBF_GLORAWCA:
    return decode_glorawca(raw);
  case SBF_GALRAWFNAV:
    return decode_galrawfnav(raw);
  case SBF_GALRAWINAV:
    return decode_galrawinav(raw);
  case SBF_GEORAWL1:
    /*  case SBF_GEORAWL5: Needs testing*/
    return decode_georawl1(raw);
  case SBF_BDSRAW:
    return decode_bdsraw(raw);
  case SBF_QZSRAWL1CA:
    return decode_qzsrawl1ca(raw);
  case SBF_NAVICRAW:
    return decode_navicraw(raw);

  case SBF_GPSNAV:
    return decode_gpsnav(raw);
  case SBF_GPSION:
    return decode_gpsion(raw);
  case SBF_GPSUTC:
    return decode_gpsutc(raw);
  case SBF_GPSALM:
    return decode_gpsalm(raw);
  case SBF_GPSRAWL2C:
  case SBF_GPSRAWL5:
    return decode_rawnav(raw, SYS_GPS);

  case SBF_GEONAV:
    return decode_sbasnav(raw);
  case SBF_GLONAV:
    return decode_glonav(raw);
  case SBF_GLOTIME:
    return decode_gloutc(raw);
  case SBF_GALNAV:
    return decode_galnav(raw);
  case SBF_GALION:
    return decode_galion(raw);
  case SBF_GALUTC:
    return decode_galutc(raw);
  case SBF_GALALM:
    return decode_galalm(raw);

    /* Ones we _know_ we wont handle*/
  case SBF_PVTGEOD:
    trace(4, "sbf unused message - PVT Geodetic\n");
    return 0;
  case SBF_DOP:
    trace(4, "sbf unused message - Dop\n");
    return 0;
  case SBF_MEASEPOCH_END:
    trace(4, "sbf unused message - End of measurements\n");
    return 0;
  case SBF_ENDOFPVT:
    trace(4, "sbf unused message - End of PVT\n");
    return 0;
  case SBF_CHNSTATUS:
    trace(4, "sbf unused message - receiver channels status\n");
    return 0;
  case SBF_BASEVECGEOD:
    trace(4, "sbf unused message - base station vector\n");
    return 0;
  case SBF_ATTEULER:
    trace(4, "sbf unused message - euler angles\n");
    return 0;
  case SBF_POSCOVGEO:
    trace(4, "sbf unused message - position covariance matrix\n");
    return 0;

  case SBF_GEOALM:
    trace(4, "sbf unused message - SBAS satellite almanac \n");
    return 0;

  case SBF_GEOSERVICELEVEL:
    trace(4, "sbf unused message - SBAS Service Message \n");
    return 0;

  case SBF_GEONETWORKTIME:
    trace(4, "sbf unused message - SBAS Network Time/UTC offset parameters \n");
    return 0;

  case SBF_GEOMT00:
    trace(4, "sbf unused message - SBAS: Don't use for safety application \n");
    return 0;

  case SBF_GEOPRNMASK:
    trace(4, "sbf unused message - PRN Mask assignments \n");
    return 0;

  case SBF_GEOFASTCORR:
    trace(4, "sbf unused message - Fast Corrections \n");
    return 0;

  case SBF_GEOINTEGRITY:
    trace(4, "sbf unused message - Integrity information \n");
    return 0;

  case SBF_GEOFASTCORRDEGR:
    trace(4, "sbf unused message - fast correction degradation factor \n");
    return 0;

  case SBF_GEODEGRFACTORS:
    trace(4, "sbf unused message - Degration factors \n");
    return 0;

  case SBF_GEOIGPMASK:
    trace(4, "sbf unused message - Ionospheric grid point mask \n");
    return 0;

  case SBF_GEOLONGTERMCOR:
    trace(4, "sbf unused message - Long term satellite error corrections \n");
    return 0;

  case SBF_GEOIONODELAY:
    trace(4, "sbf unused message - Inospheric delay correction \n");
    return 0;

  case SBF_GEOCLOCKEPHCOVMATRIX:
    trace(4, "sbf unused message - Clock-Ephemeris Covariance Matrix\n");
    return 0;
  case SBF_QUALITY_IND:
    trace(4, "sbf unused message - Quality Indicators\n");
    return 0;
  case SBF_PVTSUPPORT:
    trace(4, "sbf unused message - PVT support params\n");
    return 0;
  case 4090:
  case 4014:
  case 4053:
  case 4092:
  case 4201:
  case 4105:
  case 4079:
  case 5924:
  case 4059:
  case 4091:
    return 0;
  }
  trace(3, "sbf unsupported message: type=%d\n", type);
  return 0;
}
/* synchronize SBF block header ----------------------------------------------*/
static int sync_sbf(uint8_t *buff, uint8_t data) {
  buff[0] = buff[1];
  buff[1] = data;
  return buff[0] == SBF_SYNC1 && buff[1] == SBF_SYNC2;
}
/* input SBF raw data from stream ----------------------------------------------
 * fetch next SBF raw data and input a mesasge from stream
 * args   : raw_t *raw       IO  receiver raw data control struct
 *          uint_t data      I   stream data (1 byte)
 * return : status (-1: error message, 0: no message, 1: input observation data,
 *                  2: input ephemeris, 3: input sbas message,
 *                  9: input ion/utc parameter)
 *
 * notes  : supported SBF block (block ID):
 *
 *           MEASEPOCH(4027), GPSRAWCA(4017), GLORAWCA(4026), GALRAWFNAV(4022),
 *           GALRAWINAV(4023), GEORAWL1(4020), BDSRAW(4047), QZSRAWL1CA(4066),
 *           NAVICRAW(4093)
 *
 *          to specify input options for sbf, set raw->opt to the following
 *          option strings separated by spaces.
 *
 *          -EPHALL : input all ephemerides
 *          -AUX1   : select antenna Aux1  (default: main)
 *          -AUX2   : select antenna Aux2  (default: main)
 *          -GL1W   : select 1W for GPS L1 (default: 1C)
 *          -GL1L   : select 1L for GPS L1 (default: 1C)
 *          -GL2L   : select 2L for GPS L2 (default: 2W)
 *          -RL1P   : select 1P for GLO G1 (default: 1C)
 *          -RL2C   : select 2C for GLO G2 (default: 2P)
 *          -JL1L   : select 1L for QZS L1 (default: 1C)
 *          -JL1Z   : select 1Z for QZS L1 (default: 1C)
 *          -CL1P   : select 1P for BDS B1 (default: 2I)
 *          -GALINAV: select I/NAV for Galileo ephemeris (default: all)
 *          -GALFNAV: select F/NAV for Galileo ephemeris (default: all)
 *-----------------------------------------------------------------------------*/
extern int input_sbf(raw_t *raw, uint8_t data) {
  trace(5, "input_sbf: data=%02x\n", data);

  if (raw->nbyte == 0) {
    if (sync_sbf(raw->buff, data))
      raw->nbyte = 2;
    return 0;
  }
  raw->buff[raw->nbyte++] = data;
  if (raw->nbyte < 8)
    return 0;

  if ((raw->len = U2(raw->buff + 6)) > MAXRAWLEN) {
    trace(2, "sbf length error: len=%d\n", raw->len);
    raw->nbyte = 0;
    return -1;
  }
  if (raw->nbyte < raw->len)
    return 0;
  raw->nbyte = 0;

  /* decode SBF block */
  return decode_sbf(raw);
}
/* input SBF raw data from file ------------------------------------------------
 * fetch next SBF raw data and input a message from file
 * args   : raw_t *raw       IO  receiver raw data control struct
 *          FILE  *fp        I   file pointer
 * return : status(-2: end of file, -1...9: same as above)
 *-----------------------------------------------------------------------------*/
extern int input_sbff(raw_t *raw, FILE *fp) {
  int i, data;

  trace(4, "input_sbff:\n");

  if (raw->nbyte == 0) {
    for (i = 0;; i++) {
      if ((data = fgetc(fp)) == EOF)
        return -2;
      if (sync_sbf(raw->buff, (uint8_t)data))
        break;
      if (i >= 4096)
        return 0;
    }
  }
  if (fread(raw->buff + 2, 1, 6, fp) < 6)
    return -2;
  raw->nbyte = 8;

  if ((raw->len = U2(raw->buff + 6)) > MAXRAWLEN) {
    trace(2, "sbf length error: len=%d\n", raw->len);
    raw->nbyte = 0;
    return -1;
  }
  if (raw->len != 0 && fread(raw->buff + 8, raw->len - 8, 1, fp) < 1)
    return -2;
  raw->nbyte = 0;

  /* decode SBF block */
  return decode_sbf(raw);
}
