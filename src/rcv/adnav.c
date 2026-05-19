/*------------------------------------------------------------------------------
 * adnav.c : Advanced Navigation Packet Protocol (ANPP) decoder
 *
 *          Copyright (C) 2026 by Joseph Fox-Rabinovitz, All rights reserved.
 *
 * reference :
 *     [1] Advanced Navigation, Advanced Navigation Packet Protocol Reference,
 *         https://docs.advancednavigation.com/boreas-d/ANPP/Advanced%20Navigation%20Packet.htm
 *
 * history : 2026/05/05 1.0 new
 *-----------------------------------------------------------------------------*/
#include <string.h>

#include "rtklib.h"

/* ANPP packet IDs */
#define ID_SYSTEMSTATE 20 /* system state */
#define ID_UNIXTIME 21    /* unix time */
#define ID_RAWSATDATA 60  /* raw satellite data */
#define ID_RAWSATEPH 61   /* raw satellite ephemeris */

/* ANPP satellite system IDs (packets 60/61 sys field) */
#define ANPP_SYS_UNK 0  // Unknown
#define ANPP_SYS_GPS 1  // GPS
#define ANPP_SYS_GLO 2  // GLONASS
#define ANPP_SYS_BDS 3  // BeiDou
#define ANPP_SYS_GAL 4  // Galileo
#define ANPP_SYS_SBS 5  // SBAS
#define ANPP_SYS_QZS 6  // QZSS
//                      7 // Reserved
#define ANPP_SYS_OMN 8  // OmniStar
//                      9 // Reserved
#define ANPP_SYS_NAV 10  // NavIC

/* header size for all packets */
#define HDR_LEN 5

/* packet 20 payload length */
#define SYSSTATE_LEN 100 /* system state */

/* packet 21 payload length */
#define UNIXTIME_LEN 8 /* unix time */

/* packet 60 sub-record sizes */
#define OBS_HDR_LEN 16  /* per-epoch header (time + counters) */
#define SAT_HDR_LEN 6   /* per-satellite header */
#define FREQ_BLK_LEN 26 /* per-frequency block size */

/* tracking status bits (packet 60, per-frequency block, byte 1) */
#define TRKS_CARRIER 0x01     /* carrier phase valid */
#define TRKS_SLIP 0x02        /* cycle slip detected */
#define TRKS_HALFCYCLE 0x04   /* half-cycle ambiguity unresolved */
#define TRKS_PSEUDORANGE 0x08 /* pseudorange valid */
#define TRKS_DOPPLER 0x10     /* Doppler valid */
#define TRKS_SNR 0x20         /* SNR valid */

/* packet 61 payload lengths */
#define EPH_LEN_GPS 132 /* GPS ephemeris */
#define EPH_LEN_GLO 94  /* GLONASS ephemeris */

/* get fields (little-endian) ------------------------------------------------*/
static uint8_t U1(const raw_t* raw, size_t index) {
  RTKBOUNDSCHECK(raw->buff, sizeof(raw->buff), index);
  RTKBOUNDSCHECK(raw->buff, raw->len, index);
  return raw->buff[index];
}
static int8_t I1(const raw_t* raw, size_t index) {
  RTKBOUNDSCHECK(raw->buff, sizeof(raw->buff), index);
  RTKBOUNDSCHECK(raw->buff, raw->len, index);
  return (int8_t)raw->buff[index];
}
static uint16_t U2(const raw_t* raw, size_t index) {
  RTKBOUNDSCHECK(raw->buff, sizeof(raw->buff), index + 1);
  RTKBOUNDSCHECK(raw->buff, raw->len, index + 1);
  uint16_t u;
  memcpy(&u, raw->buff + index, 2);
  return u;
}
static uint32_t U4(const raw_t* raw, size_t index) {
  RTKBOUNDSCHECK(raw->buff, sizeof(raw->buff), index + 3);
  RTKBOUNDSCHECK(raw->buff, raw->len, index + 3);
  uint32_t u;
  memcpy(&u, raw->buff + index, 4);
  return u;
}
static float R4(const raw_t* raw, size_t index) {
  RTKBOUNDSCHECK(raw->buff, sizeof(raw->buff), index + 3);
  RTKBOUNDSCHECK(raw->buff, raw->len, index + 3);
  float r;
  memcpy(&r, raw->buff + index, 4);
  return r;
}
static double R8(const raw_t* raw, size_t index) {
  RTKBOUNDSCHECK(raw->buff, sizeof(raw->buff), index + 7);
  RTKBOUNDSCHECK(raw->buff, raw->len, index + 7);
  double r;
  memcpy(&r, raw->buff + index, 8);
  return r;
}

/* receiver state ------------------------------------------------------------*/
typedef struct {
  gtime_t time;             /* last cached epoch time (UTC) */
  uint32_t cur_unix_time;   /* unix_time of packet-60 epoch being assembled */
  uint32_t cur_nanoseconds; /* nanoseconds of packet-60 epoch being assembled */
  int epoch_active;         /* 1 if obuf currently holds a partial epoch */
} anpp_t;

/* init/free -----------------------------------------------------------------*/
extern int init_anpp(raw_t* raw) {
  if (raw->format != STRFMT_ANPP) return 0;
  anpp_t* anpp = calloc(1, sizeof(anpp_t));
  if (!anpp) {
    trace(0, "init_anpp: memory allocation error\n");
    return 0;
  }
  raw->rcv_data = (void*)anpp;
  return 1;
}

extern void free_anpp(raw_t* raw) {
  if (raw->format != STRFMT_ANPP) return;
  if (raw->rcv_data) {
    free(raw->rcv_data);
    raw->rcv_data = NULL;
  }
}

/* initialize one obs buffer slot --------------------------------------------*/
static void init_obsd_anpp(gtime_t time, int sat, obsd_t* data) {
  data->time = time;
  data->sat = (uint8_t)sat;

  for (int i = 0; i < NFREQ + NEXOBS; i++) {
    data->L[i] = data->P[i] = 0.0;
    data->D[i] = data->SNR[i] = 0.0;
    data->Lstd[i] = data->Pstd[i] = 0.0;
    data->LLI[i] = 0;
    data->code[i] = CODE_NONE;
  }
}

/* flush observation data buffer ---------------------------------------------*/
static int flush_obuf_anpp(raw_t* raw) {
  trace(3, "flush_obuf_anpp: n=%d\n", raw->obuf.n);

  int n = 0;
  for (int i = 0; i < raw->obuf.n && i < MAXOBS; i++) {
    if (!satsys(raw->obuf.data[i].sat, NULL)) continue;
    if (raw->obuf.data[i].time.time == 0) continue;
    raw->obs.data[n++] = raw->obuf.data[i];
  }
  raw->obs.n = n;

  for (int i = 0; i < MAXOBS; i++) {
    raw->obuf.data[i].time.time = 0;
    raw->obuf.data[i].time.sec = 0.0;
    for (int j = 0; j < NFREQ + NEXOBS; j++) {
      raw->obuf.data[i].L[j] = raw->obuf.data[i].P[j] = 0.0;
      raw->obuf.data[i].D[j] = raw->obuf.data[i].SNR[j] = 0.0;
      raw->obuf.data[i].Lstd[j] = raw->obuf.data[i].Pstd[j] = 0.0;
      raw->obuf.data[i].LLI[j] = 0;
      raw->obuf.data[i].code[j] = CODE_NONE;
    }
  }
  return n > 0 ? 1 : 0;
}

/* map ANPP satellite system code to rtklib SYS_* ---------------------------*/
static int anpp2sys(uint8_t anpp_sys) {
  switch (anpp_sys) {
    case ANPP_SYS_UNK:
      return SYS_NONE;
    case ANPP_SYS_GPS:
      return SYS_GPS;
    case ANPP_SYS_GLO:
      return SYS_GLO;
    case ANPP_SYS_BDS:
      return SYS_CMP;
    case ANPP_SYS_GAL:
      return SYS_GAL;
    case ANPP_SYS_SBS:
      return SYS_SBS;
    case ANPP_SYS_QZS:
      return SYS_QZS;
    case ANPP_SYS_OMN:
      return SYS_SBS;
    case ANPP_SYS_NAV:
      return SYS_IRN;
  }
  return SYS_NONE;
}

/* map ANPP (sys, freq_code) to rtklib observation code ---------------------*/
static uint8_t anpp2code(uint8_t anpp_sys, uint8_t freq_code) {
  switch (anpp_sys) {
    case ANPP_SYS_GPS:
      switch (freq_code) {
        case 1:
          return CODE_L1C; /* L1 C/A */
        case 2:
          return CODE_L1L; /* L1 C */
        case 3:
          return CODE_L1P; /* L1 P */
        case 4:
          return CODE_L1M; /* L1 M */
        case 5:
          return CODE_L2L; /* L2 C */
        case 6:
          return CODE_L2P; /* L2 P */
        case 7:
          return CODE_L2M; /* L2 M */
        case 8:
          return CODE_L5Q; /* L5 */
      }
      break;
    case ANPP_SYS_GLO:
      switch (freq_code) {
        case 1:
          return CODE_L1C; /* G1 C/A */
        case 3:
          return CODE_L1P; /* G1 P */
        case 5:
          return CODE_L2C; /* G2 C/A */
        case 6:
          return CODE_L2P; /* G2 P */
        case 8:
          return CODE_L3Q; /* G3 */
      }
      break;
    case ANPP_SYS_GAL:
      switch (freq_code) {
        case 1:
          return CODE_L1C; /* E1 OS */
        case 2:
          return CODE_L1A; /* E1 PRS */
        case 5:
          return CODE_L6B; /* E6 CS */
        case 6:
          return CODE_L6A; /* E6 PRS */
        case 8:
          return CODE_L5Q; /* E5 a */
        case 9:
          return CODE_L7Q; /* E5 b */
        case 10:
          return CODE_L8Q; /* E5 a+b */
      }
      break;
    case ANPP_SYS_BDS:
      switch (freq_code) {
        case 1:
          return CODE_L2Q; /* B1 */
        case 5:
          return CODE_L7Q; /* B2 */
        case 8:
          return CODE_L6Q; /* B3 */
      }
      break;
    case ANPP_SYS_SBS:
      switch (freq_code) {
        case 1:
          return CODE_L1C; /* L1 C/A */
        case 8:
          return CODE_L5Q; /* L5 */
      }
      break;
    case ANPP_SYS_QZS:
      switch (freq_code) {
        case 1:
          return CODE_L1C; /* L1 C/A */
        case 2:
          return CODE_L1S; /* L1 C */
        case 3:
          return CODE_L1Z; /* L1 SAIF */
        case 5:
          return CODE_L2S; /* L2 C */
        case 6:
          return CODE_L6X; /* LEX */
        case 8:
          return CODE_L5P; /* L5 */
      }
      break;
    case ANPP_SYS_OMN:
      switch (freq_code) {
        case 0:
          return CODE_L1C; /* L1 C/A */
      }
      break;
    case ANPP_SYS_NAV:
      switch (freq_code) {
        case 8:
          return CODE_L5C; /* L5 */
      }
      break;
  }
  return CODE_NONE;
}

/* find or allocate-and-init obs buffer slot for satellite -------------------*/
static int obuf_slot(raw_t* raw, gtime_t time, int sat) {
  for (int i = 0; i < raw->obuf.n; i++) {
    if (raw->obuf.data[i].sat == sat) return i;
  }
  if (raw->obuf.n >= MAXOBS) {
    trace(2, "obuf_slot: buffer full sat=%d\n", sat);
    return -1;
  }

  int n = raw->obuf.n++;
  init_obsd_anpp(time, sat, &raw->obuf.data[n]);
  return n;
}

/* packet 20/21: cache epoch time --------------------------------------------*/
static int decode_systemstate(raw_t* raw) {
  if (raw->len < HDR_LEN + SYSSTATE_LEN) {
    trace(2, "decode_systemstate: short packet len=%d\n", raw->len);
    return 0;
  }
  anpp_t* anpp = raw->rcv_data;
  anpp->time.time = (time_t)U4(raw, HDR_LEN + 4);
  anpp->time.sec = U4(raw, HDR_LEN + 8) * 1e-6;
  return 0;
}

static int decode_unixtime(raw_t* raw) {
  if (raw->len < HDR_LEN + UNIXTIME_LEN) {
    trace(2, "decode_unixtime: short packet len=%d\n", raw->len);
    return 0;
  }
  anpp_t* anpp = (anpp_t*)raw->rcv_data;
  anpp->time.time = (time_t)U4(raw, HDR_LEN);
  anpp->time.sec = U4(raw, HDR_LEN + 4) * 1e-6;
  return 0;
}

/* packet 60: raw satellite data (observations) ------------------------------*/
static int decode_rawsatdata(raw_t* raw) {
  if (raw->len < HDR_LEN + OBS_HDR_LEN) {
    trace(2, "decode_rawsatdata: short packet len=%d\n", raw->len);
    return 0;
  }

  uint32_t unix_time = U4(raw, HDR_LEN);
  uint32_t nanoseconds = U4(raw, HDR_LEN + 4);
  uint8_t receiver_number = U1(raw, HDR_LEN + 12);
  uint8_t packet_number = U1(raw, HDR_LEN + 13);
  uint8_t total_packets = U1(raw, HDR_LEN + 14);
  uint8_t nsats = U1(raw, HDR_LEN + 15);

  /* select which receiver_number to emit: parse -RCVR<n> from raw->opt
     (default 0) */
  const char* q;
  int rcvr_sel = 0;
  if ((q = strstr(raw->opt, "-RCVR")) != NULL) sscanf(q + 5, "%d", &rcvr_sel);

  /* skip fragments from antennae we are not interested in */
  if (receiver_number != (uint8_t)rcvr_sel) return 0;

  /* if a new epoch begins before the previous one finished, flush partial */
  int ret = 0;
  anpp_t* anpp = (anpp_t*)raw->rcv_data;
  if (anpp->epoch_active &&
      (unix_time != anpp->cur_unix_time || nanoseconds != anpp->cur_nanoseconds)) {
    ret = flush_obuf_anpp(raw);
    anpp->epoch_active = 0;
  }

  /* start a fresh epoch if no assembly is in progress */
  if (!anpp->epoch_active) {
    gtime_t time;
    time.time = (time_t)unix_time;
    time.sec = nanoseconds * 1e-9;
    raw->time = utc2gpst(time);
    raw->obuf.n = 0;
    anpp->cur_unix_time = unix_time;
    anpp->cur_nanoseconds = nanoseconds;
    anpp->epoch_active = 1;
  }

  size_t offset = HDR_LEN + OBS_HDR_LEN;

  for (int i = 0; i < nsats && offset + SAT_HDR_LEN <= (size_t)raw->len; i++) {
    uint8_t sys_id = U1(raw, offset);
    int sys = anpp2sys(sys_id);
    int sat = satno(sys, U1(raw, offset + 1));
    uint8_t nfreqs = U1(raw, offset + 5);
    offset += SAT_HDR_LEN;

    if (sat == 0) {
      offset += nfreqs * FREQ_BLK_LEN;
      continue;
    }
    int n = obuf_slot(raw, raw->time, sat);
    if (n < 0) {
      offset += nfreqs * FREQ_BLK_LEN;
      continue;
    }

    for (; nfreqs > 0 && offset + FREQ_BLK_LEN <= (size_t)raw->len;
         nfreqs--, offset += FREQ_BLK_LEN) {
      uint8_t freq_id = U1(raw, offset);
      uint8_t trks = U1(raw, offset + 1);
      uint8_t code = anpp2code(sys_id, freq_id);
      if (code == CODE_NONE) continue;
      int idx = code2idx(sys, code);
      if (idx < 0 || idx >= NFREQ + NEXOBS) continue;

      if (trks & TRKS_CARRIER) {
        raw->obuf.data[n].L[idx] = R8(raw, offset + 2);
        raw->obuf.data[n].LLI[idx] = (trks & TRKS_SLIP) ? LLI_SLIP : 0;
        raw->obuf.data[n].LLI[idx] |= (trks & TRKS_HALFCYCLE) ? LLI_HALFC : 0;
      }
      if (trks & TRKS_PSEUDORANGE) raw->obuf.data[n].P[idx] = R8(raw, offset + 10);
      if (trks & TRKS_DOPPLER) raw->obuf.data[n].D[idx] = R4(raw, offset + 18);
      if (trks & TRKS_SNR) raw->obuf.data[n].SNR[idx] = R4(raw, offset + 22);

      raw->obuf.data[n].code[idx] = code;
    }
  }

  /* flush when the final fragment of this epoch arrives */
  if (packet_number == total_packets) {
    ret = flush_obuf_anpp(raw);
    anpp->epoch_active = 0;
  }
  return ret;
}

/* packet 61: raw satellite ephemeris ----------------------------------------*/
static int decode_gps_eph(raw_t* raw, int sat, size_t e) {
  eph_t eph = {0};
  eph.sat = sat;
  eph.iodc = (int)U2(raw, e + 4);
  eph.iode = (int)U2(raw, e + 6);
  eph.week = (int)U2(raw, e + 116);
  eph.toes = (double)U4(raw, e + 0);
  eph.toe = gpst2time(eph.week, eph.toes);
  eph.toc = eph.toe;
  eph.ttr = gpst2time(eph.week, (double)U4(raw, e + 118));
  eph.f0 = (double)R4(raw, e + 8);
  eph.f1 = (double)R4(raw, e + 12);
  eph.f2 = (double)R4(raw, e + 16);
  eph.crs = (double)R4(raw, e + 20);
  eph.deln = (double)R4(raw, e + 24);
  eph.M0 = R8(raw, e + 28);
  eph.cuc = (double)R4(raw, e + 36);
  eph.e = R8(raw, e + 40);
  eph.cus = (double)R4(raw, e + 48);
  double sqrtA = R8(raw, e + 52);
  eph.A = sqrtA * sqrtA;
  eph.cic = (double)R4(raw, e + 60);
  eph.OMG0 = R8(raw, e + 64);
  eph.cis = (double)R4(raw, e + 72);
  eph.i0 = R8(raw, e + 76);
  eph.crc = (double)R4(raw, e + 84);
  eph.omg = R8(raw, e + 88);
  eph.OMGd = R8(raw, e + 96);
  eph.idot = R8(raw, e + 104);
  eph.tgd[0] = (double)R4(raw, e + 112);
  eph.sva = 0;
  eph.svh = 0;
  eph.code = 0;
  eph.flag = 0;
  eph.fit = 4.0;

  if (eph.iode == raw->nav.eph[sat - 1].iode && eph.iodc == raw->nav.eph[sat - 1].iodc &&
      fabs(timediff(eph.toe, raw->nav.eph[sat - 1].toe)) < 1e-9)
    return 0;

  raw->nav.eph[sat - 1] = eph;
  raw->ephsat = sat;
  raw->ephset = 0;
  return 2;
}

static int decode_glo_eph(raw_t* raw, int prn, size_t e) {
  geph_t geph = {0};
  geph.sat = satno(SYS_GLO, prn);
  geph.frq = (int)I1(raw, e + 85);
  geph.svh = (int)U1(raw, e + 86);
  geph.age = (int)U1(raw, e + 84);
  geph.taun = (double)R4(raw, e + 0);
  geph.gamn = (double)R4(raw, e + 4);
  geph.pos[0] = R8(raw, e + 8);
  geph.pos[1] = R8(raw, e + 16);
  geph.pos[2] = R8(raw, e + 24);
  geph.vel[0] = R8(raw, e + 32);
  geph.vel[1] = R8(raw, e + 40);
  geph.vel[2] = R8(raw, e + 48);
  geph.acc[0] = R8(raw, e + 56);
  geph.acc[1] = R8(raw, e + 64);
  geph.acc[2] = R8(raw, e + 72);
  gtime_t utc;
  utc.time = (time_t)U4(raw, e + 80);
  utc.sec = 0.0;
  geph.toe = utc2gpst(utc);
  geph.tof = geph.toe;

  if (geph.svh == raw->nav.geph[prn - 1].svh &&
      fabs(timediff(geph.toe, raw->nav.geph[prn - 1].toe)) < 1e-9)
    return 0;

  raw->nav.geph[prn - 1] = geph;
  raw->ephsat = geph.sat;
  raw->ephset = 0;
  return 2;
}

static int decode_rawsateph(raw_t* raw) {
  uint8_t sys_id = U1(raw, HDR_LEN + 4);
  int sys = anpp2sys(sys_id);
  int prn = (int)U1(raw, HDR_LEN + 5);
  int sat = satno(sys, prn);
  if (sat == 0) return 0;

  switch (sys) {
    case SYS_GPS:
      if (raw->len < HDR_LEN + EPH_LEN_GPS) {
        trace(2, "decode_rawsateph GPS: short packet len=%d\n", raw->len);
        return 0;
      }
      return decode_gps_eph(raw, sat, HDR_LEN + 6);
    case SYS_GLO:
      if (raw->len < HDR_LEN + EPH_LEN_GLO) {
        trace(2, "decode_rawsateph GLO: short packet len=%d\n", raw->len);
        return 0;
      }
      return decode_glo_eph(raw, prn, HDR_LEN + 6);
    default:
      trace(2, "decode_rawsateph: unsupported sys=%d\n", sys_id);
      return 0;
  }
}

/* sync, dispatch, public API ------------------------------------------------*/
static uint16_t crc_ccitt(const uint8_t* buff, int len) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < len; i++) {
    crc ^= (uint16_t)buff[i] << 8;
    for (int j = 0; j < 8; j++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
  }
  return crc;
}

static int valid_hdr(const raw_t* raw) {
  return ((U1(raw, 0) + U1(raw, 1) + U1(raw, 2) + U1(raw, 3) + U1(raw, 4)) & 0xFF) == 0;
}

static int decode_anpp(raw_t* raw) {
  int id = (int)U1(raw, 1);

  trace(4, "decode_anpp: id=%d len=%d\n", id, raw->len);

  switch (id) {
    case ID_SYSTEMSTATE:
      return decode_systemstate(raw);
    case ID_UNIXTIME:
      return decode_unixtime(raw);
    case ID_RAWSATDATA:
      return decode_rawsatdata(raw);
    case ID_RAWSATEPH:
      return decode_rawsateph(raw);
  }
  return 0;
}

extern int input_anpp(raw_t* raw, uint8_t data) {
  RTKBOUNDSCHECK(raw->buff, sizeof(raw->buff), raw->nbyte);
  raw->buff[raw->nbyte] = data;
  raw->nbyte++;

  if (raw->nbyte < HDR_LEN) return 0;

  if (raw->nbyte == HDR_LEN) {
    raw->len = HDR_LEN;
    if (!valid_hdr(raw)) {
      memmove(raw->buff, raw->buff + 1, HDR_LEN - 1);
      raw->nbyte = raw->len = HDR_LEN - 1;
      return 0;
    }
    raw->len = HDR_LEN + (int)U1(raw, 2);
    if (raw->len > MAXRAWLEN) {
      trace(2, "adnav length error: len=%d\n", raw->len);
      raw->nbyte = raw->len = 0;
      return -1;
    }
  }

  if (raw->nbyte < raw->len) return 0;
  raw->nbyte = 0;

  if (crc_ccitt(raw->buff + HDR_LEN, raw->len - HDR_LEN) != U2(raw, 3)) {
    trace(2, "input_anpp: CRC error id=%d\n", (int)raw->buff[1]);
    raw->nbyte = raw->len = 0;
    return -1;
  }
  return decode_anpp(raw);
}

extern int input_anppf(raw_t* raw, FILE* fp) {
  trace(4, "input_anppf:\n");

  for (int i = 0; i < 4096; i++) {
    int data = fgetc(fp);
    if (data == EOF) return -2;
    int ret = input_anpp(raw, (uint8_t)data);
    if (ret != 0) return ret;
  }
  return 0;
}
