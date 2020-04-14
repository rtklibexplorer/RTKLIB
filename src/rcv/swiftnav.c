/*------------------------------------------------------------------------------
 * switnav.c : Swift Navigation Binary Protocol decoder
 *
 *          Copyright (C) 2017
 *
 * reference :
 *     [1]
 *
 * version : $Revision: 1.0 $ $Date: 2017/01/30 09:00:00 $
 *
 * history : 2017/01/30  1.0  begin writing
  *-----------------------------------------------------------------------------*/
#include "rtklib.h"

#include <math.h>
#include <stdint.h>

#define SBP_SYNC1 0x55 /* SBP message header sync */

#define ID_MEASEPOCH 0x004A      /* observation */
#define ID_MSGEPHGPS_DEP1 0x0081 /* GPS L1 C/A nav message (deprecated) */
#define ID_MSGEPHGPS_DEP2 0x0086 /* GPS L1 C/A nav message (deprecated) */
#define ID_MSGEPHGPS 0x008A      /* GPS L1 C/A nav message */
#define ID_MSGEPHBDS 0x0089      /* BDS B1/B2 D1 nav message */
#define ID_MSGEPHGAL 0x0095      /* GAL E1 I/NAV message */
#define ID_MSGEPHGLO_DEP1 0x0088 /* Glonass L1/L2 OF nav message (deprecated)  \
                                    */
#define ID_MSGEPHGLO 0x008B      /* Glonass L1/L2 OF nav message */
#define ID_MSGIONGPS 0x0090      /* GPS ionospheric parameters */
#define ID_MSG_SBAS_RAW 0x7777   /* SBAS data */
#define ID_MSG_EXT_EVENT 0x101   /* external event */

#define SEC_DAY 86400.0

/** Constant difference of Beidou time from GPS time */
#define BDS_WEEK_TO_GPS_WEEK 1356
#define BDS_SECOND_TO_GPS_SECOND 14

/* get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((uint8_t *)(p)))
#define I1(p) (*((int8_t *)(p)))
static uint16_t U2(uint8_t *p) {
  uint16_t u;
  memcpy(&u, p, 2);
  return u;
}
static uint32_t U4(uint8_t *p) {
  uint32_t u;
  memcpy(&u, p, 4);
  return u;
}
static float R4(uint8_t *p) {
  float r;
  memcpy(&r, p, 4);
  return r;
}
static double R8(uint8_t *p) {
  double r;
  memcpy(&r, p, 8);
  return r;
}
static int32_t I4(uint8_t *p) {
  int32_t u;
  memcpy(&u, p, 4);
  return u;
}
static int16_t I2(uint8_t *p) {
  int16_t i;
  memcpy(&i, p, 2);
  return i;
}

/** Code identifier. */
typedef enum code_e {
  CODE_INVALID = -1,
  CODE_GPS_L1CA = 0,
  CODE_GPS_L2CM = 1,
  CODE_SBAS_L1CA = 2,
  CODE_GLO_L1OF = 3,
  CODE_GLO_L2OF = 4,
  CODE_GPS_L1P = 5,
  CODE_GPS_L2P = 6,
  CODE_GPS_L2CL = 7,
  CODE_GPS_L2CX = 8, /* combined L2C tracking */
  CODE_GPS_L5I = 9,
  CODE_GPS_L5Q = 10,
  CODE_GPS_L5X = 11,  /* combined L5 tracking */
  CODE_BDS2_B1 = 12, /* data channel at 1526 * 1.023 MHz */
  CODE_BDS2_B2 = 13,  /* data channel at 1180 * 1.023 MHz */
  CODE_GAL_E1B = 14,  /* data channel at E1 (1540 * 1.023 MHz) */
  CODE_GAL_E1C = 15,  /* pilot channel at E1 */
  CODE_GAL_E1X = 16,  /* combined tracking on E1 */
  CODE_GAL_E6B = 17,
  CODE_GAL_E6C = 18,
  CODE_GAL_E6X = 19, /* combined tracking on E6 */
  CODE_GAL_E7I = 20,
  CODE_GAL_E7Q = 21,
  CODE_GAL_E7X = 22, /* combined tracking on E5b */
  CODE_GAL_E8 = 23,  /* E5 AltBOC tracking */
  CODE_GAL_E5I = 24,
  CODE_GAL_E5Q = 25,
  CODE_GAL_E5X = 26, /* combined tracking on E5a */
  CODE_QZS_L1CA = 27,
  CODE_QZS_L2CM = 28,
  CODE_QZS_L2CL = 29,
  CODE_QZS_L2CX = 30,
  CODE_QZS_L5I = 31,
  CODE_QZS_L5Q = 32,
  CODE_QZS_L5X = 33,
  CODE_COUNT
} code_t;

typedef struct {
  uint32_t code;
  uint32_t sys;
  uint32_t freq;
} bandcode_t;

static bandcode_t rtklib_bandcode_map[CODE_COUNT] =
    {{CODE_L1C, SYS_GPS, 0},     /* [CODE_GPS_L1CA] */
     {CODE_L2S, SYS_GPS, 1},     /* [CODE_GPS_L2CM] */
     {CODE_L1C, SYS_SBS, 0},     /* [CODE_SBAS_L1CA]*/
     {CODE_L1C, SYS_GLO, 0},     /* [CODE_GLO_L1OF] */
     {CODE_L2C, SYS_GLO, 1},     /* [CODE_GLO_L2OF] */
     {CODE_L1P, SYS_GPS, 0},     /* [CODE_GPS_L1P]  */
     {CODE_L2P, SYS_GPS, 1},     /* [CODE_GPS_L2P]  */
     {CODE_L2L, SYS_GPS, 1},     /* [CODE_GPS_L2CL] */
     {CODE_L2X, SYS_GPS, 1},     /* [CODE_GPS_L2CX] */
     {CODE_L5I, SYS_GPS, 3},     /* [CODE_GPS_L5I]  */
     {CODE_L5Q, SYS_GPS, 3},     /* [CODE_GPS_L5Q]  */
     {CODE_L5X, SYS_GPS, 3},     /* [CODE_GPS_L5X]  */
     {CODE_L1I, SYS_CMP, 0},     /* [CODE_BDS2_B1]  */
     {CODE_L7I, SYS_CMP, 1},     /* [CODE_BDS2_B2]  */
     {CODE_L1B, SYS_GAL, 0},     /* [CODE_GAL_E1B]  */
     {CODE_L1C, SYS_GAL, 0},     /* [CODE_GAL_E1C]  */
     {CODE_L1X, SYS_GAL, 0},     /* [CODE_GAL_E1X]  */
     {CODE_L6B, SYS_GAL, 4},     /* [CODE_GAL_E6B]  */
     {CODE_L6C, SYS_GAL, 4},     /* [CODE_GAL_E6C]  */
     {CODE_L6X, SYS_GAL, 4},     /* [CODE_GAL_E6X]  */
     {CODE_L7I, SYS_GAL, 2},     /* [CODE_GAL_E7I]  */
     {CODE_L7Q, SYS_GAL, 2},     /* [CODE_GAL_E7Q]  */
     {CODE_L7X, SYS_GAL, 2},     /* [CODE_GAL_E7X]  */
     {CODE_L8X, SYS_GAL, 3},     /* [CODE_GAL_E8]   */
     {CODE_L5I, SYS_GAL, 3},     /* [CODE_GAL_E5I]  */
     {CODE_L5Q, SYS_GAL, 3},     /* [CODE_GAL_E5Q]  */
     {CODE_L5X, SYS_GAL, 3},     /* [CODE_GAL_E5X]  */
     {CODE_L1C, SYS_QZS, 0},     /* [CODE_QZS_L1CA] */
     {CODE_L2S, SYS_QZS, 1},     /* [CODE_QZS_L2CM] */
     {CODE_L2L, SYS_QZS, 1},     /* [CODE_QZS_L2CL] */
     {CODE_L2X, SYS_QZS, 1},     /* [CODE_QZS_L2CX] */
     {CODE_L5I, SYS_QZS, 3},     /* [CODE_QZS_L5I]  */
     {CODE_L5Q, SYS_QZS, 3},     /* [CODE_QZS_L5Q]  */
     {CODE_L5X, SYS_QZS, 3}};    /* [CODE_QZS_L5X]  */

/* checksum lookup table -----------------------------------------------------*/
static const uint32_t CRC_16CCIT_LookUp[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108,
    0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210,
    0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b,
    0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401,
    0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee,
    0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6,
    0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d,
    0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 0x5af5,
    0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc,
    0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 0x6ca6, 0x7c87, 0x4ce4,
    0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd,
    0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13,
    0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a,
    0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e,
    0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1,
    0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb,
    0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 0x34e2, 0x24c3, 0x14a0,
    0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8,
    0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657,
    0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9,
    0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882,
    0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 0xfd2e,
    0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07,
    0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 0xef1f, 0xff3e, 0xcf5d,
    0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74,
    0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};

/* it's easy to derive a function for the values below, but I'd rather map the
 * table explicitly from the RTCM standard document */
static const uint32_t rtcm_phase_lock_table[16] = {
    0,    32,   64,    128,   256,   512,    1024,   2048,
    4096, 8192, 16384, 32768, 65536, 131072, 262144, 524288};

static const uint8_t decoding_table[256] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x00, 0x00, 0x00, 0x3F,
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
    0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12,
    0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24,
    0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30,
    0x31, 0x32, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00};

static uint8_t puPayloadTmp[256];

static const gtime_t time0 = {0};
static int Base64_Decode(uint8_t *_pcData, uint32_t _uDataLen,
                         uint8_t *_puDecodedData, uint32_t *_puDecodedDataLen) {
  uint32_t i, j;
  uint32_t output_length;
  uint32_t a, b, c, d, t;

  if (NULL == _puDecodedData) {
    return -1;
  }

  if (0 != (_uDataLen % 4)) {
    return -1;
  }

  output_length = _uDataLen / 4 * 3;

  if ('=' == _pcData[_uDataLen - 1]) {
    output_length--;
  }
  if ('=' == _pcData[_uDataLen - 2]) {
    output_length--;
  }

  if (output_length > (*_puDecodedDataLen)) {
    /* Not enough space in output buffer */
    return -1;
  }

  (*_puDecodedDataLen) = output_length;

  for (i = 0, j = 0; i < _uDataLen;) {
    a = ('=' == _pcData[i]) ? 0 : decoding_table[_pcData[i]];
    i++;
    b = ('=' == _pcData[i]) ? 0 : decoding_table[_pcData[i]];
    i++;
    c = ('=' == _pcData[i]) ? 0 : decoding_table[_pcData[i]];
    i++;
    d = ('=' == _pcData[i]) ? 0 : decoding_table[_pcData[i]];
    i++;

    t = (a << 3 * 6) + (b << 2 * 6) + (c << 1 * 6) + (d << 0 * 6);

    if (j < output_length) {
      _puDecodedData[j++] = (t >> 2 * 8) & 0xFF;
    }
    if (j < output_length) {
      _puDecodedData[j++] = (t >> 1 * 8) & 0xFF;
    }
    if (j < output_length) {
      _puDecodedData[j++] = (t >> 0 * 8) & 0xFF;
    }
  } /* for() */
  return 0;
} /* Base64_Decode() */

/* SBP checksum calculation --------------------------------------------------*/
static uint16_t sbp_checksum(uint8_t *buff, int len) {
  int i;
  uint16_t crc = 0;
  for (i = 0; i < len; i++) {
    crc = (crc << 8) ^ CRC_16CCIT_LookUp[(crc >> 8) ^ buff[i]];
  }
  return crc;
}

/* flush observation data buffer ---------------------------------------------*/
static int flushobuf(raw_t *raw) {
  int i, j, n = 0;

  trace(3, "flushobuf: n=%d\n", raw->obuf.n);

  /* copy events from data buffer */
  if (raw->obuf.data[0].eventime.time>0) {
     raw->obs.data[0].eventime = raw->obuf.data[0].eventime;
     raw->obs.data[0].timevalid = raw->obuf.data[0].timevalid;
     raw->obs.flag = raw->obuf.flag;
     raw->obs.rcvcount = raw->obuf.rcvcount;
     raw->obs.tmcount=raw->obuf.tmcount;
    }

  /* copy observation data buffer */
  for (i = 0; i < raw->obuf.n && i < MAXOBS; i++) {
    if (!satsys(raw->obuf.data[i].sat, NULL))
      continue;
    if (raw->obuf.data[i].time.time == 0)
      continue;
    raw->obs.data[n++] = raw->obuf.data[i];
  }
  raw->obs.n = n;

  /* clear events from data buffer */
  raw->obuf.data[0].eventime = time0;
  raw->obuf.flag = raw->obuf.data[0].timevalid=0;
  raw->obuf.rcvcount = raw->obuf.tmcount=0;

  /* clear observation data buffer */
  for (i = 0; i < MAXOBS; i++) {
    raw->obuf.data[i].time = time0;
    for (j = 0; j < NFREQ + NEXOBS; j++) {
      raw->obuf.data[i].L[j] = raw->obuf.data[i].P[j] = 0.0;
      raw->obuf.data[i].D[j] = 0.0;
      raw->obuf.data[i].SNR[j] = raw->obuf.data[i].LLI[j] = 0;
      raw->obuf.data[i].code[j] = CODE_NONE;
    }
  }
  for (i = 0; i < MAXSAT; i++) {
    raw->prCA[i] = raw->dpCA[i] = 0.0;
  }
  return n > 0 ? 1 : 0;
}
/* clear buffer --------------------------------------------------------------*/
static void clearbuff(raw_t *raw) {
  raw->buff[0] = 0;
  raw->len = raw->nbyte = 0;
}

/* appropriate calculation of LLI for SBP */
static uint8_t calculate_loss_of_lock(double dt, uint32_t prev_lock_time,
                                      uint32_t curr_lock_time) {
  if (prev_lock_time > curr_lock_time) {
    /*    fprintf(stderr, "prev_lock_time %d curr_lock_time %d\n",
     * prev_lock_time, curr_lock_time);*/
    return 1;
  } else if ((prev_lock_time == curr_lock_time) && (dt >= prev_lock_time)) {
    /*fprintf(stderr, "2\n");*/
    return 1;
  } else if ((prev_lock_time == curr_lock_time) && (dt < prev_lock_time)) {
    return 0;
  } else if ((prev_lock_time < curr_lock_time) &&
             (dt >= (2 * curr_lock_time - prev_lock_time))) {
    /*fprintf(stderr, "3\n");*/
    return 1;
  } else if ((prev_lock_time < curr_lock_time) &&
             (curr_lock_time < dt &&
              dt < (2 * curr_lock_time - prev_lock_time))) {
    /*fprintf(stderr, "4\n");*/
    return 1;
  } else if ((prev_lock_time < curr_lock_time) && (dt <= curr_lock_time))
    return 0;
  else {
    /*fprintf(stderr, "5\n");*/
    return 1;
  }
}

/* decode SBF measurements message (observables) -----------------------------*/
static int decode_msgobs(raw_t *raw) {
  gtime_t time;
  double tow, dResTow, pseudorange, carr_phase, freq_doppler, delta_time;
  int16_t i, ii, sat, n, week, band_code;
  uint8_t *p = (raw->buff) + 6; /* jump to TOW location */
  uint8_t num_obs, lock_info;
  uint32_t prev_lockt = 0, curr_lockt = 0;
  uint8_t flags, sat_id, cn0_int, slip, half_cycle_amb;
  uint32_t code = 0, sys = 0, freq = 0;
  int iDidFlush = 0, iSatFound = 0;

  trace(4, "SBF decode_msgobs: len=%d\n", raw->len);

  /* Get time information */
  tow = U4(p);         /* TOW in ms */
  dResTow = I4(p + 4); /* residual Time Of Week */
  week = U2(p + 8);    /* GPS week */
  num_obs = p[10];       /* number of observations in message */
  /*  uSeqSize = num_obs>>4; */
  /*  uSeqIdx  = num_obs&0xf; */
  num_obs = ((raw->len) - 19) / 17;

  time = gpst2time(week, tow * 0.001 + dResTow * 1e-9);
  /* start new observation period */
  delta_time = fabs(timediff(time, raw->time));
  if ((delta_time) > 1E-6) {
    n = 0;
    iDidFlush = flushobuf(raw);
  } else {
    n = raw->obuf.n;
    iDidFlush = 0;
  }

  /* set the pointer from TOW to the beginning of sub-block */
  p = p + 11;

  /* add observations */
  for (i = 0; i < num_obs && i < MAXOBS; i++, p += 17) {

    pseudorange = U4(p) * 0.02;  /* pseudorange observation in 2cm units */
    carr_phase = I4(p + 4);     /* carrier phase integer cycles */
    carr_phase += p[8] / 256.0; /* carrier phase fractional cycles */
    freq_doppler = I2(p + 9);       /* Doppler shift in integer Hz */
    freq_doppler += p[11] / 256.0;  /* fractional part of Doppler shift */
    cn0_int = p[12];               /* C/N0 */
    lock_info = p[13] & 0xf;    /* lock time */
    flags = p[14];             /* observation flags */
    sat_id = p[15];
    band_code = p[16];

    /* Check for RAIM exclusion */
    if ((flags & 0x80) && (NULL == strstr(raw->opt, "OBSALL"))) {
      continue;
    }

    /* phase polarity flip option (INVCP) */
    if (strstr(raw->opt, "INVCP")) {
      carr_phase = -carr_phase;
    }

    if ((CODE_INVALID != band_code) && (band_code < CODE_COUNT)) {
      code = rtklib_bandcode_map[band_code].code;
      sys = rtklib_bandcode_map[band_code].sys;
      freq = rtklib_bandcode_map[band_code].freq;
    }

    /* store satellite number */
    sat = satno(sys, sat_id);
    if (sat == 0) {
      continue;
    }

    iSatFound = 0;
    for (ii = 0; ii < n; ii++) {
      if (raw->obuf.data[ii].sat == sat) {
        iSatFound = 1;
        break;
      }
    }

    raw->obuf.data[ii].time = time;
    raw->obuf.data[ii].sat = (unsigned char)sat;

    /* store signal info */
    if (freq < NFREQ + NEXOBS) {
      raw->obuf.data[ii].P[freq] = (flags & 0x1) ? pseudorange : 0.0;
      raw->obuf.data[ii].L[freq] = (flags & 0x2) ? carr_phase : 0.0;
      raw->obuf.data[ii].D[freq] = (flags & 0x8) ? (float)freq_doppler : 0.0f;
      raw->obuf.data[ii].SNR[freq] = cn0_int;
      raw->obuf.data[ii].code[freq] = code;

      if (flags & 0x2) {
        prev_lockt = rtcm_phase_lock_table[(raw->halfc[sat - 1][freq])];
        curr_lockt = rtcm_phase_lock_table[lock_info];
        slip =
            calculate_loss_of_lock(delta_time * 1000.0, prev_lockt, curr_lockt);
        half_cycle_amb = (flags & 0x4) ? 0 : 1;
        if (half_cycle_amb) {
          slip |= 0x2; /* half-cycle ambiguity unresolved */
        }

        raw->obuf.data[ii].LLI[freq] |= slip;
        /* using the field below just to store previous lock info */
        raw->halfc[sat - 1][freq] = lock_info;
      }
    }

    /* Receiver channel goes up */
    if (!iSatFound)
      n++;
  }
  raw->time = time;
  raw->obuf.n = n;
  return iDidFlush;
}

/* common part of GPS eph decoding (navigation data)
 * --------------------------*/
static void decode_gpsnav_common_dep1(uint8_t *_pBuff, eph_t *_pEph) {
  uint16_t uWeekE, uWeekC;
  double dToc;

  _pEph->toes = U4(_pBuff + 4);
  uWeekE = U2(_pBuff + 8);
  _pEph->sva = uraindex(R8(_pBuff + 10)); /* URA index */
  _pEph->fit = U4(_pBuff + 18) / 3600;
  /* _pEph->flag = U1(_pBuff + 22); SBP payload does not have L2 flag */
  _pEph->svh = U1(_pBuff + 23);

  _pEph->tgd[0] = R8(_pBuff + 24);
  _pEph->crs = R8(_pBuff + 32);
  _pEph->crc = R8(_pBuff + 40);
  _pEph->cuc = R8(_pBuff + 48);
  _pEph->cus = R8(_pBuff + 56);
  _pEph->cic = R8(_pBuff + 64);
  _pEph->cis = R8(_pBuff + 72);

  _pEph->deln = R8(_pBuff + 80);
  _pEph->M0 = R8(_pBuff + 88);
  _pEph->e = R8(_pBuff + 96);
  _pEph->A = pow(R8(_pBuff + 104), 2);
  _pEph->OMG0 = R8(_pBuff + 112);
  _pEph->OMGd = R8(_pBuff + 120);
  _pEph->omg = R8(_pBuff + 128);
  _pEph->i0 = R8(_pBuff + 136);
  _pEph->idot = R8(_pBuff + 144);

  _pEph->f0 = R8(_pBuff + 152);
  _pEph->f1 = R8(_pBuff + 160);
  _pEph->f2 = R8(_pBuff + 168);

  dToc = U4(_pBuff + 176);
  uWeekC = U2(_pBuff + 180); /* WN */
  _pEph->iode = U1(_pBuff + 182);
  _pEph->iodc = U2(_pBuff + 183);

  _pEph->week = adjgpsweek(uWeekE);
  _pEph->toe = gpst2time(_pEph->week, _pEph->toes);
  _pEph->toc = gpst2time(uWeekC, dToc);
}

/* common part of GPS eph decoding (navigation data)
 * --------------------------*/
static void decode_gpsnav_common(uint8_t *_pBuff, eph_t *_pEph) {
  uint16_t uWeekE, uWeekC;
  double dToc;

  _pEph->toes = U4(_pBuff + 4);
  uWeekE = U2(_pBuff + 8);
  _pEph->sva = uraindex(R4(_pBuff + 10)); /* URA index */
  _pEph->fit = U4(_pBuff + 14) / 3600;
  /* _pEph->flag = U1(_pBuff + 18); SBP payload does not have L2 flag */
  _pEph->svh = U1(_pBuff + 19);

  _pEph->tgd[0] = R4(_pBuff + 20);
  _pEph->crs = R4(_pBuff + 24);
  _pEph->crc = R4(_pBuff + 28);
  _pEph->cuc = R4(_pBuff + 32);
  _pEph->cus = R4(_pBuff + 36);
  _pEph->cic = R4(_pBuff + 40);
  _pEph->cis = R4(_pBuff + 44);

  _pEph->deln = R8(_pBuff + 48);
  _pEph->M0 = R8(_pBuff + 56);
  _pEph->e = R8(_pBuff + 64);
  _pEph->A = pow(R8(_pBuff + 72), 2);
  _pEph->OMG0 = R8(_pBuff + 80);
  _pEph->OMGd = R8(_pBuff + 88);
  _pEph->omg = R8(_pBuff + 96);
  _pEph->i0 = R8(_pBuff + 104);
  _pEph->idot = R8(_pBuff + 112);

  _pEph->f0 = R4(_pBuff + 120);
  _pEph->f1 = R4(_pBuff + 124);
  _pEph->f2 = R4(_pBuff + 128);

  dToc = U4(_pBuff + 132);
  uWeekC = U2(_pBuff + 136); /* WN */
  _pEph->iode = U1(_pBuff + 138);
  _pEph->iodc = U2(_pBuff + 139);

  _pEph->week = adjgpsweek(uWeekE);
  _pEph->toe = gpst2time(_pEph->week, _pEph->toes);
  _pEph->toc = gpst2time(uWeekC, dToc);
}

/* common part of BDS eph decoding (navigation data)
 * --------------------------*/
static void decode_bdsnav_common(uint8_t *_pBuff, eph_t *_pEph) {
  uint16_t uWeekE, uWeekC;
  double dToc;

  _pEph->toes = U4(_pBuff + 4) - BDS_SECOND_TO_GPS_SECOND;
  uWeekE = U2(_pBuff + 8);
  _pEph->sva = uraindex(R4(_pBuff + 10)); /* URA index */
  _pEph->fit = U4(_pBuff + 14) ? 0 : 4;
  _pEph->flag = U1(_pBuff + 18);

  _pEph->tgd[0] = R4(_pBuff + 20);
  _pEph->tgd[1] = R4(_pBuff + 24);
  _pEph->crs = R4(_pBuff + 28);
  _pEph->crc = R4(_pBuff + 32);
  _pEph->cuc = R4(_pBuff + 36);
  _pEph->cus = R4(_pBuff + 40);
  _pEph->cic = R4(_pBuff + 44);
  _pEph->cis = R4(_pBuff + 48);

  _pEph->deln = R8(_pBuff + 52);
  _pEph->M0 = R8(_pBuff + 60);
  _pEph->e = R8(_pBuff + 68);
  _pEph->A = pow(R8(_pBuff + 76), 2);
  _pEph->OMG0 = R8(_pBuff + 84);
  _pEph->OMGd = R8(_pBuff + 92);
  _pEph->omg = R8(_pBuff + 100);
  _pEph->i0 = R8(_pBuff + 108);
  _pEph->idot = R8(_pBuff + 116);

  _pEph->f0 = R8(_pBuff + 124);
  _pEph->f1 = R4(_pBuff + 132);
  _pEph->f2 = R4(_pBuff + 136);

  dToc = U4(_pBuff + 140);
  uWeekC = U2(_pBuff + 144); /* WN */
  _pEph->iode = U1(_pBuff + 146);
  _pEph->iodc = U2(_pBuff + 147);

  _pEph->week = adjgpsweek(uWeekE) - BDS_WEEK_TO_GPS_WEEK;
  _pEph->toe = gpst2time(_pEph->week, _pEph->toes);
  _pEph->toc = gpst2time(uWeekC, dToc);
}

/* common part of GAL eph decoding (navigation data)
 * --------------------------*/
static void decode_galnav_common(uint8_t *_pBuff, eph_t *_pEph) {
  uint16_t uWeekE, uWeekC;
  double dToc;

  _pEph->toes = U4(_pBuff + 4);
  uWeekE = U2(_pBuff + 8);
  _pEph->sva = sisa_index(R4(_pBuff + 10)); /* URA index */
  _pEph->fit = U4(_pBuff + 14) ? 0 : 4;
  _pEph->flag = U1(_pBuff + 18);

  _pEph->tgd[0] = R4(_pBuff + 20);
  _pEph->tgd[1] = R4(_pBuff + 24);
  _pEph->crs = R4(_pBuff + 28);
  _pEph->crc = R4(_pBuff + 32);
  _pEph->cuc = R4(_pBuff + 36);
  _pEph->cus = R4(_pBuff + 40);
  _pEph->cic = R4(_pBuff + 44);
  _pEph->cis = R4(_pBuff + 48);

  _pEph->deln = R8(_pBuff + 52);
  _pEph->M0 = R8(_pBuff + 60);
  _pEph->e = R8(_pBuff + 68);
  _pEph->A = pow(R8(_pBuff + 76), 2);
  _pEph->OMG0 = R8(_pBuff + 84);
  _pEph->OMGd = R8(_pBuff + 92);
  _pEph->omg = R8(_pBuff + 100);
  _pEph->i0 = R8(_pBuff + 108);
  _pEph->idot = R8(_pBuff + 116);

  _pEph->f0 = R8(_pBuff + 124);
  _pEph->f1 = R8(_pBuff + 132);
  _pEph->f2 = R4(_pBuff + 140);

  dToc = U4(_pBuff + 144);
  uWeekC = U2(_pBuff + 148); /* WN */
  _pEph->iode = U2(_pBuff + 150);
  _pEph->iodc = U2(_pBuff + 152);

  _pEph->week = adjgpsweek(uWeekE);
  _pEph->toe = gpst2time(_pEph->week, _pEph->toes);
  _pEph->toc = gpst2time(uWeekC, dToc);
}

/* decode deprecated SBP nav message for GPS (navigation data)
 * ----------------*/
static int decode_gpsnav_dep1(raw_t *raw) {

  uint8_t *puiTmp = (raw->buff) + 6;
  eph_t eph = {0};
  uint8_t prn, sat;

  trace(4, "SBP decode_gpsnav_dep1: len=%d\n", raw->len);

  if ((raw->len) < 193) {
    trace(2, "SBP decode_gpsnav_dep1 frame length error: len=%d\n", raw->len);
    return -1;
  }

  prn = U2(puiTmp) + 1; /* GPS coded as PRN-1 */
  if (!((prn >= 1) && (prn <= 37))) {
    trace(2, "SBP decode_gpsnav_dep1 prn error: sat=%d\n", prn);
    return -1;
  }

  sat = satno(SYS_GPS, prn);
  if (sat == 0)
    return -1;

  eph.code = U1(puiTmp + 2);

  decode_gpsnav_common_dep1(puiTmp, &eph);

  if (0 == timediff(raw->time, time0)) {
    eph.ttr = timeget();
  } else {
    eph.ttr = raw->time;
  }

  if (!strstr(raw->opt, "EPHALL")) {
    if ((eph.iode == raw->nav.eph[sat - 1].iode) &&
        (eph.iodc == raw->nav.eph[sat - 1].iodc)) {
      trace(3, "eph.iode %d raw->nav.eph[sat - 1].iode %d\n", eph.iode,
            raw->nav.eph[sat - 1].iode);
      trace(3, "eph.iodc %d raw->nav.eph[sat - 1].iodc %d\n", eph.iode,
            raw->nav.eph[sat - 1].iode);
      return 0;
    }
  }

  eph.sat = sat;
  raw->nav.eph[sat - 1] = eph;
  raw->ephsat = sat;
  return 2;
}

/* decode SBP nav message for GPS (navigation data) --------------------------*/
static int decode_gpsnav_dep2(raw_t *raw) {

  uint8_t *puiTmp = (raw->buff) + 6;
  eph_t eph = {0};
  uint8_t prn, sat;

  trace(4, "SBP decode_gpsnav: len=%d\n", raw->len);

  if ((raw->len) < 191) {
    trace(2, "SBP decode_gpsnav frame length error: len=%d\n", raw->len);
    return -1;
  }

  prn = puiTmp[0];
  if (!((prn >= 1) && (prn <= 37))) {
    trace(2, "SBP decode_gpsnav prn error: sat=%d\n", prn);
    return -1;
  }

  sat = satno(SYS_GPS, prn);
  if (sat == 0)
    return -1;

  eph.code = puiTmp[1];
  if ((CODE_GPS_L1CA != eph.code)) {
    trace(2, "Unrecognised code %d for G%02d\n", eph.code, prn);
    return -1;
  }

  decode_gpsnav_common_dep1(puiTmp - 2, &eph);

  if (0 == timediff(raw->time, time0)) {
    eph.ttr = timeget();
  } else {
  eph.ttr = raw->time;
  }

  if (!strstr(raw->opt, "EPHALL")) {
    if ((eph.iode == raw->nav.eph[sat - 1].iode) &&
        (eph.iodc == raw->nav.eph[sat - 1].iodc)) {
      return 0;
  }
  }

  trace(2, "decoded eph for G%02d\n", prn);

  eph.sat = sat;
  raw->nav.eph[sat - 1] = eph;
  raw->ephsat = sat;
  return 2;
}

/* decode SBP nav message for GPS (navigation data) --------------------------*/
static int decode_gpsnav(raw_t *raw) {

  uint8_t *puiTmp = (raw->buff) + 6;
  eph_t eph = {0};
  uint8_t prn, sat;

  trace(4, "SBP decode_gpsnav: len=%d\n", raw->len);

  if ((raw->len) < 147) {
    trace(2, "SBP decode_gpsnav frame length error: len=%d\n", raw->len);
    return -1;
  }

  prn = puiTmp[0];
  if (!((prn >= 1) && (prn <= 37))) {
    trace(2, "SBP decode_gpsnav prn error: sat=%d\n", prn);
    return -1;
  }

  sat = satno(SYS_GPS, prn);
  if (sat == 0)
    return -1;

  eph.code = puiTmp[1];
  if ((CODE_GPS_L1CA != eph.code)) {
    trace(2, "Unrecognised code %d for G%02d\n", eph.code, prn);
    return -1;
  }

  decode_gpsnav_common(puiTmp - 2, &eph);

  if (0 == timediff(raw->time, time0)) {
    eph.ttr = timeget();
  } else {
  eph.ttr = raw->time;
  }

  if (!strstr(raw->opt, "EPHALL")) {
    if ((eph.iode == raw->nav.eph[sat - 1].iode) &&
        (eph.iodc == raw->nav.eph[sat - 1].iodc)) {
      trace(3, "eph.iode %d raw->nav.eph[sat - 1].iode %d\n", eph.iode,
            raw->nav.eph[sat - 1].iode);
      trace(3, "eph.iodc %d raw->nav.eph[sat - 1].iodc %d\n", eph.iode,
            raw->nav.eph[sat - 1].iode);
      return 0;
    }
  }

  trace(2, "decoded eph for G%02d\n", prn);

  eph.sat = sat;
  raw->nav.eph[sat - 1] = eph;
  raw->ephsat = sat;
  return 2;
}

/* decode SBP nav message for BDS (navigation data) --------------------------*/
static int decode_bdsnav(raw_t *raw) {

  uint8_t *puiTmp = (raw->buff) + 6;
  eph_t eph = {0};
  uint8_t prn, sat;

  trace(4, "SBP decode_bdsnav: len=%d\n", raw->len);

  if ((raw->len) < 155) {
    trace(2, "SBP decode_bdsnav frame length error: len=%d\n", raw->len);
    return -1;
  }

  prn = puiTmp[0];
  if (!((prn >= 1) && (prn <= 37))) {
    trace(2, "SBP decode_bdsnav prn error: sat=%d\n", prn);
    return -1;
  }

  sat = satno(SYS_CMP, prn);
  if (sat == 0)
    return -1;

  eph.code = puiTmp[1];
  if ((CODE_BDS2_B1 != eph.code) && (CODE_BDS2_B2 != eph.code)) {
    trace(2, "Unrecognised code %d for C%02d\n", eph.code, prn);
    return -1;
  }

  decode_bdsnav_common(puiTmp - 2, &eph);

  eph.ttr = raw->time;

  if (!strstr(raw->opt, "EPHALL")) {
    if ((eph.iode == raw->nav.eph[sat - 1].iode) &&
        (eph.iodc == raw->nav.eph[sat - 1].iodc)) {
      return 0;
    }
  }

  trace(2, "decoded eph for C%02d\n", prn);

  eph.sat = sat;
  raw->nav.eph[sat - 1] = eph;
  raw->ephsat = sat;
  return 2;
}

/* decode SBP nav message for GAL (navigation data) --------------------------*/
static int decode_galnav(raw_t *raw) {

  uint8_t *puiTmp = (raw->buff) + 6;
  eph_t eph = {0};
  uint8_t prn, sat;

  trace(4, "SBP decode_galnav: len=%d\n", raw->len);

  if ((raw->len) < 160) {
    trace(2, "SBP decode_galnav frame length error: len=%d\n", raw->len);
    return -1;
  }

  prn = puiTmp[0];
  if (!((prn >= 1) && (prn <= 50))) {
    trace(2, "SBP decode_galnav prn error: sat=%d\n", prn);
    return -1;
  }

  sat = satno(SYS_GAL, prn);
  if (sat == 0)
    return -1;

  eph.code = puiTmp[1];
  if ((CODE_GAL_E1B != eph.code) && (CODE_GAL_E7I != eph.code)) {
    trace(2, "Unrecognised code %d for E%02d\n", eph.code, prn);
    return -1;
  }

  decode_galnav_common(puiTmp - 2, &eph);

  eph.ttr = raw->time;

  if (!strstr(raw->opt, "EPHALL")) {
    if ((eph.iode == raw->nav.eph[sat - 1].iode) &&
        (eph.iodc == raw->nav.eph[sat - 1].iodc)) {
      return 0;
    }
  }

  trace(2, "decoded eph for E%02d\n", prn);

  eph.sat = sat;
  raw->nav.eph[sat - 1] = eph;
  raw->ephsat = sat;
  return 2;
}

/* decode SBP nav message for Glonass (navigation data)
 * --------------------------*/
static int decode_glonav_dep1(raw_t *raw) {
  uint8_t *puiTmp = (raw->buff) + 6;
  geph_t geph = {0};
  uint8_t prn, sat, code;
  uint16_t uWeekE;
  double dSeconds;

  trace(4, "SBP decode_glonav: len=%d\n", raw->len);

  if ((raw->len) < 128) {
    trace(2, "SBP decode_glonav frame length error: len=%d\n", raw->len);
    return -1;
  }

  prn = puiTmp[0]; /* Glonass sid.sat */
  sat = satno(SYS_GLO, prn);

  if (sat == 0)
    return -1;

  if (!((prn >= 1) && (prn <= 28))) {
    trace(2, "SBP decode_glonav prn error: prn=%d\n", prn);
    return -1;
  }

  geph.sat = sat;
  code = puiTmp[1];

  if ((code != CODE_GLO_L1OF) && (code != CODE_GLO_L2OF)) {
    trace(2, "SBP decode_glonav code error: code=%d\n", code);
  }

  dSeconds = (double)U4(puiTmp + 2);
  uWeekE = U2(puiTmp + 6);
  geph.toe = gpst2time(uWeekE, dSeconds);

  dSeconds = dSeconds - floor(dSeconds / SEC_DAY) * SEC_DAY;
  dSeconds = floor((dSeconds + 900) / 1800) * 1800;
  geph.tof = utc2gpst(gpst2time(uWeekE, dSeconds));
  geph.iode = (int)puiTmp[119];

  geph.sva = (int)R8(puiTmp + 8); /* URA */
  geph.age = U4(puiTmp + 16);     /* fit interval */
  geph.svh = puiTmp[21];          /* health */
  geph.gamn = R8(puiTmp + 22);    /* */
  geph.taun = R8(puiTmp + 30);    /* */
  geph.dtaun = R8(puiTmp + 38);   /* */

  geph.pos[0] = R8(puiTmp + 46);
  geph.pos[1] = R8(puiTmp + 54);
  geph.pos[2] = R8(puiTmp + 62);

  geph.vel[0] = R8(puiTmp + 70);
  geph.vel[1] = R8(puiTmp + 78);
  geph.vel[2] = R8(puiTmp + 86);

  geph.acc[0] = R8(puiTmp + 94);
  geph.acc[1] = R8(puiTmp + 102);
  geph.acc[2] = R8(puiTmp + 110);

  geph.frq = (int)puiTmp[118] - 8;

  if (!strstr(raw->opt, "EPHALL")) {
    if (geph.iode == raw->nav.geph[prn - 1].iode)
      return 0; /* unchanged */
  }

  trace(2, "decoded eph for R%02d\n", prn);

  raw->nav.geph[prn - 1] = geph;
  raw->ephsat = sat;
  return 2;
}

/* decode SBP nav message for Glonass (navigation data)
 * --------------------------*/
static int decode_glonav(raw_t *raw) {
  uint8_t *puiTmp = (raw->buff) + 6;
  geph_t geph = {0};
  uint8_t prn, sat, code;
  uint16_t uWeekE;
  double dSeconds;

  trace(4, "SBP decode_glonav: len=%d\n", raw->len);

  if ((raw->len) < 100) {
    trace(2, "SBP decode_glonav frame length error: len=%d\n", raw->len);
    return -1;
  }

  prn = puiTmp[0]; /* Glonass sid.sat */
  sat = satno(SYS_GLO, prn);

  if (sat == 0)
    return -1;

  if (!((prn >= 1) && (prn <= 28))) {
    trace(2, "SBP decode_glonav prn error: prn=%d\n", prn);
    return -1;
  }

  geph.sat = sat;
  code = puiTmp[1];

  if ((code != CODE_GLO_L1OF) && (code != CODE_GLO_L2OF)) {
    trace(2, "SBP decode_glonav code error: code=%d\n", code);
  }

  dSeconds = (double)U4(puiTmp + 2);
  uWeekE = U2(puiTmp + 6);
  geph.toe = gpst2time(uWeekE, dSeconds);

  dSeconds = dSeconds - floor(dSeconds / SEC_DAY) * SEC_DAY;
  dSeconds = floor((dSeconds + 900) / 1800) * 1800;
  geph.tof = utc2gpst(gpst2time(uWeekE, dSeconds));
  geph.iode = (int)puiTmp[91];

  geph.sva = (int)R4(puiTmp + 8); /* URA */
  geph.age = U4(puiTmp + 12);     /* fit interval */
  geph.svh = puiTmp[17];          /* health */
  geph.gamn = R4(puiTmp + 18);    /* */
  geph.taun = R4(puiTmp + 22);    /* */
  geph.dtaun = R4(puiTmp + 26);   /* */

  geph.pos[0] = R8(puiTmp + 30);
  geph.pos[1] = R8(puiTmp + 38);
  geph.pos[2] = R8(puiTmp + 46);

  geph.vel[0] = R8(puiTmp + 54);
  geph.vel[1] = R8(puiTmp + 62);
  geph.vel[2] = R8(puiTmp + 70);

  geph.acc[0] = R4(puiTmp + 78);
  geph.acc[1] = R4(puiTmp + 82);
  geph.acc[2] = R4(puiTmp + 86);

  geph.frq = (int)puiTmp[90] - 8;

  if (!strstr(raw->opt, "EPHALL")) {
    if (geph.iode == raw->nav.geph[prn - 1].iode)
      return 0; /* unchanged */
  }

  trace(2, "decoded eph for R%02d\n", prn);

  raw->nav.geph[prn - 1] = geph;
  raw->ephsat = sat;
  return 2;
}

/* decode SBF gpsion --------------------------------------------------------*/
static int decode_gpsion(raw_t *raw) {

  uint8_t *puiTmp = (raw->buff) + 6;

  trace(4, "SBP decode_gpsion: len=%d\n", raw->len);

  if ((raw->len) < 72) {
    trace(2, "SBP decode_gpsion frame length error: len=%d\n", raw->len);
    return -1;
  }

  raw->nav.ion_gps[0] = R8(puiTmp + 6);
  raw->nav.ion_gps[1] = R8(puiTmp + 14);
  raw->nav.ion_gps[2] = R8(puiTmp + 18);
  raw->nav.ion_gps[3] = R8(puiTmp + 22);
  raw->nav.ion_gps[4] = R8(puiTmp + 30);
  raw->nav.ion_gps[5] = R8(puiTmp + 38);
  raw->nav.ion_gps[6] = R8(puiTmp + 46);
  raw->nav.ion_gps[7] = R8(puiTmp + 54);

  return 9;
}

/* decode sbas navigation data -----------------------------------------------*/
static int decode_snav(raw_t *raw) {
  uint8_t *puiTmp = (raw->buff) + 6;
  uint8_t sat, code, sbas_msg;
  int32_t week;
  uint32_t tow_ms, k;
  uint8_t uSbasPream[3] = {0x53, 0x9A, 0xC6};
  time2gpst(timeadd(raw->time, -1.0), &week);

  trace(4, "MSG_SBAS_RAW: len=%d\n", raw->len);

  if ((raw->len) < 42) {
    trace(2, "MSG_SBAS_RAW frame length error: len=%d\n", raw->len);
    return -1;
  }

  sat = puiTmp[0];
  if (!((sat >= 120) && (sat <= 138))) {
    trace(2, "MSG_SBAS_RAW PRN error: sat=%d\n", sat);
    return -1;
  }

  code = puiTmp[1];
  if (CODE_SBAS_L1CA != code) {
    trace(2, "MSG_SBAS_RAW PRN error: code=%d\n", code);
    return -1;
  }

  tow_ms = U4(puiTmp + 2);

  raw->sbsmsg.prn = sat;
  raw->sbsmsg.week = week;
  raw->sbsmsg.tow = tow_ms / 1000 - 1;
  raw->sbsmsg.msg[0] = uSbasPream[((raw->sbsmsg.tow) % 3)];
  sbas_msg = puiTmp[6];

  for (k = 0; k < 27; k++) {
    raw->sbsmsg.msg[1 + k] = (sbas_msg << 2) | ((puiTmp[7 + k] >> 6) & 0x03);
    sbas_msg = puiTmp[7 + k];
  }
  raw->sbsmsg.msg[1 + k] = (sbas_msg << 2);

  return 3;
}

/* decode external event -----------------------------------------------*/
static int decode_event(raw_t *raw) {
  gtime_t eventime;
  uint8_t *puiTmp = (raw->buff) + 6;
  uint16_t wn;
  uint32_t tow;
  int32_t ns_residual;
  uint8_t flags,pin;

  trace(4, "MSG_EXT_EVENT: len=%d\n", raw->len);

  if ((raw->len) < 20) {
    trace(2, "SBP decode_event length error: len=%d\n", raw->len);
    return -1;
  }

/* get message fields */
  wn = U2(puiTmp + 0);
  tow = U4(puiTmp + 2);
  ns_residual = I4(puiTmp + 6);
  flags = U1(puiTmp + 10);
  pin = U1(puiTmp + 11);

/* write to event variables */
  eventime = gpst2time(wn,tow*1E-3+ns_residual*1E-9);
  raw->obuf.flag = 5; /* event flag */
  raw->obuf.data[0].eventime = eventime;
  raw->obuf.tmcount++;
  raw->obuf.data[0].timevalid = (flags & 0x02)>>1;

  return 0;
}

/* decode SBF raw message --------------------------------------------------*/
static int decode_sbp(raw_t *raw) {
  uint16_t crc, uCalcCrc;

  /* read the SBF block ID and revision */
  int type = U2(raw->buff + 1);
  int sender = U2(raw->buff + 3);

  if ((sender == 0) && (NULL == strstr(raw->opt, "CONVBASE")))
    return 0;
  if ((sender != 0) && (NULL != strstr(raw->opt, "CONVBASE")))
    return 0;

  trace(4, "decode_sbp: type=%04x len=%d\n", type, raw->len);

  /* read the SBF block CRC */
  crc = U2(raw->buff + (raw->len) - 2);

  /* checksum skipping first 4 bytes */
  uCalcCrc = sbp_checksum(raw->buff + 1, raw->len - 3);
  if (uCalcCrc != crc) {
    trace(2, "SBP checksum error: type=%04x len=%d\n", type, raw->len);
    return -1;
  }

  if (raw->outtype) {
    sprintf(raw->msgtype, "SBP 0x%04X (%4d):", type, raw->len);
  }

  switch (type) {
  case ID_MEASEPOCH:
    return decode_msgobs(raw);
  case ID_MSGEPHGPS_DEP1:
    return decode_gpsnav_dep1(raw);
  case ID_MSGEPHGPS_DEP2:
    return decode_gpsnav_dep2(raw);
  case ID_MSGEPHGPS:
    return decode_gpsnav(raw);
  case ID_MSGEPHBDS:
    return decode_bdsnav(raw);
  case ID_MSGEPHGAL:
    return decode_galnav(raw);
  case ID_MSGEPHGLO_DEP1:
    return decode_glonav_dep1(raw);
  case ID_MSGEPHGLO:
    return decode_glonav(raw);
  case ID_MSGIONGPS:
    return decode_gpsion(raw);
  case ID_MSG_SBAS_RAW:
    return decode_snav(raw);
  case ID_MSG_EXT_EVENT:
    return decode_event(raw);

  default:
    trace(4, "decode_sbp: unused frame type=%04x len=%d\n", type, raw->len);
    /* there are many more SBF blocks to be extracted */
  }
  return 0;
}

/* sync to the beginning of a block ------------------------------------------*/
static int sync_sbp(uint8_t *buff, uint8_t data) {
  buff[0] = data;
  return buff[0] == SBP_SYNC1;
}
/* input sbf raw data from stream ----------------------------------------------
 * get to the next sbf raw block from stream
 * args   : raw_t  *raw   IO     receiver raw data control struct
 *          uint8_t data I stream data (1byte)
 * return : status (-1: error message, 0: no message, 1: input observation data,
 *                  2: input ephemeris, 3: input sbas message,
 *                  9: input ion/utc parameter)
 *-----------------------------------------------------------------------------*/
extern int input_sbp(raw_t *raw, uint8_t data) {
  trace(5, "input_sbp: data=%02x\n", data);

  if (raw->nbyte == 0) {
    if (sync_sbp(raw->buff, data))
      raw->nbyte = 1;
    return 0;
  }
  raw->buff[raw->nbyte++] = data;

  if (raw->nbyte < 6)
    return 0;

  if ((raw->len = (8 + raw->buff[5])) > MAXRAWLEN) {
    trace(2, "sbp length error: len=%d\n", raw->len);
    raw->nbyte = 0;
    return -1;
  }
  if (raw->nbyte < raw->len)
    return 0;
  raw->nbyte = 0;

  return decode_sbp(raw);
}

/* start input file ----------------------------------------------------------*/
static void startfile(raw_t *raw) {
  raw->tod = -1;
  raw->obuf.n = 0;
  raw->buff[0] = 0;
}
/* end input file ------------------------------------------------------------*/
static int endfile(raw_t *raw) {
  /* flush observation data buffer */
  if (!flushobuf(raw))
    return -2;
  raw->obuf.n = 0;
  return 1;
}

/* sbf raw block finder --------------------------------------------------------
 * get to the next sbf raw block from file
 * args   : raw_t  *raw   IO     receiver raw data control struct
 *          FILE   *fp    I      file pointer
 * return : status(-2: end of file, -1...9: same as above)
 *-----------------------------------------------------------------------------*/
extern int input_sbpf(raw_t *raw, FILE *fp) {
  int i, data, stat;

  trace(4, "input_sbpf:\n");

  if (raw->flag) {
    startfile(raw);
    raw->flag = 0;
  }

  /* go to the beginning of the first block */
  if (raw->nbyte == 0) {
    for (i = 0;; i++) {
      if ((data = fgetc(fp)) == EOF)
        return endfile(raw);
      if (sync_sbp(raw->buff, (uint8_t)data))
        break;
      if (i >= MAXRAWLEN)
        return 0;
    }
  }

  /* load block header content (8 bytes) in raw->buff */
  /* since we already read the first byte, we just read the next 5 bytes */
  if (fread(raw->buff + 1, 1, 5, fp) < 5)
    return endfile(raw);
  raw->nbyte = 6;

  /* decode the length of the block and store it in len */
  if ((raw->len = 8 + raw->buff[5]) > MAXRAWLEN) {
    trace(2, "sbp length error: len=%d\n", raw->len);
    raw->nbyte = 0;
    return -1;
  }

  /* let's store in raw->buff the whole block of length len */
  /* 8 bytes have been already read, we read raw->len-8 more */
  if (fread(raw->buff + 6, 1, raw->len - 6, fp) < (size_t)(raw->len - 6))
    return endfile(raw);

  /* decode SBF block */
  stat = decode_sbp(raw);

  clearbuff(raw);
  return stat;
}

/* sbf json block finder
 *--------------------------------------------------------
 * get to the next meaningful sbf json message
 * args   : raw_t  *raw   IO     receiver raw data control struct
 *          FILE   *fp    I      file pointer
 * return : status(-2: end of file, -1...9: same as above)
 *-----------------------------------------------------------------------------*/
extern int input_sbpjsonf(raw_t *raw, FILE *fp) {
  const char JSON_MSGTYPE_FIELD[] = "\"msg_type\":";
  const char JSON_SENDER_FIELD[] = "\"sender\":";
  const char JSON_PAYLOAD_FIELD[] = "\"payload\":";
  const char JSON_CRC_FIELD[] = "\"crc\":";
  uint8_t *pcPayloadBeg, *pcPayloadEnd;
  int stat, iRet;
  uint32_t uPayloadSize, uMsgType, uSender, uMsgCrc, uLength;
  char *pcTmp;

  trace(4, "input_sbpjsonf:\n");

  if (raw->flag) {
    startfile(raw);
    raw->flag = 0;
  }

  memset(raw->buff, 0, MAXRAWLEN);
  pcTmp = fgets((char *)raw->buff, MAXRAWLEN, fp);
  if (NULL == pcTmp) {
    return endfile(raw);
  }

  pcTmp = strstr((char *)raw->buff, JSON_MSGTYPE_FIELD);
  if (NULL == pcTmp)
    return 0;
  iRet = sscanf(pcTmp + strlen(JSON_MSGTYPE_FIELD), "%u", &uMsgType);
  if (0 == iRet)
    return 0;

  /* avoid parsing the payload if the message isn't supported in the first place
   */
  if ((uMsgType != ID_MEASEPOCH) && (uMsgType != ID_MSGEPHGPS_DEP1) &&
      (uMsgType != ID_MSGEPHGPS_DEP2) && (uMsgType != ID_MSGEPHGPS) &&
      (uMsgType != ID_MSGEPHBDS) && (uMsgType != ID_MSGEPHGAL) &&
      (uMsgType != ID_MSGEPHGLO_DEP1) && (uMsgType != ID_MSGEPHGLO) &&
      (uMsgType != ID_MSGIONGPS) && (uMsgType != ID_MSG_SBAS_RAW)) {
    return 0;
  }

  /* sender in clear */
  pcTmp = strstr((char *)raw->buff, JSON_SENDER_FIELD);
  if (NULL == pcTmp)
    return 0;
  iRet = sscanf(pcTmp + strlen(JSON_SENDER_FIELD), "%u", &uSender);
  if (0 == iRet)
    return 0;

  /* crc */
  pcTmp = strstr((char *)raw->buff, JSON_CRC_FIELD);
  if (NULL == pcTmp)
    return 0;
  iRet = sscanf(pcTmp + strlen(JSON_CRC_FIELD), "%u", &uMsgCrc);
  if (0 == iRet)
    return 0;

  /* payload */
  pcTmp = strstr((char *)raw->buff, JSON_PAYLOAD_FIELD);
  if (NULL == pcTmp)
    return 0;
  pcTmp += strlen(JSON_PAYLOAD_FIELD);

  pcPayloadBeg = (uint8_t *)strchr((char *)pcTmp, '\"') + 1;
  pcPayloadEnd = (uint8_t *)strchr((char *)pcPayloadBeg, '\"') - 1;
  if ((NULL == pcPayloadBeg) || (NULL == pcPayloadEnd))
    return 0;
  uPayloadSize = pcPayloadEnd - pcPayloadBeg + 1;
  pcPayloadEnd[1] = 0;
  /* fprintf(stderr, "%4d: %s\n", uPayloadSize, pcPayloadBeg); */
  memset(puPayloadTmp, 0, sizeof(puPayloadTmp));
  uLength = 256;
  Base64_Decode(pcPayloadBeg, uPayloadSize, puPayloadTmp, &uLength);

  raw->buff[0] = 0x55;                   /* sync char */
  raw->buff[1] = (uMsgType >> 0) & 0xFF; /* msg type LSB */
  raw->buff[2] = (uMsgType >> 8) & 0xFF; /* msg type MSB */
  raw->buff[3] = (uSender >> 0) & 0xFF;  /* sender LSB */
  raw->buff[4] = (uSender >> 8) & 0xFF;  /* sender MSB */
  raw->buff[5] = uLength;                /* payload length */
  memcpy(raw->buff + 6, puPayloadTmp, uLength);
  raw->buff[6 + uLength] = (uMsgCrc >> 0) & 0xFF; /* CRC LSB */
  raw->buff[7 + uLength] = (uMsgCrc >> 8) & 0xFF; /* CRC MSB */

  /* decode SBF block */
  raw->len = 8 + uLength;
  stat = decode_sbp(raw);

  clearbuff(raw);
  return stat;
}
