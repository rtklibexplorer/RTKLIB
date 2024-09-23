/*------------------------------------------------------------------------------
* unicore.c : Unicore UM980/981/982 decoder functions
*
*          Copyright (C) 2024 Avinab Malla, All rights reserved.
*
* version : $Revision: 1.0 $ $Date: 2024/07/23$
* references:
*   [1] Unicore N4 Products Commands and Logs Reference Book R1.4 (June 2024)
*
* history :
*   2024/07/23  1.0  new
*   2024/07/28  1.0.1  Added references, fixed sig2code
*
*/
#include "rtklib.h"

#define SYNC1       0xAA    /* sync code 1 */
#define SYNC2       0x44    /* sync code 2 */
#define SYNC3       0xB5    /* sync code 3 */
#define HLEN        24      /* message header length (bytes) */



/* message IDs */
#define ID_OBSVM       12      /* Observation of the Master Antenna */
#define ID_GPSEPH      106     /* GPS Ephemeros */
#define ID_GLOEPH      107     /* GLONASS Ephemeris */
#define ID_BDSEPH      108     /* BDS Ephemeris */
#define ID_GALEPH      109     /* Galileo Ephemeris */
#define ID_QZSSEPH     110     /* Galileo Ephemeris */
#define ID_IRNSSEPH    112     /* IRNSS Ephemeris */
#define ID_BD3EPH      2999    /* BDS-3 Ephemeris */

#define OFF_FRQNO       -7      /* F/W ver.3.620 */



/* get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((uint8_t *)(p)))
#define I1(p) (*((int8_t  *)(p)))
static uint16_t U2(uint8_t* p) { uint16_t u; memcpy(&u, p, 2); return u; }
static uint32_t U4(uint8_t* p) { uint32_t u; memcpy(&u, p, 4); return u; }
static int32_t  I4(uint8_t* p) { int32_t  i; memcpy(&i, p, 4); return i; }
static float    R4(uint8_t* p) { float    r; memcpy(&r, p, 4); return r; }
static double   R8(uint8_t* p) { double   r; memcpy(&r, p, 8); return r; }


/* sync header ---------------------------------------------------------------*/
static int sync_unicore(uint8_t* buff, uint8_t data)
{
    buff[0] = buff[1]; buff[1] = buff[2]; buff[2] = data;
    return buff[0] == SYNC1 && buff[1] == SYNC2 && buff[2] == SYNC3;
}

/* URA value (m) to URA index ------------------------------------------------*/
static int uraindex(double value)
{
    static const double ura_eph[] = {
        2.4,3.4,4.85,6.85,9.65,13.65,24.0,48.0,96.0,192.0,384.0,768.0,1536.0,
        3072.0,6144.0,0.0
    };
    int i;
    for (i = 0; i < 15; i++) if (ura_eph[i] >= value) break;
    return i;
}

/* get observation data index ------------------------------------------------*/
static int obsindex(obs_t* obs, gtime_t time, int sat)
{
    int i, j;

    if (obs->n >= MAXOBS) return -1;
    for (i = 0; i < obs->n; i++) {
        if (obs->data[i].sat == sat) return i;
    }
    obs->data[i].time = time;
    obs->data[i].sat = sat;
    for (j = 0; j < NFREQ + NEXOBS; j++) {
        obs->data[i].L[j] = obs->data[i].P[j] = 0.0;
        obs->data[i].D[j] = 0.0;
        obs->data[i].SNR[j] = obs->data[i].LLI[j] = 0;
        obs->data[i].code[j] = CODE_NONE;
    }
    obs->n++;
    return i;
}

/* adjust weekly rollover of GPS time ----------------------------------------*/
static gtime_t adjweek(gtime_t time, double tow)
{
    double tow_p;
    int week;
    tow_p = time2gpst(time, &week);
    if (tow < tow_p - 302400.0) tow += 604800.0;
    else if (tow > tow_p + 302400.0) tow -= 604800.0;
    return gpst2time(week, tow);
}

/* check code priority and return freq-index ---------------------------------*/
static int checkpri(const char* opt, int sys, int code, int idx)
{
    int nex = NEXOBS;

    if (sys == SYS_GPS) {
        if (strstr(opt, "-GL1L") && idx == 0) return (code == CODE_L1L) ? 0 : -1;
        if (strstr(opt, "-GL2S") && idx == 1) return (code == CODE_L2X) ? 1 : -1;
        if (strstr(opt, "-GL2P") && idx == 1) return (code == CODE_L2P) ? 1 : -1;
        if (code == CODE_L1L) return (nex < 1) ? -1 : NFREQ;
        if (code == CODE_L2S) return (nex < 2) ? -1 : NFREQ + 1;
        if (code == CODE_L2P) return (nex < 3) ? -1 : NFREQ + 2;
    }
    else if (sys == SYS_GLO) {
        if (strstr(opt, "-RL2C") && idx == 1) return (code == CODE_L2C) ? 1 : -1;
        if (code == CODE_L2C) return (nex < 1) ? -1 : NFREQ;
    }
    else if (sys == SYS_GAL) {
        if (strstr(opt, "-EL6B") && idx == 3) return (code == CODE_L6B) ? 3 : -1;
        if (code == CODE_L6B) return (nex < 2) ? -1 : NFREQ;
    }
    else if (sys == SYS_QZS) {
        if (strstr(opt, "-JL1L") && idx == 0) return (code == CODE_L1L) ? 0 : -1;
        if (strstr(opt, "-JL1Z") && idx == 0) return (code == CODE_L1Z) ? 0 : -1;
        if (code == CODE_L1L) return (nex < 1) ? -1 : NFREQ;
        if (code == CODE_L1Z) return (nex < 2) ? -1 : NFREQ + 1;
    }
    else if (sys == SYS_CMP) {
        if (strstr(opt, "-CL1P") && idx == 0) return (code == CODE_L1P) ? 0 : -1;
        if (strstr(opt, "-CL7D") && idx == 0) return (code == CODE_L7D) ? 0 : -1;
        if (code == CODE_L1P) return (nex < 1) ? -1 : NFREQ;
        if (code == CODE_L7D) return (nex < 2) ? -1 : NFREQ + 1;
    }
    return idx < NFREQ ? idx : -1;
}

/* signal type to obs code ---------------------------------------------------*/
static int sig2code(int sys, int sigtype, int l2c)
{
    if (sys == SYS_GPS) {
        switch (sigtype) {
        case 0: return CODE_L1C;  // L1C/A 
        case 9: return l2c == 1 ? CODE_L2S : CODE_L2W;  // L2P(Y),semi-codeless or L2C(M)
        case 3: return CODE_L1L;  // L1C Pilot
        case 11: return CODE_L1S; // L1C Data, semi-codeless
        case 6: return CODE_L5I;  // L5 Data 
        case 14: return CODE_L5Q; // L5 Pilot 
        case 17: return CODE_L2L; // L2C(L)
        default: return 0;
        }
    }
    else if (sys == SYS_GLO) {
        switch (sigtype) {
        case  0: return CODE_L1C; // L1C/A
        case  5: return CODE_L2C; // L2C/A
        case  6: return CODE_L3I; // G3I
        case  7: return CODE_L3Q; // G3Q
        default: return 0;
        }
    }
    else if (sys == SYS_GAL) {
        switch (sigtype) {
        case  1: return CODE_L1B; // E1B
        case  2: return CODE_L1C; // E1C
        case 12: return CODE_L5Q; // E5A Pilot
        case 17: return CODE_L7Q; // E5B Pilot
        case 18: return CODE_L6B; // E6B
        case 22: return CODE_L6C; // E6C
        default: return 0;
        }
    }
    else if (sys == SYS_QZS) {
        switch (sigtype) {
        case  0: return CODE_L1C; // L1C/A
        case  9: return l2c == 1 ? CODE_L2S : CODE_L2W;  // L2P(Y),semi-codeless or L2C(M)
        case  3: return CODE_L1L; // L1C pilot
        case  6: return CODE_L5I; // L5 Data
        case 11: return CODE_L1S; // L1C Data
        case 14: return CODE_L5Q; // L5 Pilot
        case 17: return CODE_L2L; // L2C(L)
        case 21: return CODE_L6Z; // L6D
        case 27: return CODE_L6E; // L6E
        default: return 0;
        }
    }
    else if (sys == SYS_CMP) {
        switch (sigtype) {
        case  0: return CODE_L2I; // B1I
        case  4: return CODE_L2Q; // B1Q
        case  8: return CODE_L1P; // B1C pilot
        case 23: return CODE_L1D; // B1C Data
        case  5: return CODE_L7Q; // B2Q
        case 17: return CODE_L7I; // B2I
        case 12: return CODE_L5P; // B2a Pilot
        case 28: return CODE_L5D; // B2a Data
        case  6: return CODE_L6Q; // B3Q
        case 21: return CODE_L6I; // B3I
        case 13: return CODE_L7P; // B2b (I)
        default: return 0;
        }
    }
    else if (sys == SYS_IRN) {
        switch (sigtype) {
        case  6: return CODE_L5A; //L5 Data
        case  14: return CODE_L5C; //L5 Pilot
        default: return 0;
        }
    }
    else if (sys == SYS_SBS) {
        switch (sigtype) {
        case  0: return CODE_L1C; // L1C/A 
        case  6: return CODE_L5I; // L5I
        default: return 0;
        }
    }
    return 0;
}

static int decode_track_stat(uint32_t stat, int* sys, int* code, int* plock, int* clock)
{
    int satsys, sigtype, idx = -1;
    int l2c;

    *code = CODE_NONE;
    *plock = (stat >> 10) & 1;
    *clock = (stat >> 12) & 1;
    satsys = (stat >> 16) & 7;
    sigtype = (stat >> 21) & 0x1F;
    l2c = (stat >> 26) & 0x01;


    switch (satsys) {
    case 0: *sys = SYS_GPS; break;
    case 1: *sys = SYS_GLO; break;
    case 2: *sys = SYS_SBS; break;
    case 3: *sys = SYS_GAL; break;
    case 4: *sys = SYS_CMP; break;
    case 5: *sys = SYS_QZS; break;
    case 6: *sys = SYS_IRN; break;
    default:
        trace(2, "unicore unknown system: sys=%d\n", satsys);
        return -1;
    }
    if (!(*code = sig2code(*sys, sigtype, l2c)) || (idx = code2idx(*sys, *code)) < 0) {
        trace(2, "unicore signal type error: sys=%d sigtype=%d\n", *sys, sigtype);
        return -1;
    }
    return idx;
}

static int decode_gpsephb(raw_t* raw)
{
    eph_t eph = { 0 };
    uint8_t* p = raw->buff + HLEN;

    double tow, tocs, N, URA;
    int prn, as, sat, week, zweek, health;
    int iode1, iode2;

    if (raw->len < HLEN + 224) {
        trace(2, "unicore gpsephb length error: len=%d\n", raw->len);
        return -1;
    }

    prn = U4(p);   p += 4;
    tow = R8(p); p += 8;
    health = U4(p) & 0x3f; p += 4;
    eph.iode = U4(p); p += 4;
    iode2 = U4(p); p += 4;

    eph.week = U4(p); p += 4;
    zweek = U4(p); p += 4;
    eph.toes = R8(p); p += 8;
    eph.A = R8(p); p += 8;
    eph.deln = R8(p); p += 8;
    eph.M0 = R8(p); p += 8;
    eph.e = R8(p); p += 8;
    eph.omg = R8(p); p += 8;
    eph.cuc = R8(p); p += 8;
    eph.cus = R8(p); p += 8;
    eph.crc = R8(p); p += 8;
    eph.crs = R8(p); p += 8;
    eph.cic = R8(p); p += 8;
    eph.cis = R8(p); p += 8;
    eph.i0 = R8(p); p += 8;
    eph.idot = R8(p); p += 8;
    eph.OMG0 = R8(p); p += 8;
    eph.OMGd = R8(p); p += 8;
    eph.iodc = U4(p); p += 4;
    tocs = R8(p); p += 8;
    eph.tgd[0] = R8(p); p += 8;
    eph.f0 = R8(p); p += 8;
    eph.f1 = R8(p); p += 8;
    eph.f2 = R8(p); p += 8;
    as = U4(p); p += 4;
    N = R8(p); p += 8;
    eph.sva = uraindex(R8(p)); p += 8;

    if (!(sat = satno(SYS_GPS, prn))) {
        trace(2, "unicore gpsephb satellite error: prn=%d\n", prn);
        return -1;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype + strlen(raw->msgtype), " prn=%d", prn);
    }

    eph.sat = sat;
    eph.code = 0;
    eph.iodc = eph.iode;
    tow = time2gpst(raw->time, &week);
    eph.week = week; /* gps-week = gal-week */
    eph.toe = gpst2time(eph.week, eph.toes);

    double tt = timediff(eph.toe, raw->time);
    if (tt < -302400.0) eph.week++;
    else if (tt > 302400.0) eph.week--;
    eph.toe = gpst2time(eph.week, eph.toes);
    eph.toc = adjweek(raw->time, tocs);
    eph.ttr = raw->time;

    if (!strstr(raw->opt, "-EPHALL")) {
        if (timediff(raw->nav.eph[sat - 1].toe, eph.toe) == 0.0 &&
            timediff(raw->nav.eph[sat - 1].toc, eph.toc) == 0.0) return 0;
    }

    raw->nav.eph[sat - 1] = eph;
    raw->ephsat = sat;
    raw->ephset = 0;
    return 2;
}

static int decode_gloephb(raw_t* raw)
{
    uint8_t* p = raw->buff + HLEN;
    geph_t geph = { 0 };
    double tow, tof, toff;
    int prn, sat, week;

    if (raw->len < HLEN + 144) {
        trace(2, "unicore gloephb length error: len=%d\n", raw->len);
        return -1;
    }
    prn = U2(p) - 37; p += 2; //Slot o

    if (!(sat = satno(SYS_GLO, prn))) {
        trace(2, "unicore gloephb prn error: prn=%d\n", prn);
        return -1;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype + strlen(raw->msgtype), " prn=%d", prn);
    }
    geph.frq = U2(p) + OFF_FRQNO; p += 2;
    int satType = U1(p); p += 1 + 1;

    week = U2(p); p += 2;
    tow = floor(U4(p) / 1000.0 + 0.5); p += 4; /* rounded to integer sec */
    toff = U4(p); p += 4;
    int Nt = U2(p); p += 2 + 2;
    geph.iode = U4(p) & 0x7F; p += 4;
    geph.svh = (U4(p) < 4) ? 0 : 1; p += 4; /* 0:healthy,1:unhealthy */
    geph.pos[0] = R8(p); p += 8;
    geph.pos[1] = R8(p); p += 8;
    geph.pos[2] = R8(p); p += 8;
    geph.vel[0] = R8(p); p += 8;
    geph.vel[1] = R8(p); p += 8;
    geph.vel[2] = R8(p); p += 8;
    geph.acc[0] = R8(p); p += 8;
    geph.acc[1] = R8(p); p += 8;
    geph.acc[2] = R8(p); p += 8;
    geph.taun = R8(p); p += 8;
    geph.dtaun = R8(p); p += 8;
    geph.gamn = R8(p); p += 8;
    tof = U4(p) - toff; p += 4; /* glonasst->gpst */

    int P = U4(p); p += 4;
    int Ft = U4(p); p += 4;
    geph.age = U4(p); p += 4;

    geph.toe = gpst2time(week, tow);
    tof += floor(tow / 86400.0) * 86400;
    if (tof < tow - 43200.0) tof += 86400.0;
    else if (tof > tow + 43200.0) tof -= 86400.0;
    geph.tof = gpst2time(week, tof);

    if (!strstr(raw->opt, "-EPHALL")) {
        if (fabs(timediff(geph.toe, raw->nav.geph[prn - 1].toe)) < 1.0 &&
            geph.svh == raw->nav.geph[prn - 1].svh) return 0; /* unchanged */
    }
    geph.sat = sat;
    raw->nav.geph[prn - 1] = geph;
    raw->ephsat = sat;
    raw->ephset = 0;
    return 2;
}

static int decode_galephb(raw_t* raw) {
    eph_t eph = { 0 };
    uint8_t* p = raw->buff + HLEN;
    double tow, sqrtA, af0_fnav, af1_fnav, af2_fnav, af0_inav, af1_inav, af2_inav, tt;
    int prn, sat, week, rcv_fnav, rcv_inav, svh_e1b, svh_e5a, svh_e5b, dvs_e1b, dvs_e5a;
    int dvs_e5b, toc_fnav, toc_inav, set, sel_eph = 3; /* 1:I/NAV+2:F/NAV */

    if (strstr(raw->opt, "-GALINAV")) sel_eph = 1;
    if (strstr(raw->opt, "-GALFNAV")) sel_eph = 2;

    if (raw->len < HLEN + 220) {
        trace(2, "unicore galephb length error: len=%d\n", raw->len);
        return -1;
    }
    prn = U4(p);   p += 4;
    rcv_fnav = U4(p) & 1; p += 4;
    rcv_inav = U4(p) & 1; p += 4;
    svh_e1b = U1(p) & 3; p += 1;
    svh_e5a = U1(p) & 3; p += 1;
    svh_e5b = U1(p) & 3; p += 1;
    dvs_e1b = U1(p) & 1; p += 1;
    dvs_e5a = U1(p) & 1; p += 1;
    dvs_e5b = U1(p) & 1; p += 1;
    eph.sva = U1(p);   p += 1 + 1; /* SISA index */
    eph.iode = U4(p);   p += 4;   /* IODNav */
    eph.toes = U4(p);   p += 4;
    sqrtA = R8(p);   p += 8;
    eph.deln = R8(p);   p += 8;
    eph.M0 = R8(p);   p += 8;
    eph.e = R8(p);   p += 8;
    eph.omg = R8(p);   p += 8;
    eph.cuc = R8(p);   p += 8;
    eph.cus = R8(p);   p += 8;
    eph.crc = R8(p);   p += 8;
    eph.crs = R8(p);   p += 8;
    eph.cic = R8(p);   p += 8;
    eph.cis = R8(p);   p += 8;
    eph.i0 = R8(p);   p += 8;
    eph.idot = R8(p);   p += 8;
    eph.OMG0 = R8(p);   p += 8;
    eph.OMGd = R8(p);   p += 8;
    toc_fnav = U4(p);   p += 4;
    af0_fnav = R8(p);   p += 8;
    af1_fnav = R8(p);   p += 8;
    af2_fnav = R8(p);   p += 8;
    toc_inav = U4(p);   p += 4;
    af0_inav = R8(p);   p += 8;
    af1_inav = R8(p);   p += 8;
    af2_inav = R8(p);   p += 8;
    eph.tgd[0] = R8(p);   p += 8; /* BGD: E5A-E1 (s) */
    eph.tgd[1] = R8(p);         /* BGD: E5B-E1 (s) */

    if (!(sat = satno(SYS_GAL, prn))) {
        trace(2, "unicore galephb satellite error: prn=%d\n", prn);
        return -1;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype + strlen(raw->msgtype), " prn=%d", prn);
    }
    set = rcv_fnav ? 1 : 0; /* 0:I/NAV,1:F/NAV */
    if (!(sel_eph & 1) && set == 0) return 0;
    if (!(sel_eph & 2) && set == 1) return 0;

    eph.sat = sat;
    eph.A = sqrtA * sqrtA;
    eph.f0 = set ? af0_fnav : af0_inav;
    eph.f1 = set ? af1_fnav : af1_inav;
    eph.f2 = set ? af2_fnav : af2_inav;
    eph.svh = ((svh_e5b << 7) | (dvs_e5b << 6) | (svh_e5a << 4) | (dvs_e5a << 3) |
        (svh_e1b << 1) | dvs_e1b);
    eph.code = set ? ((1 << 1) + (1 << 8)) : ((1 << 0) + (1 << 2) + (1 << 9));
    eph.iodc = eph.iode;
    tow = time2gpst(raw->time, &week);
    eph.week = week; /* gps-week = gal-week */
    eph.toe = gpst2time(eph.week, eph.toes);

    tt = timediff(eph.toe, raw->time);
    if (tt < -302400.0) eph.week++;
    else if (tt > 302400.0) eph.week--;
    eph.toe = gpst2time(eph.week, eph.toes);
    eph.toc = adjweek(raw->time, set ? toc_fnav : toc_inav);
    eph.ttr = raw->time;

    if (!strstr(raw->opt, "-EPHALL")) {
        if (eph.iode == raw->nav.eph[sat - 1 + MAXSAT * set].iode &&
            timediff(eph.toe, raw->nav.eph[sat - 1 + MAXSAT * set].toe) == 0.0 &&
            timediff(eph.toc, raw->nav.eph[sat - 1 + MAXSAT * set].toc) == 0.0) {
            return 0; /* unchanged */
        }
    }
    raw->nav.eph[sat - 1 + MAXSAT * set] = eph;
    raw->ephsat = sat;
    raw->ephset = set;
    return 2;
}

static int decode_bdsephb(raw_t* raw)
{
    eph_t eph = { 0 };
    uint8_t* p = raw->buff + HLEN;
    double ura;
    int prn, sat, toc;

    if (raw->len < HLEN + 232) {
        trace(2, "unicore bdsephb length error: len=%d\n", raw->len);
        return -1;
    }
    prn = U4(p);   p += 4;
    double tow = R8(p); p += 8;
    eph.svh = U4(p); p += 4;
    eph.iode = U4(p); p += 4;
    uint32_t AODE2 = U4(p); p += 4;
    eph.week = U4(p) - 1356;   p += 4;
    int zweek = U4(p);   p += 4;
    eph.toes = R8(p);   p += 8;
    eph.A = R8(p);   p += 8;
    eph.deln = R8(p);   p += 8;
    eph.M0 = R8(p);   p += 8;
    eph.e = R8(p);   p += 8;
    eph.omg = R8(p);   p += 8;
    eph.cuc = R8(p);   p += 8;
    eph.cus = R8(p);   p += 8;
    eph.crc = R8(p);   p += 8;
    eph.crs = R8(p);   p += 8;
    eph.cic = R8(p);   p += 8;
    eph.cis = R8(p);   p += 8;
    eph.i0 = R8(p);   p += 8;
    eph.idot = R8(p);   p += 8;
    eph.OMG0 = R8(p);   p += 8;
    eph.OMGd = R8(p);   p += 8;

    eph.iodc = U4(p); p += 4;
    toc = R8(p);   p += 8;

    eph.tgd[0] = R8(p);   p += 8; /* TGD1 for B1 (s) */
    eph.tgd[1] = R8(p);   p += 8; /* TGD2 for B2 (s) */

    eph.f0 = R8(p);   p += 8;
    eph.f1 = R8(p);   p += 8;
    eph.f2 = R8(p);   p += 8;

    int as = U4(p); p += 4;
    double N = R8(p);   p += 8;
    ura = R8(p);   p += 8;


    if (!(sat = satno(SYS_CMP, prn))) {
        trace(2, "unicore bdsephb satellite error: prn=%d\n", prn);
        return -1;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype + strlen(raw->msgtype), " prn=%d", prn);
    }
    eph.sat = sat;
    eph.sva = uraindex(ura);
    eph.toe = bdt2gpst(bdt2time(eph.week, eph.toes)); /* bdt -> gpst */
    eph.toc = bdt2gpst(bdt2time(eph.week, toc));      /* bdt -> gpst */
    eph.ttr = raw->time;

    if (!strstr(raw->opt, "-EPHALL")) {
        if (timediff(raw->nav.eph[sat - 1].toe, eph.toe) == 0.0 &&
            timediff(raw->nav.eph[sat - 1].toc, eph.toc) == 0.0) return 0;
    }
    raw->nav.eph[sat - 1] = eph;
    raw->ephsat = sat;
    raw->ephset = 0;
    return 2;
}

static int decode_qzssephb(raw_t* raw) {
    eph_t eph = { 0 };
    uint8_t* p = raw->buff + HLEN;

    double tow, tocs, N, URA;
    int prn, as, sat, week, zweek, health;
    int iode1, iode2;

    if (raw->len < HLEN + 224) {
        trace(2, "unicore qzssephemrisb length error: len=%d\n", raw->len);
        return -1;
    }

    prn = U4(p) + MINPRNQZS - 1;   p += 4;
    tow = R8(p); p += 8;
    health = U4(p) & 0x3f; p += 4;
    eph.iode = U4(p); p += 4;
    iode2 = U4(p); p += 4;

    eph.week = U4(p); p += 4;
    zweek = U4(p); p += 4;
    eph.toes = R8(p); p += 8;
    eph.A = R8(p); p += 8;
    eph.deln = R8(p); p += 8;
    eph.M0 = R8(p); p += 8;
    eph.e = R8(p); p += 8;
    eph.omg = R8(p); p += 8;
    eph.cuc = R8(p); p += 8;
    eph.cus = R8(p); p += 8;
    eph.crc = R8(p); p += 8;
    eph.crs = R8(p); p += 8;
    eph.cic = R8(p); p += 8;
    eph.cis = R8(p); p += 8;
    eph.i0 = R8(p); p += 8;
    eph.idot = R8(p); p += 8;
    eph.OMG0 = R8(p); p += 8;
    eph.OMGd = R8(p); p += 8;
    eph.iodc = U4(p); p += 4;
    tocs = R8(p); p += 8;
    eph.tgd[0] = R8(p); p += 8;
    eph.f0 = R8(p); p += 8;
    eph.f1 = R8(p); p += 8;
    eph.f2 = R8(p); p += 8;
    as = U4(p); p += 4;
    N = R8(p); p += 8;
    eph.sva = uraindex(R8(p)); p += 8;

    if (!(sat = satno(SYS_QZS, prn))) {
        trace(2, "unicore qzsseph satellite error: prn=%d\n", prn);
        return -1;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype + strlen(raw->msgtype), " prn=%d", prn);
    }

    eph.sat = sat;
    eph.code = 0;
    eph.iodc = eph.iode;
    tow = time2gpst(raw->time, &week);
    eph.week = week; /* gps-week = gal-week */
    eph.toe = gpst2time(eph.week, eph.toes);

    double tt = timediff(eph.toe, raw->time);
    if (tt < -302400.0) eph.week++;
    else if (tt > 302400.0) eph.week--;
    eph.toe = gpst2time(eph.week, eph.toes);
    eph.toc = adjweek(raw->time, tocs);
    eph.ttr = raw->time;

    if (!strstr(raw->opt, "-EPHALL")) {
        if (timediff(raw->nav.eph[sat - 1].toe, eph.toe) == 0.0 &&
            timediff(raw->nav.eph[sat - 1].toc, eph.toc) == 0.0) return 0;
    }

    raw->nav.eph[sat - 1] = eph;
    raw->ephsat = sat;
    raw->ephset = 0;
    return 2;
}

static int decode_irnssephb(raw_t* raw) {
    eph_t eph = { 0 };
    uint8_t* p = raw->buff + HLEN;
    int prn, sat, l5_health, s_health;
    double toc;

    if (raw->len < HLEN + 224) {
        trace(2, "unicore irnssephb length error: len=%d\n", raw->len);
        return -1;
    }

    prn = U4(p);   p += 4;
    double towc = R8(p); p += 8;
    l5_health = U4(p) & 1; p += 4;
    eph.iode = U4(p);   p += 4; /* IODEC */
    s_health = U4(p);   p += 4;
    eph.week = U4(p);   p += 4 + 4;
    eph.toes = R8(p);   p += 8;
    eph.A = R8(p);   p += 8;
    eph.deln = R8(p);   p += 8;
    eph.M0 = R8(p);   p += 8;
    eph.e = R8(p);   p += 8;
    eph.omg = R8(p);   p += 8;

    eph.cuc = R8(p);   p += 8;
    eph.cus = R8(p);   p += 8;
    eph.crc = R8(p);   p += 8;
    eph.crs = R8(p);   p += 8;
    eph.cic = R8(p);   p += 8;
    eph.cis = R8(p);   p += 8;

    eph.i0 = R8(p);   p += 8;
    eph.idot = R8(p);   p += 8;
    eph.OMG0 = R8(p);   p += 8;
    eph.OMGd = R8(p);   p += 8 + 4;

    toc = R8(p);   p += 8;
    eph.tgd[0] = R8(p);   p += 8; /* TGD */

    eph.f0 = R8(p);   p += 8;
    eph.f1 = R8(p);   p += 8;
    eph.f2 = R8(p);   p += 8;

    uint32_t flag = U4(p); p += 4;
    double N = R8(p); p += 8;
    eph.sva = uraindex(R8(p)); p += 8;

    if (toc != eph.toes) { /* toe and toc should be matched */
        trace(2, "unicore irnssephb toe and toc unmatch prn=%d\n", prn);
        return -1;
    }
    if (!(sat = satno(SYS_IRN, prn))) {
        trace(2, "unicore irnssephb satellite error: prn=%d\n", prn);
        return 0;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype + strlen(raw->msgtype), " prn=%d", prn);
    }

    eph.sat = sat;
    eph.svh = (l5_health << 1) | s_health;
    eph.iodc = eph.iode;
    //eph.week += 1024; /* irnss-week -> gps-week */
    eph.toe = gpst2time(eph.week, eph.toes);
    eph.toc = gpst2time(eph.week, toc);
    eph.ttr = raw->time;
    eph.tgd[1] = 0.0;

    if (!strstr(raw->opt, "-EPHALL")) {
        if (timediff(raw->nav.eph[sat - 1].toe, eph.toe) == 0.0 &&
            raw->nav.eph[sat - 1].iode == eph.iode) return 0; /* unchanged */
    }
    raw->nav.eph[sat - 1] = eph;
    raw->ephsat = sat;
    raw->ephset = 0;
    return 2;
}

// decode OBSVMB
static int decode_obsvmb(raw_t* raw)
{
    uint8_t* p = raw->buff + HLEN;
    char* q;
    double psr, adr, dop, snr, lockt, tt, freq, glo_bias = 0.0;
    int i, index, nobs, prn, sat, sys, code, idx, track, plock, clock, lli;
    int gfrq;

    if ((q = strstr(raw->opt, "-GLOBIAS="))) sscanf(q, "-GLOBIAS=%lf", &glo_bias);

    int rcvstds = 0;
    if (strstr(raw->opt, "-RCVSTDS")) rcvstds = 1;

    nobs = U4(p);
    if (nobs == 0)return 1;
    if (raw->len < HLEN + 4 + nobs * 40) {
        trace(2, "unicore obsvmb length error: len=%d nobs=%d\n", raw->len, nobs);
        return -1;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype + strlen(raw->msgtype), " nobs=%d", nobs);
    }

    p += 4; // Number of observation messages

    for (i = 0; i < nobs; i++, p += 40) {
        uint32_t trk_stat = U4(p + 36);

        if ((idx = decode_track_stat(trk_stat, &sys, &code, &plock, &clock)) < 0) {
            continue;
        }
        prn = U2(p + 2);


        if (sys == SYS_GLO) prn -= 37;
        if (sys == SYS_SBS && prn >= MINPRNQZS_S && prn <= MAXPRNQZS_S && code == CODE_L1C) {
            sys = SYS_QZS;
            prn += 10;
            code = CODE_L1Z; /* QZS L1S */
        }
        if (!(sat = satno(sys, prn))) {
            trace(3, "unicore obsvm satellite number error: sys=%d,prn=%d\n", sys, prn);
            continue;
        }
        //if (sys == SYS_GLO && !parity) continue;

        if ((idx = checkpri(raw->opt, sys, code, idx)) < 0) continue;

        gfrq = U2(p) + 1; /* GLONASS FCN+8 */
        psr = R8(p + 4);
        adr = R8(p + 12);
        dop = R4(p + 24);
        snr = U2(p + 28) / 100.0;
        lockt = R4(p + 32);

        if (sys == SYS_GLO) {
            freq = sat2freq(sat, (uint8_t)code, &raw->nav);
            adr -= glo_bias * freq / CLIGHT;
            if (!raw->nav.glo_fcn[prn - 1]) {
                raw->nav.glo_fcn[prn - 1] = gfrq; /* fcn+8 */
            }
        }
        if (raw->tobs[sat - 1][idx].time != 0) {
            tt = timediff(raw->time, raw->tobs[sat - 1][idx]);
            lli = lockt - raw->lockt[sat - 1][idx] + 0.05 <= tt ? LLI_SLIP : 0;
        }
        else {
            lli = 0;
        }
        //      if (!parity) lli |= LLI_HALFC;
        //      if (halfc) lli |= LLI_HALFA;
        raw->tobs[sat - 1][idx] = raw->time;
        raw->lockt[sat - 1][idx] = lockt;
        raw->halfc[sat - 1][idx] = 0;

        if (!clock) psr = 0.0;     /* code unlock */
        if (!plock) adr = dop = 0.0; /* phase unlock */

        if (fabs(timediff(raw->obs.data[0].time, raw->time)) > 1E-9) {
            raw->obs.n = 0;
        }
        if ((index = obsindex(&raw->obs, raw->time, sat)) >= 0) {
            raw->obs.data[index].L[idx] = -adr;
            raw->obs.data[index].P[idx] = psr;
            raw->obs.data[index].D[idx] = (float)dop;
            raw->obs.data[index].SNR[idx] = (uint16_t)(snr / SNR_UNIT + 0.5);
            raw->obs.data[index].LLI[idx] = (uint8_t)lli;
            raw->obs.data[index].code[idx] = (uint8_t)code;
            if (rcvstds) {
                double pstd = U2(p + 20) * 0.01;  // Meters
                // To RTKlib encoding
                pstd = log2(pstd / 0.01) - 5;
                pstd = pstd > 0 ? pstd : 0;
                // Further limited to 9 in RINEX output
                pstd = pstd <= 254 ? pstd : 254;
                raw->obuf.data[index].Pstd[idx] = pstd + 0.5;

                double lstd = U2(p + 22) * 0.0001; // Cycles
                // To RTKlib encoding
                lstd = lstd / 0.004;
                lstd = lstd > 0 ? lstd : 0;
                // Further limited to 9 in RINEX output
                lstd = lstd <= 254 ? lstd : 254;
                raw->obuf.data[index].Lstd[idx] = lstd + 0.5;
            }
        }
    }
    return 1;
}


/* decode Unicore message -----------------------------------------*/
static int decode_unicore(raw_t* raw)
{
    double tow;
    char tstr[32];
    int stat, week, type = U2(raw->buff + 4);

    trace(3, "decode_unicore: type=%3d len=%d\n", type, raw->len);

    /* check crc32 */
    if (rtk_crc32(raw->buff, raw->len) != U4(raw->buff + raw->len)) {
        trace(2, "unicore crc error: type=%3d len=%d\n", type, raw->len);
        return -1;
    }
    stat = U1(raw->buff + 9);
    week = U2(raw->buff + 10);

    if (stat == 201 || week == 0) {
        trace(3, "unicore time error: type=%3d stat=%d week=%d\n", type,
            stat, week);
        return 0;
    }
    week = adjgpsweek(week);
    tow = U4(raw->buff + 12) * 0.001;
    raw->time = gpst2time(week, tow);
    double ep[6];
    time2epoch_n(raw->time, ep, 7);


    if (raw->outtype) {
        time2str(gpst2time(week, tow), tstr, 2);
        sprintf(raw->msgtype, "UNICORE %4d (%4d): %s", type, raw->len, tstr);
    }

    switch (type) {
    case ID_OBSVM: return decode_obsvmb(raw);
    case ID_GPSEPH: return decode_gpsephb(raw);
    case ID_GLOEPH: return decode_gloephb(raw);
    case ID_GALEPH: return decode_galephb(raw);
    case ID_BDSEPH: return decode_bdsephb(raw);
    case ID_QZSSEPH: return decode_qzssephb(raw);
    case ID_IRNSSEPH: return decode_irnssephb(raw);
    }
    return 0;
}


/* input Unicore raw data from stream -------------------------------
* fetch next Unicore raw data and input a mesasge from stream
* args   : raw_t *raw       IO  receiver raw data control struct
*          uint8_t data     I   stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 3: input sbas message,
*                  9: input ion/utc parameter)
*/
extern int input_unicore(raw_t* raw, uint8_t data)
{
    trace(5, "input_unicore: data=%02x\n", data);

    /* synchronize frame */
    if (raw->nbyte == 0) {
        if (sync_unicore(raw->buff, data)) raw->nbyte = 3;
        return 0;
    }
    raw->buff[raw->nbyte++] = data;

    if (raw->nbyte == 10 && (raw->len = U2(raw->buff + 8) + HLEN) > MAXRAWLEN - 4) {
        trace(2, "unicore length error: len=%d\n", raw->len);
        raw->nbyte = 0;
        return -1;
    }
    if (raw->nbyte < 10 || raw->nbyte < raw->len + 4) return 0;
    raw->nbyte = 0;

    return decode_unicore(raw);
}


/* input Unicore raw data from file ---------------------------------
* fetch next Unicore raw data and input a message from file
* args   : raw_t  *raw      IO  receiver raw data control struct
*          FILE   *fp       I   file pointer
* return : status(-2: end of file, -1...9: same as above)
*-----------------------------------------------------------------------------*/
extern int input_unicoref(raw_t* raw, FILE* fp)
{
    int i, data;

    trace(4, "input_unicoref:\n");

    /* synchronize frame */
    if (raw->nbyte == 0) {
        for (i = 0;; i++) {
            if ((data = fgetc(fp)) == EOF) return -2;
            if (sync_unicore(raw->buff, (uint8_t)data)) break;
            if (i >= 4096) return 0;
        }
    }
    if (fread(raw->buff + 3, 7, 1, fp) < 1) return -2;
    raw->nbyte = 10;

    if ((raw->len = U2(raw->buff + 6) + HLEN) > MAXRAWLEN - 4) {
        trace(2, "unicore length error: len=%d\n", raw->len);
        raw->nbyte = 0;
        return -1;
    }
    if (fread(raw->buff + 10, raw->len - 6, 1, fp) < 1) return -2;
    raw->nbyte = 0;

    return decode_unicore(raw);
}
