/*------------------------------------------------------------------------------
* ublox.c : ublox receiver dependent functions
*
*          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
*          Copyright (C) 2014 by T.SUZUKI, All rights reserved.
*
* reference :
*     [1] ublox-AG, GPS.G3-X-03002-D, ANTARIS Positioning Engine NMEA and UBX
*         Protocol Specification, Version 5.00, 2003
*     [2] ublox-AG, UBX-13003221-R03, u-blox M8 Receiver Description including
*         Protocol Specification V5, Dec 20, 2013
*     [3] ublox-AG, UBX-13003221-R07, u-blox M8 Receiver Description including
*         Protocol Specification V15.00-17.00, Nov 3, 2014
*     [4] ublox-AG, UBX-13003221-R09, u-blox 8 /u-blox M8 Receiver Description
*         including Protocol Specification V15.00-18.00, January, 2016
*     [5] ublox-AG, UBX-18010854-R08, u-blox ZED-F9P Interface Description,
*         May, 2020
*
* version : $Revision: 1.2 $ $Date: 2008/07/14 00:05:05 $
* history : 2007/10/08 1.0  new
*           2008/06/16 1.1  separate common functions to rcvcmn.c
*           2009/04/01 1.2  add range check of prn number
*           2009/04/10 1.3  refactored
*           2009/09/25 1.4  add function gen_ubx()
*           2010/01/17 1.5  add time tag adjustment option -tadj sec
*           2010/10/31 1.6  fix bug on playback disabled for raw data (2.4.0_p9)
*           2011/05/27 1.7  add almanac decoding
*                           add -EPHALL option
*                           fix problem with ARM compiler
*           2013/02/23 1.8  fix memory access violation problem on arm
*                           change options -tadj to -TADJ, -invcp to -INVCP
*           2014/05/26 1.9  fix bug on message size of CFG-MSG
*                           fix bug on return code of decode_alm1()
*           2014/06/21 1.10 support message TRK-MEAS and TRK-SFRBX
*                           support message NAV-SOL and NAV-TIMEGPS to get time
*                           support message GFG-GNSS generation
*           2014/06/23 1.11 support message TRK-MEAS for beidou ephemeris
*           2014/08/11 1.12 fix bug on unable to read RXM-RAW
*                           fix problem on decoding glo ephemeris in TRK-SFRBX
*                           support message TRK-TRKD5
*           2014/08/31 1.13 suppress warning
*           2014/11/04 1.14 support message RXM-RAWX and RXM-SFRBX
*           2015/03/20 1.15 omit time adjustment for RXM-RAWX
*           2016/01/22 1.16 add time-tag in raw-message-type
*           2016/01/26 1.17 support galileo navigation data in RXM-SFRBX
*                           enable option -TADJ for RXM-RAWX
*           2016/05/25 1.18 fix bug on crc-buffer-overflow by decoding galileo
*                           navigation data
*           2016/07/04 1.19 add half-cycle vaild check for ubx-trk-meas
*           2016/07/29 1.20 support RXM-CFG-TMODE3 (0x06 0x71) for M8P
*                           crc24q() -> rtk_crc24q()
*                           check week number zero for ubx-rxm-raw and rawx
*           2016/08/20 1.21 add test of std-dev for carrier-phase valid
*           2016/08/26 1.22 add option -STD_SLIP to test slip by std-dev of cp
*                           fix on half-cyc valid for sbas in trkmeas
*           2017/04/11 1.23 (char *) -> (signed char *)
*                           fix bug on week handover in decode_trkmeas/trkd5()
*                           fix bug on prn for geo in decode_cnav()
*           2017/06/10 1.24 output half-cycle-subtracted flag
*           2018/10/09 1.25 support ZED-F9P according to [5]
*                           beidou C17 is handled as GEO (navigation D2).
*           2018/11/05 1.26 fix problem on missing QZSS L2C signal
*                           save signal in obs data by signal index
*                           suppress warning for cnav in ubx-rxm-sfrbx
*           2019/05/10 1.27 disable half-cyc-subtract flag on LLI for RXM-RAWX
*                           save galileo E5b data to obs index 2
*                           handle C17 as no-GEO (MEO/IGSO)
*           2020/11/30 1.28 update reference [5]
*                           support UBX-CFG-VALDEL,VALGET,VALSET
*                           support hex field format for ubx binary message
*                           add quality test for receiver time in decode_trkd5()
*                           add half cycle shift correction for BDS GEO
*                           delete receiver option -GALFNAV
*                           use API code2idx() and code2freq()
*                           support QZSS L1S (CODE_L1Z)
*                           CODE_L1I -> CODE_L2I for BDS B1I (RINEX 3.04)
*                           use integer types in stdint.h
*-----------------------------------------------------------------------------*/
#include "rtklib.h"

#define UBXSYNC1    0xB5        /* ubx message sync code 1 */
#define UBXSYNC2    0x62        /* ubx message sync code 2 */
#define UBXCFG      0x06        /* ubx message cfg-??? */

#define PREAMB_CNAV 0x8B        /* cnav preamble */

#define ID_NAVSOL   0x0106      /* ubx message id: nav solution info */
#define ID_NAVTIME  0x0120      /* ubx message id: nav time gps */
#define ID_RXMRAW   0x0210      /* ubx message id: raw measurement data */
#define ID_RXMSFRB  0x0211      /* ubx message id: subframe buffer */
#define ID_RXMSFRBX 0x0213      /* ubx message id: raw subframe data */
#define ID_RXMRAWX  0x0215      /* ubx message id: multi-gnss raw meas data */
#define ID_TRKD5    0x030A      /* ubx message id: trace mesurement data */
#define ID_TRKMEAS  0x0310      /* ubx message id: trace mesurement data */
#define ID_TRKSFRBX 0x030F      /* ubx message id: trace subframe buffer */
#define ID_TIMTM2   0x0D03      /* ubx message id: time mark data */

#define FU1         1           /* ubx message field types */
#define FU2         2
#define FU4         3
#define FU8         4
#define FI1         5
#define FI2         6
#define FI4         7
#define FR4         8
#define FR8         9
#define FS32        10

typedef enum { false, true } bool;

#define P2_10       0.0009765625 /* 2^-10 */

/* max std-dev for valid carrier-phases */
#define MAX_CPSTD_VALID_GEN8 5       /* optimal value for Gen8 modules  */
#define MAX_CPSTD_VALID_GEN9 14      /* optimal value for Gen9 modules  */
#define CPSTD_SLIP 15                /* std-dev threshold for slip */

#define ROUND(x)    (int)floor((x)+0.5)

/* get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((uint8_t *)(p)))
#define I1(p) (*((int8_t  *)(p)))
static uint16_t U2(uint8_t *p) {uint16_t u; memcpy(&u,p,2); return u;}
static uint32_t U4(uint8_t *p) {uint32_t u; memcpy(&u,p,4); return u;}
static int32_t  I4(uint8_t *p) {int32_t  u; memcpy(&u,p,4); return u;}
static float    R4(uint8_t *p) {float    r; memcpy(&r,p,4); return r;}
static double   R8(uint8_t *p) {double   r; memcpy(&r,p,8); return r;}
static double   I8(uint8_t *p) {return I4(p+4)*4294967296.0+U4(p);}

/* set fields (little-endian) ------------------------------------------------*/
static void setU1(uint8_t *p, uint8_t  u) {*p=u;}
static void setU2(uint8_t *p, uint16_t u) {memcpy(p,&u,2);}
static void setU4(uint8_t *p, uint32_t u) {memcpy(p,&u,4);}
static void setI1(uint8_t *p, int8_t   i) {*p=(uint8_t)i;}
static void setI2(uint8_t *p, int16_t  i) {memcpy(p,&i,2);}
static void setI4(uint8_t *p, int32_t  i) {memcpy(p,&i,4);}
static void setR4(uint8_t *p, float    r) {memcpy(p,&r,4);}
static void setR8(uint8_t *p, double   r) {memcpy(p,&r,8);}

/* checksum ------------------------------------------------------------------*/
static int checksum(uint8_t *buff, int len)
{
    uint8_t cka=0,ckb=0;
    int i;
    
    for (i=2;i<len-2;i++) {
        cka+=buff[i]; ckb+=cka;
    }
    return cka==buff[len-2]&&ckb==buff[len-1];
}
static void setcs(uint8_t *buff, int len)
{
    uint8_t cka=0,ckb=0;
    int i;
    
    for (i=2;i<len-2;i++) {
        cka+=buff[i]; ckb+=cka;
    }
    buff[len-2]=cka;
    buff[len-1]=ckb;
}
/* UBX GNSSId to system (ref [2] 25) -----------------------------------------*/
static int ubx_sys(int gnssid)
{
    switch (gnssid) {
        case 0: return SYS_GPS;
        case 1: return SYS_SBS;
        case 2: return SYS_GAL;
        case 3: return SYS_CMP;
        case 5: return SYS_QZS;
        case 6: return SYS_GLO;
    }
    return 0;
}
/* UBX SigId to signal (ref [5] 1.5.4) ---------------------------------------*/
static int ubx_sig(int sys, int sigid)
{
    if (sys == SYS_GPS) {
        if (sigid == 0) return CODE_L1C; /* L1C/A */
        if (sigid==3) return CODE_L2L; /* L2CL */
        if (sigid==4) return CODE_L2S; /* L2CM */
        if (sigid==6) return CODE_L5I; /* L5I */
        if (sigid==7) return CODE_L5Q; /* L5Q */
    }
    else if (sys == SYS_GLO) {
        if (sigid == 0) return CODE_L1C; /* G1C/A (GLO L1 OF) */
        if (sigid == 2) return CODE_L2C; /* G2C/A (GLO L2 OF) */
    }
    else if (sys == SYS_GAL) {
        if (sigid==0) return CODE_L1C; /* E1C */
        if (sigid==1) return CODE_L1B; /* E1B */
        if (sigid==3) return CODE_L5I; /* E5aI */
        if (sigid==4) return CODE_L5Q; /* E5aQ */
        if (sigid==5) return CODE_L7I; /* E5bI */
        if (sigid==6) return CODE_L7Q; /* E5bQ */
    }
    else if (sys == SYS_QZS) {
        if (sigid==0) return CODE_L1C; /* L1C/A */
        if (sigid==1) return CODE_L1Z; /* L1S */
        if (sigid==4) return CODE_L2S; /* L2CM */
        if (sigid==5) return CODE_L2L; /* L2CL */
        if (sigid==8) return CODE_L5I; /* L5I */
        if (sigid==9) return CODE_L5Q; /* L5Q */
    }
    else if (sys == SYS_CMP) {
        if (sigid==0) return CODE_L2I; /* B1I D1 */
        if (sigid==1) return CODE_L2I; /* B1I D2 */
        if (sigid == 2) return CODE_L7I; /* B2I D1 */
        if (sigid == 3) return CODE_L7I; /* B2I D2 */
        if (sigid == 7) return CODE_L5X; /* B2a */
    }
    else if (sys == SYS_SBS) {
        if (sigid==0) return CODE_L1C; /* L1C/A */
    }
    return CODE_NONE;
}
/* UBX SigId to signal - combine codes ------------------------*/
static int ubx_sig_combined(int sys, int sigid)
{
    if (sys == SYS_GPS) {
        if (sigid == 0) return CODE_L1C; /* L1C/A */
        if (sigid==3) return CODE_L2X; /* L2CL */
        if (sigid==4) return CODE_L2X; /* L2CM */
        if (sigid==6) return CODE_L5X; /* L5I */
        if (sigid==7) return CODE_L5X; /* L5Q */
    }
    else if (sys == SYS_GLO) {
        if (sigid == 0) return CODE_L1C; /* G1C/A (GLO L1 OF) */
        if (sigid == 2) return CODE_L2C; /* G2C/A (GLO L2 OF) */
    }
    else if (sys == SYS_GAL) {
        if (sigid==0) return CODE_L1X; /* E1C */
        if (sigid==1) return CODE_L1X; /* E1B */
        if (sigid==3) return CODE_L5X; /* E5aI */
        if (sigid==4) return CODE_L5X; /* E5aQ */
        if (sigid==5) return CODE_L7X; /* E5bI */
        if (sigid==6) return CODE_L7X; /* E5bQ */
        }
    else if (sys == SYS_QZS) {
        if (sigid == 0) return CODE_L1C; /* L1C/A */
        if (sigid==1) return CODE_L1C; /* L1S */
        if (sigid==4) return CODE_L2X; /* L2CM */
        if (sigid==5) return CODE_L2X; /* L2CL */
        if (sigid==8) return CODE_L5X; /* L5I */
        if (sigid==9) return CODE_L5X; /* L5Q */
    }
    else if (sys == SYS_CMP) {
        if (sigid==0) return CODE_L2I; /* B1I D1 */
        if (sigid==1) return CODE_L2I; /* B1I D2 */
        if (sigid == 2) return CODE_L7I; /* B2I D1 */
        if (sigid == 3) return CODE_L7I; /* B2I D2 */
        if (sigid == 7) return CODE_L5X; /* B2a */
    }
    else if (sys == SYS_SBS) {
        if (sigid==0) return CODE_L1C; /* L1C/A */
    }
    return CODE_NONE;
}
/* signal index in obs data --------------------------------------------------*/
static int sig_idx(int sys, uint8_t code)
{
    int idx=code2idx(sys,code),nex=NEXOBS;
    
    if (sys == SYS_GPS) {
        if (code==CODE_L2S) return (nex<1)?-1:NFREQ;   /* L2CM */
    }
    else if (sys == SYS_GAL) {
        if (code==CODE_L1B) return (nex<1)?-1:NFREQ;   /* E1B */
        if (code==CODE_L7I) return (nex<2)?-1:NFREQ+1; /* E5bI */
    }
    else if (sys == SYS_QZS) {
        if (code==CODE_L2S) return (nex<1)?-1:NFREQ;   /* L2CM */
        if (code==CODE_L1Z) return (nex<2)?-1:NFREQ+1; /* L1S */
    }
    return (idx<NFREQ)?idx:-1;
}
/* decode UBX-RXM-RAW: raw measurement data ----------------------------------*/
static int decode_rxmraw(raw_t *raw)
{
    uint8_t *p=raw->buff+6;
    gtime_t time;
    double tow,tt,tadj=0.0,toff=0.0,tn;
    int i,j,prn,sat,n=0,nsat,week;
    char *q;
    
    trace(4,"decode_rxmraw: len=%d\n",raw->len);
    
    if (raw->outtype) {
        sprintf(raw->msgtype,"UBX RXM-RAW   (%4d): nsat=%d",raw->len,U1(p+6));
    }
    /* time tag adjustment option (-TADJ) */
    if ((q=strstr(raw->opt,"-TADJ="))) {
        sscanf(q,"-TADJ=%lf",&tadj);
    }
    nsat=U1(p+6);
    if (raw->len<12+24*nsat) {
        trace(2,"ubx rxmraw length error: len=%d nsat=%d\n",raw->len,nsat);
        return -1;
    }
    tow =U4(p  );
    week=U2(p+4);
    time=gpst2time(week,tow*0.001);
    
    if (week==0) {
        trace(3,"ubx rxmraw week=0 error: len=%d nsat=%d\n",raw->len,nsat);
        return 0;
    }
    /* time tag adjustment */
    if (tadj>0.0) {
        tn=time2gpst(time,&week)/tadj;
        toff=(tn-floor(tn+0.5))*tadj;
        time=timeadd(time,-toff);
    }
    tt=timediff(time,raw->time);
    
    for (i=0,p+=8;i<nsat&&i<MAXOBS;i++,p+=24) {
        raw->obs.data[n].time=time;
        raw->obs.data[n].L[0]  =R8(p   )-toff*FREQL1;
        raw->obs.data[n].P[0]  =R8(p+ 8)-toff*CLIGHT;
        raw->obs.data[n].D[0]  =R4(p+16);
        prn                    =U1(p+20);
        raw->obs.data[n].SNR[0]=(uint16_t)(I1(p+22)*1.0/SNR_UNIT+0.5);
        raw->obs.data[n].LLI[0]=U1(p+23);
        raw->obs.data[n].code[0]=CODE_L1C;
        
        /* phase polarity flip option (-INVCP) */
        if (strstr(raw->opt,"-INVCP")) {
            raw->obs.data[n].L[0]=-raw->obs.data[n].L[0];
        }
        if (!(sat=satno(MINPRNSBS<=prn?SYS_SBS:SYS_GPS,prn))) {
            trace(2,"ubx rxmraw sat number error: prn=%d\n",prn);
            continue;
        }
        raw->obs.data[n].sat=sat;
        
        if (raw->obs.data[n].LLI[0]&1) raw->lockt[sat-1][0]=0.0;
        else if (tt<1.0||10.0<tt) raw->lockt[sat-1][0]=0.0;
        else raw->lockt[sat-1][0]+=tt;
        
        for (j=1;j<NFREQ+NEXOBS;j++) {
            raw->obs.data[n].L[j]=raw->obs.data[n].P[j]=0.0;
            raw->obs.data[n].D[j]=0.0;
            raw->obs.data[n].SNR[j]=raw->obs.data[n].LLI[j]=0;
            raw->obs.data[n].Lstd[j]=raw->obs.data[n].Pstd[j]=0;
            raw->obs.data[n].code[j]=CODE_NONE;
        }
        n++;
    }
    raw->time=time;
    raw->obs.n=n;
    return 1;
}
/* decode UBX-RXM-RAWX: multi-GNSS raw measurement data (ref [3][4][5]) ------*/
static int decode_rxmrawx(raw_t *raw)
{
    uint8_t *p=raw->buff+6;
    gtime_t time;
    char *q,tstr[64];
    double tow,P,L,D,tn,tadj=0.0,toff=0.0;
    int i,j,k,idx,sys,prn,sat,code,slip,halfv,halfc,LLI,n=0,cpstd_valid,cpstd_slip;
    int week,nmeas,ver,gnss,svid,sigid,frqid,lockt,cn0,cpstd=0,prstd=0,tstat;
    int multicode=0, rcvstds=0;

    trace(4,"decode_rxmrawx: len=%d\n",raw->len);
    
    if (raw->len<24) {
        trace(2,"ubx rxmrawx length error: len=%d\n",raw->len);
        return -1;
    }
    tow  =R8(p   ); /* rcvTow (s) */
    week =U2(p+ 8); /* week */
    nmeas=U1(p+11); /* numMeas */
    ver  =U1(p+13); /* version ([5] 5.15.3.1) */
    
    if (raw->len<24+32*nmeas) {
        trace(2,"ubx rxmrawx length error: len=%d nmeas=%d\n",raw->len,nmeas);
        return -1;
    }
    if (week==0) {
        trace(3,"ubx rxmrawx week=0 error: len=%d nmeas=%d\n",raw->len,nmeas);
        return 0;
    }
    time=gpst2time(week,tow);
    
    if (raw->outtype) {
        time2str(time,tstr,2);
        sprintf(raw->msgtype,"UBX RXM-RAWX  (%4d): time=%s nmeas=%d ver=%d",
                raw->len,tstr,nmeas,ver);
    }
    /* time tag adjustment option (-TADJ) */
    if ((q=strstr(raw->opt,"-TADJ="))) {
        sscanf(q,"-TADJ=%lf",&tadj);
    }
    /* max valid std-dev of carrier-phase (-MAX_STD_CP) */
    if ((q=strstr(raw->opt,"-MAX_STD_CP="))) {
        sscanf(q,"-MAX_STD_CP=%d",&cpstd_valid);
    }
    else if (raw->rcvtype==1) cpstd_valid=MAX_CPSTD_VALID_GEN9;  /* F9P */
    else cpstd_valid=MAX_CPSTD_VALID_GEN8;  /* M8T, M8P */

    /* slip threshold of std-dev of carrier-phase (-STD_SLIP) */
    if ((q=strstr(raw->opt,"-STD_SLIP="))) {
        sscanf(q,"-STD_SLIP=%d",&cpstd_slip);
    } else cpstd_slip=CPSTD_SLIP;
    /* use multiple codes for each freq (-MULTICODE) */
    if ((q=strstr(raw->opt,"-MULTICODE"))) multicode=1;
    /* write rcvr stdevs to unused rinex fields */
    if ((q=strstr(raw->opt,"-RCVSTDS"))) rcvstds=1;

    /* time tag adjustment */
    if (tadj>0.0) {
        tn=time2gpst(time,&week)/tadj;
        toff=(tn-floor(tn+0.5))*tadj;
        time=timeadd(time,-toff);
    }
    for (i=0,p+=16;i<nmeas&&n<MAXOBS;i++,p+=32) {
        P    =R8(p   );    /* prMes (m) */
        L    =R8(p+ 8);    /* cpMes (cyc) */
        D    =R4(p+16);    /* doMes (hz) */
        gnss =U1(p+20);    /* gnssId */
        svid =U1(p+21);    /* svId */
        sigid=U1(p+22);    /* sigId ([5] 5.15.3.1) */
        frqid=U1(p+23);    /* freqId (fcn + 7) */
        lockt=U2(p+24);    /* locktime (ms) */
        cn0  =U1(p+26);    /* cn0 (dBHz) */
        prstd=U1(p+27)&15; /* pseudorange std-dev: (0.01*2^n meters) */
        cpstd=U1(p+28)&15; /* cpStdev (n*0.004 m) */
        /* subtract offset to use valid rinex format range (0->9) */
        prstd=prstd>=5?prstd-5:0; /* prstd=0.01*2^(x-5) meters*/

        tstat=U1(p+30);    /* trkStat */
        if (!(tstat&1)) P=0.0;
        if (!(tstat&2)||L==-0.5||cpstd>cpstd_valid) L=0.0; /* invalid phase */
        if (sigid>1) raw->rcvtype=1;  /* flag as Gen9 receiver */

        if (!(sys=ubx_sys(gnss))) {
            trace(2,"ubx rxmrawx: system error gnss=%d\n", gnss);
            continue;
        }
        prn=svid+(sys==SYS_QZS?192:0);
        if (!(sat=satno(sys,prn))) {
            if (sys==SYS_GLO&&prn==255) {
                continue; /* suppress warning for unknown glo satellite */
            }
            trace(2,"ubx rxmrawx sat number error: sys=%2d prn=%2d\n",sys,prn);
            continue;
        }
        if (sys==SYS_GLO&&!raw->nav.glo_fcn[prn-1]) {
            raw->nav.glo_fcn[prn-1]=frqid-7+8;
        }
        if (ver>=1) {
            if (multicode)
                code=ubx_sig(sys,sigid);
            else
                code=ubx_sig_combined(sys,sigid);
        }
        else {
            code=(sys==SYS_CMP)?CODE_L2I:((sys==SYS_GAL)?CODE_L1X:CODE_L1C);
        }
        /* signal index in obs data */
        if ((idx=sig_idx(sys,code))<0) {
            trace(2,"ubx rxmrawx signal error: sat=%2d sigid=%d\n",sat,sigid);
            continue;
        }
        /* offset by time tag adjustment */
        if (toff!=0.0) {
            P-=P!=0.0?toff*CLIGHT:0.0;
            L-=L!=0.0?toff*code2freq(sys,code,frqid-7):0.0;
        }
        /* half-cycle shift correction for BDS GEO */
        if (sys==SYS_CMP&&(prn<=5||prn>=59)&&L!=0.0) {
            L+=0.5;
        }
        if (sys==SYS_SBS)
           halfv=lockt>8000?1:0; /* half-cycle valid */
        else
            halfv=(tstat&4)?1:0; /* half cycle valid */
        halfc=(tstat&8)?1:0; /* half cycle subtracted from phase */
        slip=lockt==0||lockt*1E-3<raw->lockt[sat-1][idx]||
             halfc!=raw->halfc[sat-1][idx];
        if (cpstd>=cpstd_slip) slip=LLI_SLIP;
        if (slip) raw->lockflag[sat-1][idx]=slip;
        raw->lockt[sat-1][idx]=lockt*1E-3;
        /* LLI: bit0=slip,bit1=half-cycle-unresolved */
        LLI=!halfv&&L!=0.0?LLI_HALFC:0;
        /* half cycle adjusted */
        LLI|=halfc?LLI_HALFA:0; 
        /* set cycle slip if half cycle subtract bit changed state */
        LLI|=halfc!=raw->halfc[sat-1][idx]?LLI_SLIP:0;
        raw->halfc[sat-1][idx]=halfc;
        /* set cycle slip flag if first valid phase since slip */
        if (L!=0.0) LLI|=raw->lockflag[sat-1][idx]>0.0?LLI_SLIP:0;

        for (j=0;j<n;j++) {
            if (raw->obs.data[j].sat==sat) break;
        }
        if (j>=n) {
            raw->obs.data[n].time=time;
            raw->obs.data[n].sat=sat;
            raw->obs.data[n].rcv=0;
            for (k=0;k<NFREQ+NEXOBS;k++) {
                raw->obs.data[n].L[k]=raw->obs.data[n].P[k]=0.0;
                raw->obs.data[n].Lstd[k]=raw->obs.data[n].Pstd[k]=0;
                raw->obs.data[n].D[k]=0.0;
                raw->obs.data[n].SNR[k]=raw->obs.data[n].LLI[k]=0;
                raw->obs.data[n].code[k]=CODE_NONE;
            }
            n++;
        }
        prstd=prstd<=9?prstd:9;  /* limit to 9 to fit RINEX format */
        cpstd=cpstd<=9?cpstd:9;  /* limit to 9 to fit RINEX format */
        raw->obs.data[j].L[idx]=L;
        raw->obs.data[j].P[idx]=P;
        raw->obs.data[j].Lstd[idx]=rcvstds?cpstd:0;
        raw->obs.data[j].Pstd[idx]=rcvstds?prstd:0;
        raw->obs.data[j].D[idx]=(float)D;
        raw->obs.data[j].SNR[idx]=(uint16_t)(cn0*1.0/SNR_UNIT+0.5);
        raw->obs.data[j].LLI[idx]=(uint8_t)LLI;
        raw->obs.data[j].code[idx]=(uint8_t)code;
        if (L!=0.0) raw->lockflag[sat-1][idx]=0; /* clear slip carry-forward flag if valid phase*/
    }
    raw->time=time;
    raw->obs.n=n;
    return 1;
}
/* decode UBX-NAV-SOL: navigation solution -----------------------------------*/
static int decode_navsol(raw_t *raw)
{
    uint8_t *p=raw->buff+6;
    int itow,ftow,week;
    
    trace(4,"decode_navsol: len=%d\n",raw->len);
    
    if (raw->outtype) {
        sprintf(raw->msgtype,"UBX NAV-SOL   (%4d):",raw->len);
    }
    itow=U4(p);
    ftow=I4(p+4);
    week=U2(p+8);
    if ((U1(p+11)&0x0C)==0x0C) {
        raw->time=gpst2time(week,itow*1E-3+ftow*1E-9);
    }
    return 0;
}
/* decode UBX-NAV-TIMEGPS: GPS time solution ---------------------------------*/
static int decode_navtime(raw_t *raw)
{
    int itow,ftow,week;
    uint8_t *p=raw->buff+6;
    
    trace(4,"decode_navtime: len=%d\n",raw->len);
    
    if (raw->outtype) {
        sprintf(raw->msgtype,"UBX NAV-TIME  (%4d):",raw->len);
    }
    itow=U4(p);
    ftow=I4(p+4);
    week=U2(p+8);
    if ((U1(p+11)&0x03)==0x03) {
        raw->time=gpst2time(week,itow*1E-3+ftow*1E-9);
    }
    return 0;
}
/* decode UBX-TRK-MEAS: trace measurement data (unofficial) ------------------*/
static int decode_trkmeas(raw_t *raw)
{
    static double adrs[MAXSAT]={0};
    uint8_t *p=raw->buff+6;
    gtime_t time;
    double ts,tr=-1.0,t,tau,utc_gpst,snr,adr,dop;
    int i,j,n=0,nch,sys,prn,sat,qi,frq,flag,lock1,lock2,week,fw=0;
    char *q;
    /* adjustment to code measurement in meters, based on GLONASS freq,
       values based on difference between TRK_MEAS values and  RXM-RAWX values */
    const char P_adj_fw2[]={ 0, 0, 0, 0, 1, 3, 2, 0,-4,-3,-9,-8,-7,-4, 0};  /* fw 2.30 */
    const char P_adj_fw3[]={11,13,13,14,14,13,12,10, 8, 6, 5, 5, 5, 7, 0};  /* fw 3.01 */
    
    trace(4,"decode_trkmeas: len=%d\n",raw->len);
    
    if (raw->outtype) {
        sprintf(raw->msgtype,"UBX TRK-MEAS  (%4d):",raw->len);
    }
    if (!raw->time.time) return 0;

    /* trk meas code adjust (-TRKM_ADJ) */
    if ((q=strstr(raw->opt,"-TRKM_ADJ="))) {
        sscanf(q,"-TRKM_ADJ=%d",&fw);
    }
    
    /* number of channels */
    nch=U1(p+2);
    
    if (raw->len<112+nch*56) {
        trace(2,"decode_trkmeas: length error len=%d nch=%2d\n",raw->len,nch);
        return -1;
    }
    /* time-tag = max(transmission time + 0.08) rounded by 100 ms */
    for (i=0,p=raw->buff+110;i<nch;i++,p+=56) {
        if (U1(p+1)<4||ubx_sys(U1(p+4))!=SYS_GPS) continue;
        if ((t=I8(p+24)*P2_32/1000.0)>tr) tr=t;
    }
    if (tr<0.0) return 0;
    
    tr=ROUND((tr+0.08)/0.1)*0.1;
    
    /* adjust week handover */
    t=time2gpst(raw->time,&week);
    if      (tr<t-302400.0) week++;
    else if (tr>t+302400.0) week--;
    time=gpst2time(week,tr);
    
    utc_gpst=timediff(gpst2utc(time),time);
    
    for (i=0,p=raw->buff+110;i<nch;i++,p+=56) {
        
        /* quality indicator (0:idle,1:search,2:aquired,3:unusable, */
        /*                    4:code lock,5,6,7:code/carrier lock) */
        qi=U1(p+1);
        if (qi<4||7<qi) continue;
        
        /* system and satellite number */
        if (!(sys=ubx_sys(U1(p+4)))) {
            trace(2,"ubx trkmeas: system error\n");
            continue;
        }
        prn=U1(p+5)+(sys==SYS_QZS?192:0);
        if (!(sat=satno(sys,prn))) {
            trace(2,"ubx trkmeas sat number error: sys=%2d prn=%2d\n",sys,prn);
            continue;
        }
        /* transmission time */
        ts=I8(p+24)*P2_32/1000.0;
        if      (sys==SYS_CMP) ts+=14.0;             /* bdt  -> gpst */
        else if (sys==SYS_GLO) ts-=10800.0+utc_gpst; /* glot -> gpst */
        
        /* signal travel time */
        tau=tr-ts;
        if      (tau<-302400.0) tau+=604800.0;
        else if (tau> 302400.0) tau-=604800.0;
        
        frq  =U1(p+ 7)-7; /* frequency */
        flag =U1(p+ 8);   /* tracking status */
        lock1=U1(p+16);   /* code lock count */
        lock2=U1(p+17);   /* phase lock count */
        snr  =U2(p+20)/256.0;
        adr  =I8(p+32)*P2_32+(flag&0x40?0.5:0.0);
        dop  =I4(p+40)*P2_10*10.0;
        
        /* set slip flag */
        if (lock2==0||lock2<raw->lockt[sat-1][0]) raw->lockt[sat-1][1]=1.0;
        raw->lockt[sat-1][0]=lock2;
        
#if 0 /* for debug */
        trace(2,"[%2d] qi=%d sys=%d prn=%3d frq=%2d flag=%02X ?=%02X %02X "
              "%02X %02X %02X %02X %02X lock=%3d %3d ts=%10.3f snr=%4.1f "
              "dop=%9.3f adr=%13.3f %6.3f\n",U1(p),qi,U1(p+4),prn,frq,flag,
              U1(p+9),U1(p+10),U1(p+11),U1(p+12),U1(p+13),U1(p+14),U1(p+15),
              lock1,lock2,ts,snr,dop,adr,
              adrs[sat-1]==0.0||dop==0.0?0.0:(adr-adrs[sat-1])-dop);
#endif
        adrs[sat-1]=adr;
        
        /* check phase lock */
        if (!(flag&0x20)) continue;
        
        raw->obs.data[n].time=time;
        raw->obs.data[n].sat=sat;
        raw->obs.data[n].P[0]=tau*CLIGHT;
        raw->obs.data[n].L[0]=-adr;
        raw->obs.data[n].D[0]=(float)dop;
        raw->obs.data[n].SNR[0]=(uint16_t)(snr/SNR_UNIT+0.5);
        raw->obs.data[n].code[0]=sys==SYS_CMP?CODE_L2I:CODE_L1C;
        raw->obs.data[n].Lstd[0]=8-qi;
        raw->obs.data[n].LLI[0]=raw->lockt[sat-1][1]>0.0?1:0;
        if (sys==SYS_SBS) { /* half-cycle valid */
            raw->obs.data[n].LLI[0]|=lock2>142?0:2;
        }
        else {
            raw->obs.data[n].LLI[0]|=flag&0x80?0:2;
        }
        raw->lockt[sat-1][1]=0.0;
        /* adjust code measurements for GLONASS sats */
        if (sys==SYS_GLO&&frq>=-7&&frq<=7) {
            if (fw==2) raw->obs.data[n].P[0]+=(double)P_adj_fw2[frq+7];
            if (fw==3) raw->obs.data[n].P[0]+=(double)P_adj_fw3[frq+7];
        }
        for (j=1;j<NFREQ+NEXOBS;j++) {
            raw->obs.data[n].L[j]=raw->obs.data[n].P[j]=0.0;
            raw->obs.data[n].D[j]=0.0;
            raw->obs.data[n].SNR[j]=raw->obs.data[n].LLI[j]=0;
            raw->obs.data[n].Lstd[j]=raw->obs.data[n].Pstd[j]=0;
            raw->obs.data[n].code[j]=CODE_NONE;
        }
        n++;
    }
    if (n<=0) return 0;
    raw->time=time;
    raw->obs.n=n;
    return 1;
}
/* decode UBX-TRKD5: trace measurement data (unofficial) ---------------------*/
static int decode_trkd5(raw_t *raw)
{
    static double adrs[MAXSAT]={0};
    gtime_t time;
    double ts,tr=-1.0,t,tau,adr,dop,snr,utc_gpst;
    int i,j,n=0,type,off,len,sys,prn,sat,qi,frq,flag,week;
    uint8_t *p=raw->buff+6;
    
    trace(4,"decode_trkd5: len=%d\n",raw->len);
    
    if (raw->outtype) {
        sprintf(raw->msgtype,"UBX TRK-D5    (%4d):",raw->len);
    }
    if (!raw->time.time) return 0;
    
    utc_gpst=timediff(gpst2utc(raw->time),raw->time);
    
    switch ((type=U1(p))) {
        case 3 : off=86; len=56; break;
        case 6 : off=86; len=64; break; /* u-blox 7 */
        default: off=78; len=56; break;
    }
    for (i=0,p=raw->buff+off;p-raw->buff<raw->len-2;i++,p+=len) {
        qi=U1(p+41)&7;
        if (qi<4||7<qi) continue;
        t=I8(p)*P2_32/1000.0;
        if (ubx_sys(U1(p+56))==SYS_GLO) t-=10800.0+utc_gpst;
        if (t>tr) {tr=t; break;}
    }
    if (tr<0.0) return 0;
    
    tr=ROUND((tr+0.08)/0.1)*0.1;
    
    /* adjust week handover */
    t=time2gpst(raw->time,&week);
    if      (tr<t-302400.0) week++;
    else if (tr>t+302400.0) week--;
    time=gpst2time(week,tr);
    
    trace(4,"time=%s\n",time_str(time,0));
    
    for (i=0,p=raw->buff+off;p-raw->buff<raw->len-2;i++,p+=len) {
        
        /* quality indicator */
        qi =U1(p+41)&7;
        if (qi<4||7<qi) continue;
        
        if (type==6) {
            if (!(sys=ubx_sys(U1(p+56)))) {
                trace(2,"ubx trkd5: system error\n");
                continue;
            }
            prn=U1(p+57)+(sys==SYS_QZS?192:0);
            frq=U1(p+59)-7;
        }
        else {
            prn=U1(p+34);
            sys=prn<MINPRNSBS?SYS_GPS:SYS_SBS;
        }
        if (!(sat=satno(sys,prn))) {
            trace(2,"ubx trkd5 sat number error: sys=%2d prn=%2d\n",sys,prn);
            continue;
        }
        /* transmission time */
        ts=I8(p)*P2_32/1000.0;
        if (sys==SYS_GLO) ts-=10800.0+utc_gpst; /* glot -> gpst */
        
        /* signal travel time */
        tau=tr-ts;
        if      (tau<-302400.0) tau+=604800.0;
        else if (tau> 302400.0) tau-=604800.0;
        
        flag=U1(p+54);   /* tracking status */
        adr=qi<6?0.0:I8(p+8)*P2_32+(flag&0x01?0.5:0.0);
        dop=I4(p+16)*P2_10/4.0;
        snr=U2(p+32)/256.0;
        
        if (snr<=10.0) raw->lockt[sat-1][1]=1.0;
        
#if 0 /* for debug */
        trace(2,"[%2d] qi=%d sys=%d prn=%3d frq=%2d flag=%02X ts=%1.3f "
              "snr=%4.1f dop=%9.3f adr=%13.3f %6.3f\n",U1(p+35),qi,U1(p+56),
              prn,frq,flag,ts,snr,dop,adr,
              adrs[sat-1]==0.0||dop==0.0?0.0:(adr-adrs[sat-1])-dop);
#endif
        adrs[sat-1]=adr;
        
        /* check phase lock */
        if (!(flag&0x08)) continue;
        
        raw->obs.data[n].time=time;
        raw->obs.data[n].sat=sat;
        raw->obs.data[n].P[0]=tau*CLIGHT;
        raw->obs.data[n].L[0]=-adr;
        raw->obs.data[n].D[0]=(float)dop;
        raw->obs.data[n].SNR[0]=(uint16_t)(snr/SNR_UNIT+0.5);
        raw->obs.data[n].code[0]=sys==SYS_CMP?CODE_L2I:CODE_L1C;
        raw->obs.data[n].LLI[0]=raw->lockt[sat-1][1]>0.0?1:0;
        raw->lockt[sat-1][1]=0.0;
        
        for (j=1;j<NFREQ+NEXOBS;j++) {
            raw->obs.data[n].L[j]=raw->obs.data[n].P[j]=0.0;
            raw->obs.data[n].D[j]=0.0;
            raw->obs.data[n].SNR[j]=raw->obs.data[n].LLI[j]=0;
            raw->obs.data[n].code[j]=CODE_NONE;
        }
        n++;
    }
    if (n<=0) return 0;
    raw->time=time;
    raw->obs.n=n;
    return 1;
}
/* UTC 8-bit week -> full week -----------------------------------------------*/
static void adj_utcweek(gtime_t time, double *utc)
{
    int week;
    
    time2gpst(time,&week);
    utc[3]+=week/256*256;
    if      (utc[3]<week-127) utc[3]+=256.0;
    else if (utc[3]>week+127) utc[3]-=256.0;
    utc[5]+=utc[3]/256*256;
    if      (utc[5]<utc[3]-127) utc[5]+=256.0;
    else if (utc[5]>utc[3]+127) utc[5]-=256.0;
}
/* decode GPS/QZSS ephemeris -------------------------------------------------*/
static int decode_eph(raw_t *raw, int sat)
{
    eph_t eph={0};
    
    if (!decode_frame(raw->subfrm[sat-1],&eph,NULL,NULL,NULL)) return 0;
    
    if (!strstr(raw->opt,"-EPHALL")) {
        if (eph.iode==raw->nav.eph[sat-1].iode&&
            eph.iodc==raw->nav.eph[sat-1].iodc&&
            timediff(eph.toe,raw->nav.eph[sat-1].toe)==0.0&&
            timediff(eph.toc,raw->nav.eph[sat-1].toc)==0.0) return 0;
    }
    eph.sat=sat;
    raw->nav.eph[sat-1]=eph;
    raw->ephsat=sat;
    raw->ephset=0;
    return 2;
}
/* decode GPS/QZSS ION/UTC parameters ----------------------------------------*/
static int decode_ionutc(raw_t *raw, int sat)
{
    double ion[8],utc[8];
    int sys=satsys(sat,NULL);
    
    if (!decode_frame(raw->subfrm[sat-1],NULL,NULL,ion,utc)) return 0;
    
    adj_utcweek(raw->time,utc);
    if (sys==SYS_QZS) {
        matcpy(raw->nav.ion_qzs,ion,8,1);
        matcpy(raw->nav.utc_qzs,utc,8,1);
    }
    else {
        matcpy(raw->nav.ion_gps,ion,8,1);
        matcpy(raw->nav.utc_gps,utc,8,1);
    }
    return 9;
}
/* decode GPS/QZSS navigation data -------------------------------------------*/
static int decode_nav(raw_t *raw, int sat, int off)
{
    uint8_t *p=raw->buff+6+off,buff[30];
    int i,id,ret;
    
    if (raw->len<48+off) {
        trace(2,"ubx rxmsfrbx nav length error: sat=%d len=%d\n",sat,raw->len);
        return -1;
    }
    if ((U4(p)>>24)==PREAMB_CNAV) {
        trace(3,"ubx rxmsfrbx nav unsupported sat=%d len=%d\n",sat,raw->len);
        return 0;
    }
    for (i=0;i<10;i++,p+=4) { /* 24 x 10 bits w/o parity */
        setbitu(buff,24*i,24,U4(p)>>6);
    }
    id=getbitu(buff,43,3);
    if (id<1||id>5) {
        trace(2,"ubx rxmsfrbx nav subframe id error: sat=%d id=%d\n",sat,id);
        return -1;
    }
    memcpy(raw->subfrm[sat-1]+(id-1)*30,buff,30);
    
    if (id==3) {
        return decode_eph(raw,sat);
    }
    if (id==4||id==5) {
        ret=decode_ionutc(raw,sat);
        memset(raw->subfrm[sat-1]+(id-1)*30,0,30);
        return ret;
    }
    return 0;
}
/* decode Galileo I/NAV navigation data --------------------------------------*/
static int decode_enav(raw_t *raw, int sat, int off)
{
    eph_t eph={0};
    double ion[4]={0},utc[8]={0};
    uint8_t *p=raw->buff+6+off,buff[32],crc_buff[26]={0};
    int i,j,part1,page1,part2,page2,type;
    
    if (raw->len<40+off) {
        trace(2,"ubx rxmsfrbx enav length error: sat=%d len=%d\n",sat,raw->len);
        return -1;
    }
    if (raw->len<36+off) return 0; /* E5b I/NAV */
    
    for (i=0;i<8;i++,p+=4) {
        setbitu(buff,32*i,32,U4(p));
    }
    part1=getbitu(buff   ,0,1);
    page1=getbitu(buff   ,1,1);
    part2=getbitu(buff,128,1);
    page2=getbitu(buff,129,1);
    
    if (part1!=0||part2!=1) {
        trace(3,"ubx rxmsfrbx enav page even/odd error: sat=%d\n",sat);
        return -1;
    }
    if (page1==1||page2==1) return 0; /* alert page */
    
    /* test crc (4(pad) + 114 + 82 bits) */
    for (i=0,j=  4;i<15;i++,j+=8) setbitu(crc_buff,j,8,getbitu(buff   ,i*8,8));
    for (i=0,j=118;i<11;i++,j+=8) setbitu(crc_buff,j,8,getbitu(buff,i*8+128,8));
    if (rtk_crc24q(crc_buff,25)!=getbitu(buff,128+82,24)) {
        trace(2,"ubx rxmsfrbx enav crc error: sat=%d\n",sat);
        return -1;
    }
    type=getbitu(buff,2,6); /* word type */
    
    if (type>6) return 0;
    
    /* save 128 (112:even+16:odd) bits word */
    for (i=0,j=2;i<14;i++,j+=8) {
        raw->subfrm[sat-1][type*16+i]=getbitu(buff,j,8);
    }
    for (i=14,j=130;i<16;i++,j+=8) {
        raw->subfrm[sat-1][type*16+i]=getbitu(buff,j,8);
    }
    if (type!=5) return 0;
    if (!decode_gal_inav(raw->subfrm[sat-1],&eph,ion,utc)) return 0;
        
    if (eph.sat!=sat) {
        trace(2,"ubx rxmsfrbx enav satellite error: sat=%d %d\n",sat,eph.sat);
        return -1;
    }
    eph.code|=(1<<0); /* data source: E1 */
    
    adj_utcweek(raw->time,utc);
    matcpy(raw->nav.ion_gal,ion,4,1);
    matcpy(raw->nav.utc_gal,utc,8,1);
    
    if (!strstr(raw->opt,"-EPHALL")) {
        if (eph.iode==raw->nav.eph[sat-1].iode&&
            timediff(eph.toe,raw->nav.eph[sat-1].toe)==0.0&&
            timediff(eph.toc,raw->nav.eph[sat-1].toc)==0.0) return 0;
    }
    raw->nav.eph[sat-1]=eph;
    raw->ephsat=sat;
    raw->ephset=0; /* 0:I/NAV */
    return 2;
}
/* decode BDS navigation data ------------------------------------------------*/
static int decode_cnav(raw_t *raw, int sat, int off)
{
    eph_t eph={0};
    double ion[8],utc[8];
    uint8_t *p=raw->buff+6+off,buff[38]={0};
    int i,id,pgn,prn;
    
    if (raw->len<48+off) {
        trace(2,"ubx rxmsfrbx cnav length error: sat=%d len=%d\n",sat,raw->len);
        return -1;
    }
    for (i=0;i<10;i++,p+=4) {
        setbitu(buff,30*i,30,U4(p));
    }
    id=getbitu(buff,15,3); /* subframe ID */
    if (id<1||5<id) {
        trace(2,"ubx rxmsfrbx cnav subframe id error: sat=%2d\n",sat);
        return -1;
    }
    satsys(sat,&prn);
    
    if (prn>=6&&prn<=58) { /* IGSO/MEO */
        memcpy(raw->subfrm[sat-1]+(id-1)*38,buff,38);
        
        if (id==3) {
            if (!decode_bds_d1(raw->subfrm[sat-1],&eph,NULL,NULL)) return 0;
        }
        else if (id==5) {
            if (!decode_bds_d1(raw->subfrm[sat-1],NULL,ion,utc)) return 0;
            matcpy(raw->nav.ion_cmp,ion,8,1);
            matcpy(raw->nav.utc_cmp,utc,8,1);
            return 9;
        }
        else return 0;
    }
    else { /* GEO */
        pgn=getbitu(buff,42,4); /* page numuber */
        
        if (id==1&&pgn>=1&&pgn<=10) {
            memcpy(raw->subfrm[sat-1]+(pgn-1)*38,buff,38);
            if (pgn!=10) return 0;
            if (!decode_bds_d2(raw->subfrm[sat-1],&eph,NULL)) return 0;
        }
        else if (id==5&&pgn==102) {
            memcpy(raw->subfrm[sat-1]+10*38,buff,38);
            if (!decode_bds_d2(raw->subfrm[sat-1],NULL,utc)) return 0;
            matcpy(raw->nav.utc_cmp,utc,8,1);
            return 9;
        }
        else return 0;
    }
    if (!strstr(raw->opt,"-EPHALL")) {
        if (timediff(eph.toe,raw->nav.eph[sat-1].toe)==0.0) return 0;
    }
    eph.sat=sat;
    raw->nav.eph[sat-1]=eph;
    raw->ephsat=sat;
    raw->ephset=0;
    return 2;
}
/* decode GLONASS navigation data --------------------------------------------*/
static int decode_gnav(raw_t *raw, int sat, int off, int frq)
{
    geph_t geph={0};
    double utc_glo[8]={0};
    int i,j,k,m,prn;
    uint8_t *p=raw->buff+6+off,buff[64],*fid;
    
    satsys(sat,&prn);
    
    if (raw->len<24+off) {
        trace(2,"ubx rxmsfrbx gnav length error: len=%d\n",raw->len);
        return -1;
    }
    for (i=k=0;i<4;i++,p+=4) for (j=0;j<4;j++) {
        buff[k++]=p[3-j];
    }
    /* test hamming of GLONASS string */
    if (!test_glostr(buff)) {
        trace(2,"ubx rxmsfrbx gnav hamming error: sat=%2d\n",sat);
        return -1;
    }
    m=getbitu(buff,1,4);
    if (m<1||15<m) {
        trace(2,"ubx rxmsfrbx gnav string no error: sat=%2d\n",sat);
        return -1;
    }
    /* flush frame buffer if frame-ID changed */
    fid=raw->subfrm[sat-1]+150;
    if (fid[0]!=buff[12]||fid[1]!=buff[13]) {
        for (i=0;i<4;i++) memset(raw->subfrm[sat-1]+i*10,0,10);
        memcpy(fid,buff+12,2); /* save frame-id */
    }
    memcpy(raw->subfrm[sat-1]+(m-1)*10,buff,10);
    
    if (m==4) {
        /* decode GLONASS ephemeris strings */
        geph.tof=raw->time;
        if (!decode_glostr(raw->subfrm[sat-1],&geph,NULL)||geph.sat!=sat) {
            return 0;
        }
        geph.frq=frq-7;
        
        if (!strstr(raw->opt,"-EPHALL")) {
            if (geph.iode==raw->nav.geph[prn-1].iode) return 0;
        }
        raw->nav.geph[prn-1]=geph;
        raw->ephsat=sat;
        raw->ephset=0;
        return 2;
    }
    else if (m==5) {
        if (!decode_glostr(raw->subfrm[sat-1],NULL,utc_glo)) return 0;
        matcpy(raw->nav.utc_glo,utc_glo,8,1);
        return 9;
    }
    return 0;
}
/* decode SBAS navigation data -----------------------------------------------*/
static int decode_snav(raw_t *raw, int prn, int off)
{
    int i,tow,week;
    uint8_t *p=raw->buff+6+off,buff[32];
    
    if (raw->len<40+off) {
        trace(2,"ubx rxmsfrbx snav length error: len=%d\n",raw->len);
        return -1;
    }
    tow=(int)time2gpst(timeadd(raw->time,-1.0),&week);
    raw->sbsmsg.prn=prn;
    raw->sbsmsg.tow=tow;
    raw->sbsmsg.week=week;
    for (i=0;i<8;i++,p+=4) {
        setbitu(buff,32*i,32,U4(p));
    }
    memcpy(raw->sbsmsg.msg,buff,29);
    raw->sbsmsg.msg[28]&=0xC0;
    return 3;
}
/* decode UBX-RXM-SFRBX: raw subframe data (ref [3][4][5]) -------------------*/
static int decode_rxmsfrbx(raw_t *raw)
{
    uint8_t *p=raw->buff+6;
    int prn,sat,sys;
    
    trace(4,"decode_rxmsfrbx: len=%d\n",raw->len);
    
    if (raw->outtype) {
        sprintf(raw->msgtype,"UBX RXM-SFRBX (%4d): sys=%d prn=%3d",raw->len,
                U1(p),U1(p+1));
    }
    if (!(sys=ubx_sys(U1(p)))) {
        trace(2,"ubx rxmsfrbx sys id error: sys=%d\n",U1(p));
        return -1;
    }
    prn=U1(p+1)+((sys==SYS_QZS)?192:0);
    if (!(sat=satno(sys,prn))) {
        if (sys==SYS_GLO&&prn==255) {
            return 0; /* suppress error for unknown GLONASS satellite */
        }
        trace(2,"ubx rxmsfrbx sat number error: sys=%d prn=%d\n",sys,prn);
        return -1;
    }
    if (sys==SYS_QZS&&raw->len==52) { /* QZSS L1S */
        sys=SYS_SBS;
        prn-=10;
    }
    switch (sys) {
        case SYS_GPS: return decode_nav (raw,sat,8);
        case SYS_QZS: return decode_nav (raw,sat,8);
        case SYS_GAL: return decode_enav(raw,sat,8);
        case SYS_CMP: return decode_cnav(raw,sat,8);
        case SYS_GLO: return decode_gnav(raw,sat,8,U1(p+3));
        case SYS_SBS: return decode_snav(raw,prn,8);
    }
    return 0;
}
/* decode UBX-TRK-SFRBX: subframe buffer extension (unofficial) --------------*/
static int decode_trksfrbx(raw_t *raw)
{
    uint8_t *p=raw->buff+6;
    int prn,sat,sys;
    
    if (raw->outtype) {
        sprintf(raw->msgtype,"UBX TRK-SFRBX (%4d): sys=%d prn=%3d",raw->len,
                U1(p+1),U1(p+2));
    }
    if (!(sys=ubx_sys(U1(p+1)))) {
        trace(2,"ubx trksfrbx sys id error: sys=%d\n",U1(p+1));
        return -1;
    }
    prn=U1(p+2)+(sys==SYS_QZS?192:0);
    if (!(sat=satno(sys,prn))) {
        trace(2,"ubx trksfrbx sat number error: sys=%d prn=%d\n",sys,prn);
        return -1;
    }
    switch (sys) {
        case SYS_GPS: return decode_nav (raw,sat,13);
        case SYS_QZS: return decode_nav (raw,sat,13);
        case SYS_GAL: return decode_enav(raw,sat,13);
        case SYS_CMP: return decode_cnav(raw,sat,13);
        case SYS_GLO: return decode_gnav(raw,sat,13,U1(p+4));
        case SYS_SBS: return decode_snav(raw,sat,13);
    }
    return 0;
}
/* decode UBX-RXM-SFRB: subframe buffer (GPS/SBAS) ---------------------------*/
static int decode_rxmsfrb(raw_t *raw)
{
    uint32_t words[10];
    uint8_t *p=raw->buff+6,buff[30];
    int i,sys,prn,sat,id;
    
    if (raw->outtype) {
        sprintf(raw->msgtype,"UBX RXM-SFRB  (%4d): prn=%2d",raw->len,U1(p+1));
    }
    if (raw->len<42) {
        trace(2,"ubx rxmsfrb length error: len=%d\n",raw->len);
        return -1;
    }
    prn=U1(p+1);
    sys=(prn>=MINPRNSBS)?SYS_SBS:SYS_GPS;
    
    if (!(sat=satno(sys,prn))) {
        trace(2,"ubx rxmsfrb satellite error: prn=%d\n",prn);
        return -1;
    }
    if (sys==SYS_GPS) {
        for (i=0,p+=2;i<10;i++,p+=4) setbitu(buff,24*i,24,U4(p));
        id=getbitu(buff,43,3);
        if (id>=1&&id<=5) {
            memcpy(raw->subfrm[sat-1]+(id-1)*30,buff,30);
            if      (id==3) return decode_eph   (raw,sat);
            else if (id==4) return decode_ionutc(raw,sat);
        }
    }
    else {
        for (i=0,p+=2;i<10;i++,p+=4) words[i]=U4(p);
        if (!sbsdecodemsg(raw->time,prn,words,&raw->sbsmsg)) return 0;
        return 3;
    }
    return 0;
}
/* decode ubx-tim-tm2: time mark data ----------------------------------------*/
static int decode_timtm2(raw_t *raw)
{
    gtime_t eventime;
    char ch, flags;
    unsigned int count, wnR, wnF;
    unsigned long towMsR, towSubMsR, towMsF, towSubMsF, accEst;
    int time, timeBase, newRisingEdge, newFallingEdge;
    unsigned char *p=raw->buff+6;
    double tr[6],tf[6];

    trace(4, "decode_timtm2: len=%d\n", raw->len);

    if (raw->outtype) {
        sprintf(raw->msgtype, "UBX TIM-TM2 (%4d)", raw->len);
    }
    ch = U1(p);
    flags = *(p+1);
    count = U2(p+2);
    wnR = U2(p+4);
    wnF = U2(p+6);
    towMsR = U4(p+8);
    towSubMsR = U4(p+12);
    towMsF = U4(p+16);
    towSubMsF = U4(p+20);
    accEst = U4(p+24);

    /* extract flags to variables */
    newFallingEdge = ((flags >> 2) & 0x01);
    timeBase =       ((flags >> 3) & 0x03);
    time =           ((flags >> 6) & 0x01);
    newRisingEdge =  ((flags >> 7) & 0x01);

    if (newFallingEdge)
    {
        eventime = gpst2time(wnF,towMsF*1E-3+towSubMsF*1E-9);
        if (timeBase==2) /* if timeBase is UTC, convert to GPS */
            eventime = utc2gpst(eventime);
        raw->obs.flag = 5; /* Event flag */
        raw->obs.data[0].eventime = eventime;
        raw->obs.rcvcount = count;
        raw->obs.tmcount++;
        raw->obs.data[0].timevalid = time;
    } else {
        raw->obs.flag = 0;
    }
    time2epoch(gpst2time(wnR,towMsR*1E-3+towSubMsR*1E-9),tr);
    time2epoch(gpst2time(wnF,towMsF*1E-3+towSubMsF*1E-9),tf);
    trace(3,"time mark rise: %f:%f:%.3f\n",tr[3],tr[4],tr[5]);
    trace(3,"time mark fall: %f:%f:%.3f\n",tf[3],tf[4],tf[5]);
    return 0;
}

/* decode ublox raw message --------------------------------------------------*/
static int decode_ubx(raw_t *raw)
{
    int type=(U1(raw->buff+2)<<8)+U1(raw->buff+3);
    
    trace(3,"decode_ubx: type=%04x len=%d\n",type,raw->len);
    
    /* checksum */
    if (!checksum(raw->buff,raw->len)) {
        trace(2,"ubx checksum error: type=%04x len=%d\n",type,raw->len);
        return -1;
    }
    switch (type) {
        case ID_RXMRAW  : return decode_rxmraw  (raw);
        case ID_RXMRAWX : return decode_rxmrawx (raw);
        case ID_RXMSFRB : return decode_rxmsfrb (raw);
        case ID_RXMSFRBX: return decode_rxmsfrbx(raw);
        case ID_NAVSOL  : return decode_navsol  (raw);
        case ID_NAVTIME : return decode_navtime (raw);
        case ID_TRKMEAS : return decode_trkmeas (raw);
        case ID_TRKD5   : return decode_trkd5   (raw);
        case ID_TRKSFRBX: return decode_trksfrbx(raw);
        case ID_TIMTM2  : return decode_timtm2  (raw);
    }
    if (raw->outtype) {
        sprintf(raw->msgtype,"UBX 0x%02X 0x%02X (%4d)",type>>8,type&0xF,
                raw->len);
    }
    return 0;
}
/* sync code -----------------------------------------------------------------*/
static int sync_ubx(uint8_t *buff, uint8_t data)
{
    buff[0]=buff[1]; buff[1]=data;
    return buff[0]==UBXSYNC1&&buff[1]==UBXSYNC2;
}
/* input ublox raw message from stream -----------------------------------------
* fetch next ublox raw data and input a message from stream
* args   : raw_t *raw   IO     receiver raw data control struct
*          uint8_t data     I   stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 3: input sbas message,
*                  9: input ion/utc parameter)
*
* notes  : to specify input options, set raw->opt to the following option
*          strings separated by spaces.
*
*          -EPHALL    : input all ephemerides
*          -INVCP     : invert polarity of carrier-phase
*          -TADJ=tint : adjust time tags to multiples of tint (sec)
*          -STD_SLIP=std: slip by std-dev of carrier phase under std
*          -MAX_CP_STD=std: max std-dev of carrier phase
*          -MULTICODE :  preserve multiple signal codes for single freq
*          -RCVSTDS :  save receiver stdevs to unused rinex fields

*
*          The supported messages are as follows.
*
*          UBX-RXM-RAW  : raw measurement data
*          UBX-RXM-RAWX : multi-gnss measurement data
*          UBX-RXM-SFRB : subframe buffer
*          UBX-RXM-SFRBX: subframe buffer extension
*
*          UBX-TRK-MEAS and UBX-TRK-SFRBX are based on NEO-M8N (F/W 2.01).
*          UBX-TRK-D5 is based on NEO-7N (F/W 1.00). They are not formally
*          documented and not supported by u-blox.
*          Users can use these messages by their own risk.
*-----------------------------------------------------------------------------*/
extern int input_ubx(raw_t *raw, uint8_t data)
{
    trace(5,"input_ubx: data=%02x\n",data);
    
    /* synchronize frame */
    if (raw->nbyte==0) {
        if (!sync_ubx(raw->buff,data)) return 0;
        raw->nbyte=2;
        return 0;
    }
    raw->buff[raw->nbyte++]=data;
    
    if (raw->nbyte==6) {
        if ((raw->len=U2(raw->buff+4)+8)>MAXRAWLEN) {
            trace(2,"ubx length error: len=%d\n",raw->len);
            raw->nbyte=0;
            return -1;
        }
    }
    if (raw->nbyte<6||raw->nbyte<raw->len) return 0;
    raw->nbyte=0;
    
    /* decode ublox raw message */
    return decode_ubx(raw);
}
/* input ublox raw message from file -------------------------------------------
* fetch next ublox raw data and input a message from file
* args   : raw_t  *raw   IO     receiver raw data control struct
*          FILE   *fp    I      file pointer
* return : status(-2: end of file, -1...9: same as above)
*-----------------------------------------------------------------------------*/
extern int input_ubxf(raw_t *raw, FILE *fp)
{
    int i,data;
    
    trace(4,"input_ubxf:\n");
    
    /* synchronize frame */
    if (raw->nbyte==0) {
        for (i=0;;i++) {
            if ((data=fgetc(fp))==EOF) return -2;
            if (sync_ubx(raw->buff,(uint8_t)data)) break;
            if (i>=4096) return 0;
        }
    }
    if (fread(raw->buff+2,1,4,fp)<4) return -2;
    raw->nbyte=6;
    
    if ((raw->len=U2(raw->buff+4)+8)>MAXRAWLEN) {
        trace(2,"ubx length error: len=%d\n",raw->len);
        raw->nbyte=0;
        return -1;
    }
    if (fread(raw->buff+6,1,raw->len-6,fp)<(size_t)(raw->len-6)) return -2;
    raw->nbyte=0;
    
    /* decode ubx raw message */
    return decode_ubx(raw);
}
/* convert string to integer -------------------------------------------------*/
static int stoi(const char *s)
{
    uint32_t n;
    if (sscanf(s,"0x%X",&n)==1) return (int)n; /* hex (0xXXXX) */
    return atoi(s);
}
/* generate ublox binary message -----------------------------------------------
* generate ublox binary message from message string
* args   : char  *msg   IO     message string 
*            "CFG-PRT   portid res0 res1 mode baudrate inmask outmask flags"
*            "CFG-USB   vendid prodid res1 res2 power flags vstr pstr serino"
*            "CFG-MSG   msgid rate0 rate1 rate2 rate3 rate4 rate5 rate6"
*            "CFG-NMEA  filter version numsv flags"
*            "CFG-RATE  meas nav time"
*            "CFG-CFG   clear_mask save_mask load_mask [dev_mask]"
*            "CFG-TP    interval length status time_ref res adelay rdelay udelay"
*            "CFG-NAV2  ..."
*            "CFG-DAT   maja flat dx dy dz rotx roty rotz scale"
*            "CFG-INF   protocolid res0 res1 res2 mask0 mask1 mask2 ... mask5"
*            "CFG-RST   navbbr reset res"
*            "CFG-RXM   gpsmode lpmode"
*            "CFG-ANT   flags pins"
*            "CFG-FXN   flags treacq tacq treacqoff tacqoff ton toff res basetow"
*            "CFG-SBAS  mode usage maxsbas res scanmode"
*            "CFG-LIC   key0 key1 key2 key3 key4 key5"
*            "CFG-TM    intid rate flags"
*            "CFG-TM2   ch res0 res1 rate flags"
*            "CFG-TMODE tmode posx posy posz posvar svinmindur svinvarlimit"
*            "CFG-EKF   ..."
*            "CFG-GNSS  ..."
*            "CFG-ITFM  conf conf2"
*            "CFG-LOGFILTER ver flag min_int time_thr speed_thr pos_thr"
*            "CFG-NAV5  ..."
*            "CFG-NAVX5 ..."
*            "CFG-ODO   ..."
*            "CFG-PM2   ..."
*            "CFG-PWR   ver rsv1 rsv2 rsv3 state"
*            "CFG-RINV  flag data ..."
*            "CFG-SMGR  ..."
*            "CFG-TMODE2 ..."
*            "CFG-TMODE3 ..."
*            "CFG-TPS   ..."
*            "CFG-TXSLOT ..."
*            "CFG-VALDEL ver layer res0 res1 key [key ...]"
*            "CFG-VALGET ver layer pos key [key ...]"
*            "CFG-VALSET ver layer res0 res1 key value [key value ...]"
*          uint8_t *buff O binary message
* return : length of binary message (0: error)
* note   : see reference [1][3][5] for details.
*          the following messages are not supported:
*             CFG-DOSC,CFG-ESRC
*-----------------------------------------------------------------------------*/
extern int gen_ubx(const char *msg, uint8_t *buff)
{
    const char *cmd[]={
        "PRT","USB","MSG","NMEA","RATE","CFG","TP","NAV2","DAT","INF",
        "RST","RXM","ANT","FXN","SBAS","LIC","TM","TM2","TMODE","EKF",
        "GNSS","ITFM","LOGFILTER","NAV5","NAVX5","ODO","PM2","PWR","RINV","SMGR",
        "TMODE2","TMODE3","TPS","TXSLOT",
        "VALDEL","VALGET","VALSET",""
    };
    const uint8_t id[]={
        0x00,0x1B,0x01,0x17,0x08,0x09,0x07,0x1A,0x06,0x02,
        0x04,0x11,0x13,0x0E,0x16,0x80,0x10,0x19,0x1D,0x12,
        0x3E,0x39,0x47,0x24,0x23,0x1E,0x3B,0x57,0x34,0x62,
        0x36,0x71,0x31,0x53,
        0x8c,0x8b,0x8a
    };
    const int prm[][32]={
        {FU1,FU1,FU2,FU4,FU4,FU2,FU2,FU2,FU2},    /* PRT */
        {FU2,FU2,FU2,FU2,FU2,FU2,FS32,FS32,FS32}, /* USB */
        {FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1},        /* MSG */
        {FU1,FU1,FU1,FU1},                        /* NMEA */
        {FU2,FU2,FU2},                            /* RATE */
        {FU4,FU4,FU4,FU1},                        /* CFG */
        {FU4,FU4,FI1,FU1,FU2,FI2,FI2,FI4},        /* TP */
        {FU1,FU1,FU2,FU1,FU1,FU1,FU1,FI4,FU1,FU1,FU1,FU1,FU1,FU1,FU2,FU2,FU2,FU2,
         FU2,FU1,FU1,FU2,FU4,FU4},                /* NAV2 */
        {FR8,FR8,FR4,FR4,FR4,FR4,FR4,FR4,FR4},    /* DAT */
        {FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1}, /* INF */
        {FU2,FU1,FU1},                            /* RST */
        {FU1,FU1},                                /* RXM */
        {FU2,FU2},                                /* ANT */
        {FU4,FU4,FU4,FU4,FU4,FU4,FU4,FU4},        /* FXN */
        {FU1,FU1,FU1,FU1,FU4},                    /* SBAS */
        {FU2,FU2,FU2,FU2,FU2,FU2},                /* LIC */
        {FU4,FU4,FU4},                            /* TM */
        {FU1,FU1,FU2,FU4,FU4},                    /* TM2 */
        {FU4,FI4,FI4,FI4,FU4,FU4,FU4},            /* TMODE */
        {FU1,FU1,FU1,FU1,FU4,FU2,FU2,FU1,FU1,FU2}, /* EKF */
        {FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU4},    /* GNSS */
        {FU4,FU4},                                /* ITFM */
        {FU1,FU1,FU2,FU2,FU2,FU4},                /* LOGFILTER */
        {FU2,FU1,FU1,FI4,FU4,FI1,FU1,FU2,FU2,FU2,FU2,FU1,FU1,FU1,FU1,FU1,FU1,FU2,
         FU1,FU1,FU1,FU1,FU1,FU1},                /* NAV5 */
        {FU2,FU2,FU4,FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU2,FU1,FU1,FU1,FU1,
         FU1,FU1,FU1,FU1,FU1,FU1,FU2},            /* NAVX5 */
        {FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1},    /* ODO */
        {FU1,FU1,FU1,FU1,FU4,FU4,FU4,FU4,FU2,FU2}, /* PM2 */
        {FU1,FU1,FU1,FU1,FU4},                    /* PWR */
        {FU1,FU1},                                /* RINV */
        {FU1,FU1,FU2,FU2,FU1,FU1,FU2,FU2,FU2,FU2,FU4}, /* SMGR */
        {FU1,FU1,FU2,FI4,FI4,FI4,FU4,FU4,FU4},    /* TMODE2 */
        {FU1,FU1,FU2,FI4,FI4,FI4,FU4,FU4,FU4},    /* TMODE3 */
        {FU1,FU1,FU1,FU1,FI2,FI2,FU4,FU4,FU4,FU4,FI4,FU4}, /* TPS */
        {FU1,FU1,FU1,FU1,FU4,FU4,FU4,FU4,FU4},     /* TXSLOT */
        {FU1,FU1,FU1,FU1},                        /* VALDEL */
        {FU1,FU1,FU2},                            /* VALGET */
        {FU1,FU1,FU1,FU1}                         /* VALSET */
    };
    uint8_t *q=buff;
    char mbuff[1024],*args[32],*p;
    int i,j,n,narg=0;
    bool isvalset = false;
    
    trace(4,"gen_ubxf: msg=%s\n",msg);
    
    strcpy(mbuff,msg);
    for (p=strtok(mbuff," ");p&&narg<32;p=strtok(NULL," ")) {
        args[narg++]=p;
    }
    if (narg<1||strncmp(args[0],"CFG-",4)) return 0;
    
    for (i=0;*cmd[i];i++) {
        if (!strcmp(args[0]+4,cmd[i])) break;
    }
    if (!*cmd[i]) return 0;
    
    *q++=UBXSYNC1;
    *q++=UBXSYNC2;
    *q++=UBXCFG;
    *q++=id[i];
    q+=2;


    if (i == 36) isvalset = true;

    /* VALSET sanity check */
    if (isvalset) {
        if (narg == 7) narg = narg - 2;     /* Adjusting for key value addition */
        else return 0;
    }
    for (j=1;prm[i][j-1]||j<narg;j++) {
        switch (prm[i][j-1]) {
            case FU1 : setU1(q,j<narg?(uint8_t )stoi(args[j]):0); q+=1; break;
            case FU2 : setU2(q,j<narg?(uint16_t)stoi(args[j]):0); q+=2; break;
            case FU4 : setU4(q,j<narg?(uint32_t)stoi(args[j]):0); q+=4; break;
            case FI1 : setI1(q,j<narg?(int8_t  )stoi(args[j]):0); q+=1; break;
            case FI2 : setI2(q,j<narg?(int16_t )stoi(args[j]):0); q+=2; break;
            case FI4 : setI4(q,j<narg?(int32_t )stoi(args[j]):0); q+=4; break;
            case FR4 : setR4(q,j<narg?(float         )atof(args[j]):0); q+=4; break;
            case FR8 : setR8(q,j<narg?(double)atof(args[j]):0); q+=8; break;
            case FS32: sprintf((char *)q,"%-32.32s",j<narg?args[j]:""); q+=32; break;
            default  : setU1(q,j<narg?(uint8_t )stoi(args[j]):0); q+=1; break;
        }
    }
    
    /* Add VALSET cfgData here */
    if (isvalset) {
        int k;
        
        /* VALSET commands courtesy of gpsd's ubxtool */
        const char *vcmd[]={
            "GEOFENCE-CONFLVL", "GEOFENCE-USE_PIO", "GEOFENCE-PINPOL", "GEOFENCE-PIN", "GEOFENCE-USE_FENCE1", "GEOFENCE-FENCE1_LAT", "GEOFENCE-FENCE1_LON", "GEOFENCE-FENCE1_RAD", "GEOFENCE-USE_FENCE2", "GEOFENCE-FENCE2_LAT", 
            "GEOFENCE-FENCE2_LON", "GEOFENCE-FENCE2_RAD", "GEOFENCE-USE_FENCE3", "GEOFENCE-FENCE3_LAT", "GEOFENCE-FENCE3_LON", "GEOFENCE-FENCE3_RAD", "GEOFENCE-USE_FENCE4", "GEOFENCE-FENCE4_LAT", "GEOFENCE-FENCE4_LON", "GEOFENCE-FENCE4_RAD", 
            "HW-ANT_CFG_VOLTCTRL", "HW-ANT_CFG_SHORTDET", "HW-ANT_CFG_SHORTDET_POL", "HW-ANT_CFG_OPENDET", "HW-ANT_CFG_OPENDET_POL", "HW-ANT_CFG_PWRDOWN", "HW-ANT_CFG_PWRDOWN_POL", "HW-ANT_CFG_RECOVER", "HW-ANT_SUP_SWITCH_PIN", "HW-ANT_SUP_SHORT_PIN", 
            "HW-ANT_SUP_OPEN_PIN", "I2C-ADDRESS", "I2C-EXTENDEDTIMEOUT", "I2C-ENABLED", "I2CINPROT-UBX", "I2CINPROT-NMEA", "I2CINPROT-RTCM2X", "I2CINPROT-RTCM3X", "I2COUTPROT-UBX", "I2COUTPROT-NMEA", 
            "I2COUTPROT-RTCM3X", "INFMSG-UBX_I2C", "INFMSG-UBX_UART1", "INFMSG-UBX_UART2", "INFMSG-UBX_USB", "INFMSG-UBX_SPI", "INFMSG-NMEA_I2C", "INFMSG-NMEA_UART1", "INFMSG-NMEA_UART2", "INFMSG-NMEA_USB", 
            "INFMSG-NMEA_SPI", "ITFM-BBTHRESHOLD", "ITFM-CWTHRESHOLD", "ITFM-ENABLE", "ITFM-ANTSETTING", "ITFM-ENABLE_AUX", "LOGFILTER-RECORD_ENA", "LOGFILTER-ONCE_PER_WAKE_UP_ENA", "LOGFILTER-APPLY_ALL_FILTERS", "LOGFILTER-MIN_INTERVAL", 
            "LOGFILTER-TIME_THRS", "LOGFILTER-SPEED_THRS", "LOGFILTER-POSITION_THRS", "MOT-GNSSSPEED_THRS", "MOT-GNSSDIST_THRS", "MSGOUT-NMEA_ID_DTM_I2C", "MSGOUT-NMEA_ID_DTM_SPI", "MSGOUT-NMEA_ID_DTM_UART1", "MSGOUT-NMEA_ID_DTM_UART2", "MSGOUT-NMEA_ID_DTM_USB", 
            "MSGOUT-NMEA_ID_GBS_I2C", "MSGOUT-NMEA_ID_GBS_SPI", "MSGOUT-NMEA_ID_GBS_UART1", "MSGOUT-NMEA_ID_GBS_UART2", "MSGOUT-NMEA_ID_GBS_USB", "MSGOUT-NMEA_ID_GGA_I2C", "MSGOUT-NMEA_ID_GGA_SPI", "MSGOUT-NMEA_ID_GGA_UART1", "MSGOUT-NMEA_ID_GGA_UART2", "MSGOUT-NMEA_ID_GGA_USB", 
            "MSGOUT-NMEA_ID_GLL_I2C", "MSGOUT-NMEA_ID_GLL_SPI", "MSGOUT-NMEA_ID_GLL_UART1", "MSGOUT-NMEA_ID_GLL_UART2", "MSGOUT-NMEA_ID_GLL_USB", "MSGOUT-NMEA_ID_GNS_I2C", "MSGOUT-NMEA_ID_GNS_SPI", "MSGOUT-NMEA_ID_GNS_UART1", "MSGOUT-NMEA_ID_GNS_UART2", "MSGOUT-NMEA_ID_GNS_USB", 
            "MSGOUT-NMEA_ID_GRS_I2C", "MSGOUT-NMEA_ID_GRS_SPI", "MSGOUT-NMEA_ID_GRS_UART1", "MSGOUT-NMEA_ID_GRS_UART2", "MSGOUT-NMEA_ID_GRS_USB", "MSGOUT-NMEA_ID_GSA_I2C", "MSGOUT-NMEA_ID_GSA_SPI", "MSGOUT-NMEA_ID_GSA_UART1", "MSGOUT-NMEA_ID_GSA_UART2", "MSGOUT-NMEA_ID_GSA_USB", 
            "MSGOUT-NMEA_ID_GST_I2C", "MSGOUT-NMEA_ID_GST_SPI", "MSGOUT-NMEA_ID_GST_UART1", "MSGOUT-NMEA_ID_GST_UART2", "MSGOUT-NMEA_ID_GST_USB", "MSGOUT-NMEA_ID_GSV_I2C", "MSGOUT-NMEA_ID_GSV_SPI", "MSGOUT-NMEA_ID_GSV_UART1", "MSGOUT-NMEA_ID_GSV_UART2", "MSGOUT-NMEA_ID_GSV_USB", 
            "MSGOUT-NMEA_ID_RMC_I2C", "MSGOUT-NMEA_ID_RMC_SPI", "MSGOUT-NMEA_ID_RMC_UART1", "MSGOUT-NMEA_ID_RMC_UART2", "MSGOUT-NMEA_ID_RMC_USB", "MSGOUT-NMEA_ID_VLW_I2C", "MSGOUT-NMEA_ID_VLW_SPI", "MSGOUT-NMEA_ID_VLW_UART1", "MSGOUT-NMEA_ID_VLW_UART2", "MSGOUT-NMEA_ID_VLW_USB", 
            "MSGOUT-NMEA_ID_VTG_I2C", "MSGOUT-NMEA_ID_VTG_SPI", "MSGOUT-NMEA_ID_VTG_UART1", "MSGOUT-NMEA_ID_VTG_UART2", "MSGOUT-NMEA_ID_VTG_USB", "MSGOUT-NMEA_ID_ZDA_I2C", "MSGOUT-NMEA_ID_ZDA_SPI", "MSGOUT-NMEA_ID_ZDA_UART1", "MSGOUT-NMEA_ID_ZDA_UART2", "MSGOUT-NMEA_ID_ZDA_USB", 
            "MSGOUT-PUBX_ID_POLYP_I2C", "MSGOUT-PUBX_ID_POLYP_SPI", "MSGOUT-PUBX_ID_POLYP_UART1", "MSGOUT-PUBX_ID_POLYP_UART2", "MSGOUT-PUBX_ID_POLYP_USB", "MSGOUT-PUBX_ID_POLYS_I2C", "MSGOUT-PUBX_ID_POLYS_SPI", "MSGOUT-PUBX_ID_POLYS_UART1", "MSGOUT-PUBX_ID_POLYS_UART2", "MSGOUT-PUBX_ID_POLYS_USB", 
            "MSGOUT-PUBX_ID_POLYT_I2C", "MSGOUT-PUBX_ID_POLYT_SPI", "MSGOUT-PUBX_ID_POLYT_UART1", "MSGOUT-PUBX_ID_POLYT_UART2", "MSGOUT-PUBX_ID_POLYT_USB", "MSGOUT-RTCM_3X_TYPE1005_I2C", "MSGOUT-RTCM_3X_TYPE1005_SPI", "MSGOUT-RTCM_3X_TYPE1005_UART1", "MSGOUT-RTCM_3X_TYPE1005_UART2", "MSGOUT-RTCM_3X_TYPE1005_USB", 
            "MSGOUT-RTCM_3X_TYPE1074_I2C", "MSGOUT-RTCM_3X_TYPE1074_SPI", "MSGOUT-RTCM_3X_TYPE1074_UART1", "MSGOUT-RTCM_3X_TYPE1074_UART2", "MSGOUT-RTCM_3X_TYPE1074_USB", "MSGOUT-RTCM_3X_TYPE1077_I2C", "MSGOUT-RTCM_3X_TYPE1077_SPI", "MSGOUT-RTCM_3X_TYPE1077_UART1", "MSGOUT-RTCM_3X_TYPE1077_UART2", "MSGOUT-RTCM_3X_TYPE1077_USB", 
            "MSGOUT-RTCM_3X_TYPE1087_I2C", "MSGOUT-RTCM_3X_TYPE1084_SPI", "MSGOUT-RTCM_3X_TYPE1084_UART1", "MSGOUT-RTCM_3X_TYPE1084_UART2", "MSGOUT-RTCM_3X_TYPE1084_USB", "MSGOUT-RTCM_3X_TYPE1087_SPI", "MSGOUT-RTCM_3X_TYPE1087_UART1", "MSGOUT-RTCM_3X_TYPE1087_UART2", "MSGOUT-RTCM_3X_TYPE1087_USB", "MSGOUT-RTCM_3X_TYPE1094_I2C", 
            "MSGOUT-RTCM_3X_TYPE1094_SPI", "MSGOUT-RTCM_3X_TYPE1094_UART1", "MSGOUT-RTCM_3X_TYPE1094_UART2", "MSGOUT-RTCM_3X_TYPE1094_USB", "MSGOUT-RTCM_3X_TYPE1097_I2C", "MSGOUT-RTCM_3X_TYPE1097_SPI", "MSGOUT-RTCM_3X_TYPE1097_UART1", "MSGOUT-RTCM_3X_TYPE1097_UART2", "MSGOUT-RTCM_3X_TYPE1097_USB", "MSGOUT-RTCM_3X_TYPE1124_I2C", 
            "MSGOUT-RTCM_3X_TYPE1124_SPI", "MSGOUT-RTCM_3X_TYPE1124_UART1", "MSGOUT-RTCM_3X_TYPE1124_UART2", "MSGOUT-RTCM_3X_TYPE1124_USB", "MSGOUT-RTCM_3X_TYPE1127_I2C", "MSGOUT-RTCM_3X_TYPE1127_SPI", "MSGOUT-RTCM_3X_TYPE1127_UART1", "MSGOUT-RTCM_3X_TYPE1127_UART2", "MSGOUT-RTCM_3X_TYPE1127_USB", "MSGOUT-RTCM_3X_TYPE1230_I2C", 
            "MSGOUT-RTCM_3X_TYPE1230_SPI", "MSGOUT-RTCM_3X_TYPE1230_UART1", "MSGOUT-RTCM_3X_TYPE1230_UART2", "MSGOUT-RTCM_3X_TYPE1230_USB", "MSGOUT-RTCM_3X_TYPE4072_0_I2C", "MSGOUT-RTCM_3X_TYPE4072_0_SPI", "MSGOUT-RTCM_3X_TYPE4072_0_UART1", "MSGOUT-RTCM_3X_TYPE4072_0_UART2", "MSGOUT-RTCM_3X_TYPE4072_0_USB", "MSGOUT-RTCM_3X_TYPE4072_1_I2C", 
            "MSGOUT-RTCM_3X_TYPE4072_1_SPI", "MSGOUT-RTCM_3X_TYPE4072_1_UART1", "MSGOUT-RTCM_3X_TYPE4072_1_UART2", "MSGOUT-RTCM_3X_TYPE4072_1_USB", "MSGOUT-UBX_LOG_INFO_I2C", "MSGOUT-UBX_LOG_INFO_SPI", "MSGOUT-UBX_LOG_INFO_UART1", "MSGOUT-UBX_LOG_INFO_UART2", "MSGOUT-UBX_LOG_INFO_USB", "MSGOUT-UBX_MON_COMMS_I2C", 
            "MSGOUT-UBX_MON_COMMS_SPI", "MSGOUT-UBX_MON_COMMS_UART1", "MSGOUT-UBX_MON_COMMS_UART2", "MSGOUT-UBX_MON_COMMS_USB", "MSGOUT-UBX_MON_HW2_I2C", "MSGOUT-UBX_MON_HW2_SPI", "MSGOUT-UBX_MON_HW2_UART1", "MSGOUT-UBX_MON_HW2_UART2", "MSGOUT-UBX_MON_HW2_USB", "MSGOUT-UBX_MON_HW3_I2C", 
            "MSGOUT-UBX_MON_HW3_SPI", "MSGOUT-UBX_MON_HW3_UART1", "MSGOUT-UBX_MON_HW3_UART2", "MSGOUT-UBX_MON_HW3_USB", "MSGOUT-UBX_MON_HW_I2C", "MSGOUT-UBX_MON_HW_SPI", "MSGOUT-UBX_MON_HW_UART1", "MSGOUT-UBX_MON_HW_UART2", "MSGOUT-UBX_MON_HW_USB", "MSGOUT-UBX_MON_IO_I2C", 
            "MSGOUT-UBX_MON_IO_SPI", "MSGOUT-UBX_MON_IO_UART1", "MSGOUT-UBX_MON_IO_UART2", "MSGOUT-UBX_MON_IO_USB", "MSGOUT-UBX_MON_MSGPP_I2C", "MSGOUT-UBX_MON_MSGPP_SPI", "MSGOUT-UBX_MON_MSGPP_UART1", "MSGOUT-UBX_MON_MSGPP_UART2", "MSGOUT-UBX_MON_MSGPP_USB", "MSGOUT-UBX_MON_RF_I2C", 
            "MSGOUT-UBX_MON_RF_SPI", "MSGOUT-UBX_MON_RF_UART1", "MSGOUT-UBX_MON_RF_UART2", "MSGOUT-UBX_MON_RF_USB", "MSGOUT-UBX_MON_RXBUF_I2C", "MSGOUT-UBX_MON_RXBUF_SPI", "MSGOUT-UBX_MON_RXBUF_UART1", "MSGOUT-UBX_MON_RXBUF_UART2", "MSGOUT-UBX_MON_RXBUF_USB", "MSGOUT-UBX_MON_RXR_I2C", 
            "MSGOUT-UBX_MON_RXR_SPI", "MSGOUT-UBX_MON_RXR_UART1", "MSGOUT-UBX_MON_RXR_UART2", "MSGOUT-UBX_MON_RXR_USB", "MSGOUT-UBX_MON_TXBUF_I2C", "MSGOUT-UBX_MON_TXBUF_SPI", "MSGOUT-UBX_MON_TXBUF_UART1", "MSGOUT-UBX_MON_TXBUF_UART2", "MSGOUT-UBX_MON_TXBUF_USB", "MSGOUT-UBX_MON_TXBUF_I2C", 
            "MSGOUT-UBX_MON_TXBUF_SPI", "MSGOUT-UBX_MON_TXBUF_UART1", "MSGOUT-UBX_MON_TXBUF_UART2", "MSGOUT-UBX_MON_TXBUF_USB", "MSGOUT-UBX_NAV_CLOCK_I2C", "MSGOUT-UBX_NAV_CLOCK_SPI", "MSGOUT-UBX_NAV_CLOCK_UART1", "MSGOUT-UBX_NAV_CLOCK_UART2", "MSGOUT-UBX_NAV_CLOCK_USB", "MSGOUT-UBX_NAV_DOP_I2C", 
            "MSGOUT-UBX_NAV_DOP_SPI", "MSGOUT-UBX_NAV_DOP_UART1", "MSGOUT-UBX_NAV_DOP_UART2", "MSGOUT-UBX_NAV_DOP_USB", "MSGOUT-UBX_NAV_EOE_I2C", "MSGOUT-UBX_NAV_EOE_SPI", "MSGOUT-UBX_NAV_EOE_UART1", "MSGOUT-UBX_NAV_EOE_UART2", "MSGOUT-UBX_NAV_EOE_USB", "MSGOUT-UBX_NAV_GEOFENCE_I2C", 
            "MSGOUT-UBX_NAV_GEOFENCE_SPI", "MSGOUT-UBX_NAV_GEOFENCE_UART1", "MSGOUT-UBX_NAV_GEOFENCE_UART2", "MSGOUT-UBX_NAV_GEOFENCE_USB", "MSGOUT-UBX_NAV_HPPOSECEF_I2C", "MSGOUT-UBX_NAV_HPPOSECEF_SPI", "MSGOUT-UBX_NAV_HPPOSECEF_UART1", "MSGOUT-UBX_NAV_HPPOSECEF_UART2", "MSGOUT-UBX_NAV_HPPOSECEF_USB", "MSGOUT-UBX_NAV_HPPOSLLH_I2C", 
            "MSGOUT-UBX_NAV_HPPOSLLH_SPI", "MSGOUT-UBX_NAV_HPPOSLLH_UART1", "MSGOUT-UBX_NAV_HPPOSLLH_UART2", "MSGOUT-UBX_NAV_HPPOSLLH_USB", "MSGOUT-UBX_NAV_ODO_I2C", "MSGOUT-UBX_NAV_ODO_SPI", "MSGOUT-UBX_NAV_ODO_UART1", "MSGOUT-UBX_NAV_ODO_UART2", "MSGOUT-UBX_NAV_ODO_USB", "MSGOUT-UBX_NAV_ORB_I2C", 
            "MSGOUT-UBX_NAV_ORB_SPI", "MSGOUT-UBX_NAV_ORB_UART1", "MSGOUT-UBX_NAV_ORB_UART2", "MSGOUT-UBX_NAV_ORB_USB", "MSGOUT-UBX_NAV_POSECEF_I2C", "MSGOUT-UBX_NAV_POSECEF_SPI", "MSGOUT-UBX_NAV_POSECEF_UART1", "MSGOUT-UBX_NAV_POSECEF_UART2", "MSGOUT-UBX_NAV_POSECEF_USB", "MSGOUT-UBX_NAV_POSLLH_I2C", 
            "MSGOUT-UBX_NAV_POSLLH_SPI", "MSGOUT-UBX_NAV_POSLLH_UART1", "MSGOUT-UBX_NAV_POSLLH_UART2", "MSGOUT-UBX_NAV_POSLLH_USB", "MSGOUT-UBX_NAV_PVT_I2C", "MSGOUT-UBX_NAV_PVT_SPI", "MSGOUT-UBX_NAV_PVT_UART1", "MSGOUT-UBX_NAV_PVT_UART2", "MSGOUT-UBX_NAV_PVT_USB", "MSGOUT-UBX_NAV_RELPOSNED_I2C", 
            "MSGOUT-UBX_NAV_RELPOSNED_SPI", "MSGOUT-UBX_NAV_RELPOSNED_UART1", "MSGOUT-UBX_NAV_RELPOSNED_UART2", "MSGOUT-UBX_NAV_RELPOSNED_USB", "MSGOUT-UBX_NAV_SAT_I2C", "MSGOUT-UBX_NAV_SAT_SPI", "MSGOUT-UBX_NAV_SAT_UART1", "MSGOUT-UBX_NAV_SAT_UART2", "MSGOUT-UBX_NAV_SAT_USB", "MSGOUT-UBX_NAV_SBAS_I2C", 
            "MSGOUT-UBX_NAV_SBAS_SPI", "MSGOUT-UBX_NAV_SBAS_UART1", "MSGOUT-UBX_NAV_SBAS_UART2", "MSGOUT-UBX_NAV_SBAS_USB", "MSGOUT-UBX_NAV_SIG_I2C", "MSGOUT-UBX_NAV_SIG_SPI", "MSGOUT-UBX_NAV_SIG_UART1", "MSGOUT-UBX_NAV_SIG_UART2", "MSGOUT-UBX_NAV_SIG_USB", "MSGOUT-UBX_NAV_STATUS_I2C", 
            "MSGOUT-UBX_NAV_STATUS_SPI", "MSGOUT-UBX_NAV_STATUS_UART1", "MSGOUT-UBX_NAV_STATUS_UART2", "MSGOUT-UBX_NAV_STATUS_USB", "MSGOUT-UBX_NAV_SVIN_I2C", "MSGOUT-UBX_NAV_SVIN_SPI", "MSGOUT-UBX_NAV_SVIN_UART1", "MSGOUT-UBX_NAV_SVIN_UART2", "MSGOUT-UBX_NAV_SVIN_USB", "MSGOUT-UBX_NAV_TIMEBDS_I2C", 
            "MSGOUT-UBX_NAV_TIMEBDS_SPI", "MSGOUT-UBX_NAV_TIMEBDS_UART1", "MSGOUT-UBX_NAV_TIMEBDS_UART2", "MSGOUT-UBX_NAV_TIMEBDS_USB", "MSGOUT-UBX_NAV_TIMEGAL_I2C", "MSGOUT-UBX_NAV_TIMEGAL_SPI", "MSGOUT-UBX_NAV_TIMEGAL_UART1", "MSGOUT-UBX_NAV_TIMEGAL_UART2", "MSGOUT-UBX_NAV_TIMEGAL_USB", "MSGOUT-UBX_NAV_TIMEGLO_I2C", 
            "MSGOUT-UBX_NAV_TIMEGLO_SPI", "MSGOUT-UBX_NAV_TIMEGLO_UART1", "MSGOUT-UBX_NAV_TIMEGLO_UART2", "MSGOUT-UBX_NAV_TIMEGLO_USB", "MSGOUT-UBX_NAV_TIMEGPS_I2C", "MSGOUT-UBX_NAV_TIMEGPS_SPI", "MSGOUT-UBX_NAV_TIMEGPS_UART1", "MSGOUT-UBX_NAV_TIMEGPS_UART2", "MSGOUT-UBX_NAV_TIMEGPS_USB", "MSGOUT-UBX_NAV_TIMELS_I2C", 
            "MSGOUT-UBX_NAV_TIMELS_SPI", "MSGOUT-UBX_NAV_TIMELS_UART1", "MSGOUT-UBX_NAV_TIMELS_UART2", "MSGOUT-UBX_NAV_TIMELS_USB", "MSGOUT-UBX_NAV_TIMEUTC_I2C", "MSGOUT-UBX_NAV_TIMEUTC_SPI", "MSGOUT-UBX_NAV_TIMEUTC_UART1", "MSGOUT-UBX_NAV_TIMEUTC_UART2", "MSGOUT-UBX_NAV_TIMEUTC_USB", "MSGOUT-UBX_NAV_VELECEF_I2C", 
            "MSGOUT-UBX_NAV_VELECEF_SPI", "MSGOUT-UBX_NAV_VELECEF_UART1", "MSGOUT-UBX_NAV_VELECEF_UART2", "MSGOUT-UBX_NAV_VELECEF_USB", "MSGOUT-UBX_NAV_VELNED_I2C", "MSGOUT-UBX_NAV_VELNED_SPI", "MSGOUT-UBX_NAV_VELNED_UART1", "MSGOUT-UBX_NAV_VELNED_UART2", "MSGOUT-UBX_NAV_VELNED_USB", "MSGOUT-UBX_RXM_MEASX_I2C", 
            "MSGOUT-UBX_RXM_MEASX_SPI", "MSGOUT-UBX_RXM_MEASX_UART1", "MSGOUT-UBX_RXM_MEASX_UART2", "MSGOUT-UBX_RXM_MEASX_USB", "MSGOUT-UBX_RXM_RAWX_I2C", "MSGOUT-UBX_RXM_RAWX_SPI", "MSGOUT-UBX_RXM_RAWX_UART1", "MSGOUT-UBX_RXM_RAWX_UART2", "MSGOUT-UBX_RXM_RAWX_USB", "MSGOUT-UBX_RXM_RLM_I2C", 
            "MSGOUT-UBX_RXM_RLM_SPI", "MSGOUT-UBX_RXM_RLM_UART1", "MSGOUT-UBX_RXM_RLM_UART2", "MSGOUT-UBX_RXM_RLM_USB", "MSGOUT-UBX_RXM_RTCM_I2C", "MSGOUT-UBX_RXM_RTCM_SPI", "MSGOUT-UBX_RXM_RTCM_UART1", "MSGOUT-UBX_RXM_RTCM_UART2", "MSGOUT-UBX_RXM_RTCM_USB", "MSGOUT-UBX_RXM_SFRBX_I2C", 
            "MSGOUT-UBX_RXM_SFRBX_SPI", "MSGOUT-UBX_RXM_SFRBX_UART1", "MSGOUT-UBX_RXM_SFRBX_UART2", "MSGOUT-UBX_RXM_SFRBX_USB", "MSGOUT-UBX_TIM_SVIN_I2C", "MSGOUT-UBX_TIM_SVIN_SPI", "MSGOUT-UBX_TIM_SVIN_UART1", "MSGOUT-UBX_TIM_SVIN_UART2", "MSGOUT-UBX_TIM_SVIN_USB", "MSGOUT-UBX_TIM_TM2_I2C", 
            "MSGOUT-UBX_TIM_TM2_SPI", "MSGOUT-UBX_TIM_TM2_UART1", "MSGOUT-UBX_TIM_TM2_UART2", "MSGOUT-UBX_TIM_TM2_USB", "MSGOUT-UBX_TIM_TP_I2C", "MSGOUT-UBX_TIM_TP_SPI", "MSGOUT-UBX_TIM_TP_UART1", "MSGOUT-UBX_TIM_TP_UART2", "MSGOUT-UBX_TIM_TP_USB", "MSGOUT-UBX_TIM_VRFY_I2C", 
            "MSGOUT-UBX_TIM_VRFY_SPI", "MSGOUT-UBX_TIM_VRFY_UART1", "MSGOUT-UBX_TIM_VRFY_UART2", "MSGOUT-UBX_TIM_VRFY_USB", "NAVHPG-DGNSSMODE", "NAVSPG-FIXMODE", "NAVSPG-INIFIX3D", "NAVSPG-WKNROLLOVER", "NAVSPG-USE_PPP", "NAVSPG-UTCSTANDARD", 
            "NAVSPG-DYNMODEL", "NAVSPG-ACKAIDING", "NAVSPG-USE_USRDAT", "NAVSPG-USRDAT_MAJA", "NAVSPG-USRDAT_FLAT", "NAVSPG-USRDAT_DX", "NAVSPG-USRDAT_DY", "NAVSPG-USRDAT_DZ", "NAVSPG-USRDAT_ROTX", "NAVSPG-USRDAT_ROTY", 
            "NAVSPG-USRDAT_ROTZ", "NAVSPG-USRDAT_SCALE", "NAVSPG-INFIL_MINSVS", "NAVSPG-INFIL_MAXSVS", "NAVSPG-INFIL_MINCNO", "NAVSPG-INFIL_MINELEV", "NAVSPG-INFIL_NCNOTHRS", "NAVSPG-INFIL_CNOTHRS", "NAVSPG-OUTFIL_PDOP", "NAVSPG-OUTFIL_TDOP", 
            "NAVSPG-OUTFIL_PACC", "NAVSPG-OUTFIL_TACC", "NAVSPG-OUTFIL_FACC", "NAVSPG-CONSTR_ALT", "NAVSPG-CONSTR_ALTVAR", "NAVSPG-CONSTR_DGNSSTO", "NMEA-PROTVER", "NMEA-MAXSVS", "NMEA-COMPAT", "NMEA-CONSIDER", 
            "NMEA-LIMIT82", "NMEA-HIGHPREC", "NMEA-SVNUMBERING", "NMEA-FILT_GPS", "NMEA-FILT_SBAS", "NMEA-FILT_QZSS", "NMEA-FILT_GLO", "NMEA-FILT_BDS", "NMEA-OUT_INVFIX", "NMEA-OUT_MSKFIX", 
            "NMEA-OUT_INVTIME", "NMEA-OUT_INVDATE", "NMEA-OUT_ONLYGPS", "NMEA-OUT_FROZENCOG", "NMEA-MAINTALKERID", "NMEA-GSVTALKERID", "NMEA-BDSTALKERID", "ODO-USE_ODO", "ODO-USE_COG", "ODO-OUTLPVEL", 
            "ODO-OUTLPCOG", "ODO-PROFILE", "ODO-COGMAXSPEED", "ODO-COGMAXPOSACC", "ODO-COGLPGAIN", "ODO-VELLPGAIN", "RATE-MEAS", "RATE-NAV", "RATE-TIMEREF", "RINV-DUMP", 
            "RINV-BINARY", "RINV-DATA_SIZE", "RINV-CHUNK0", "RINV-CHUNK1", "RINV-CHUNK2", "RINV-CHUNK3", "SBAS-USE_TESTMODE", "SBAS-USE_RANGING", "SBAS-USE_DIFFCORR", "SBAS-USE_INTEGRITY", 
            "SBAS-PRNSCANMASK", "SIGNAL-GPS_ENA", "SIGNAL-GPS_L1CA_ENA", "SIGNAL-GPS_L2C_ENA", "SIGNAL-SBAS_ENA", "SIGNAL-SBAS_L1CA_ENA", "SIGNAL-GAL_ENA", "SIGNAL-GAL_E1_ENA", "SIGNAL-GAL_E5B_ENA", "SIGNAL-BDS_ENA", 
            "SIGNAL-BDS_B1_ENA", "SIGNAL-BDS_B2_ENA", "SIGNAL-QZSS_ENA", "SIGNAL-QZSS_L1CA_ENA", "SIGNAL-QZSS_L1S_ENA", "SIGNAL-QZSS_L2C_ENA", "SIGNAL-GLO_ENA", "SIGNAL-GLO_L1_ENA", "SIGNAL-GLO_L2_ENA", "SPI-MAXFF", 
            "SPI-CPOLARITY", "SPI-CPHASE", "SPI-EXTENDEDTIMEOUT", "SPI-ENABLED", "SPIINPROT-UBX", "SPIINPROT-NMEA", "SPIINPROT-RTCM2X", "SPIINPROT-RTCM3X", "SPIOUTPROT-UBX", "SPIOUTPROT-NMEA", 
            "SPIOUTPROT-RTCM3X", "TMODE-MODE", "TMODE-POS_TYPE", "TMODE-ECEF_X", "TMODE-ECEF_Y", "TMODE-ECEF_Z", "TMODE-ECEF_X_HP", "TMODE-ECEF_Y_HP", "TMODE-ECEF_Z_HP", "TMODE-LAT", 
            "TMODE-LON", "TMODE-HEIGHT", "TMODE-LAT_HP", "TMODE-LON_HP", "TMODE-HEIGHT_HP", "TMODE-FIXED_POS_ACC", "TMODE-SVIN_MIN_DUR", "TMODE-SVIN_ACC_LIMIT", "TP-PULSE_DEF", "TP-PULSE_LENGTH_DEF", 
            "TP-ANT_CABLEDELAY", "TP-PERIOD_TP1", "TP-PERIOD_LOCK_TP1", "TP-FREQ_TP1", "TP-FREQ_LOCK_TP1", "TP-LEN_TP1", "TP-LEN_LOCK_TP1", "TP-DUTY_TP1", "TP-DUTY_LOCK_TP1", "TP-USER_DELAY_TP1", 
            "TP-TP1_ENA", "TP-SYNC_GNSS_TP1", "TP-USE_LOCKED_TP1", "TP-ALIGN_TO_TOW_TP1", "TP-POL_TP1", "TP-TIMEGRID_TP1", "TP-PERIOD_TP2", "TP-PERIOD_LOCK_TP2", "TP-FREQ_TP2", "TP-FREQ_LOCK_TP2", 
            "TP-LEN_TP2", "TP-LEN_LOCK_TP2", "TP-DUTY_TP2", "TP-DUTY_LOCK_TP2", "TP-USER_DELAY_TP2", "TP-TP2_ENA", "TP-SYNC_GNSS_TP2", "TP-USE_LOCKED_TP2", "TP-ALIGN_TO_TOW_TP2", "TP-POL_TP2", 
            "TP-TIMEGRID_TP2", "UART1-BAUDRATE", "UART1-STOPBITS", "UART1-DATABITS", "UART1-PARITY", "UART1-ENABLED", "UART1INPROT-UBX", "UART1INPROT-NMEA", "UART1INPROT-RTCM2X", "UART1INPROT-RTCM3X", 
            "UART1OUTPROT-UBX", "UART1OUTPROT-NMEA", "UART1OUTPROT-RTCM3X", "UART2-BAUDRATE", "UART2-STOPBITS", "UART2-DATABITS", "UART2-PARITY", "UART2-ENABLED", "UART2-REMAP", "UART2INPROT-UBX", 
            "UART2INPROT-NMEA", "UART2INPROT-RTCM2X", "UART2INPROT-RTCM3X", "UART2OUTPROT-UBX", "UART2OUTPROT-NMEA", "UART2OUTPROT-RTCM3X", "USB-ENABLED", "USB-SELFPOW", "USB-VENDOR_ID", "USB-PRODUCT_ID", 
            "USB-POWER", "USB-VENDOR_STR0", "USB-VENDOR_STR1", "USB-VENDOR_STR2", "USB-VENDOR_STR3", "USB-PRODUCT_STR0", "USB-PRODUCT_STR1", "USB-PRODUCT_STR2", "USB-PRODUCT_STR3", "USB-SERIAL_NO_STR0", 
            "USB-SERIAL_NO_STR1", "USB-SERIAL_NO_STR2", "USB-SERIAL_NO_STR3", "USBINPROT-UBX", "USBINPROT-NMEA", "USBINPROT-RTCM2X", "USBINPROT-RTCM3X", "USBOUTPROT-UBX", "USBOUTPROT-NMEA", "USBOUTPROT-RTCM3X",""
        };
        const unsigned long vid[]={
            0x20240011,  0x10240012,  0x20240013,  0x20240014,  0x10240020,  0x40240021,  0x40240022,  0x40240023,  0x10240030,  0x40240031,
            0x40240032,  0x40240033,  0x10240040,  0x40240041,  0x40240042,  0x40240043,  0x10240050,  0x40240051,  0x40240052,  0x40240053,
            0x10a3002e,  0x10a3002f,  0x10a30030,  0x10a30031,  0x10a30032,  0x10a30033,  0x10a30034,  0x10a30035,  0x20a30036,  0x20a30037,
            0x20a30038,  0x20510001,  0x10510002,  0x10510003,  0x10710001,  0x10710002,  0x10710003,  0x10710004,  0x10720001,  0x10720002,
            0x10720004,  0x20920001,  0x20920002,  0x20920003,  0x20920004,  0x20920005,  0x20920006,  0x20920007,  0x20920008,  0x20920009,
            0x2092000a,  0x20410001,  0x20410002,  0x1041000d,  0x20410010,  0x10410013,  0x10de0002,  0x10de0003,  0x10de0004,  0x30de0005,
            0x30de0006,  0x30de0007,  0x40de0008,  0x20250038,  0x3025003b,  0x209100a6,  0x209100aa,  0x209100a7,  0x209100a8,  0x209100a9,
            0x209100dd,  0x209100e1,  0x209100de,  0x209100df,  0x209100e0,  0x209100ba,  0x209100be,  0x209100bb,  0x209100bc,  0x209100bd,
            0x209100c9,  0x209100cd,  0x209100ca,  0x209100cb,  0x209100cc,  0x209100b5,  0x209100b9,  0x209100b6,  0x209100b7,  0x209100b8,
            0x209100ce,  0x209100d2,  0x209100cf,  0x209100d0,  0x209100d1,  0x209100bf,  0x209100c3,  0x209100c0,  0x209100c1,  0x209100c2,
            0x209100d3,  0x209100d7,  0x209100d4,  0x209100d5,  0x209100d6,  0x209100c4,  0x209100c8,  0x209100c5,  0x209100c6,  0x209100c7,
            0x209100ab,  0x209100af,  0x209100ac,  0x209100ad,  0x209100ae,  0x209100e7,  0x209100eb,  0x209100e8,  0x209100e9,  0x209100ea,
            0x209100b0,  0x209100b4,  0x209100b1,  0x209100b2,  0x209100b3,  0x209100d8,  0x209100dc,  0x209100d9,  0x209100da,  0x209100db,
            0x209100ec,  0x209100f0,  0x209100ed,  0x209100ee,  0x209100ef,  0x209100f1,  0x209100f5,  0x209100f2,  0x209100f3,  0x209100f4,
            0x209100f6,  0x209100fa,  0x209100f7,  0x209100f8,  0x209100f9,  0x209102bd,  0x209102c1,  0x209102be,  0x209102bf,  0x209102c0,
            0x2091035e,  0x20910362,  0x2091035f,  0x20910360,  0x20910361,  0x209102cc,  0x209102d0,  0x209102cd,  0x209102ce,  0x209102cf,
            0x209102d1,  0x20910367,  0x20910364,  0x20910365,  0x20910366,  0x209102d5,  0x209102d2,  0x209102d3,  0x209102d4,  0x20910368,
            0x2091036c,  0x20910369,  0x2091036a,  0x2091036b,  0x20910318,  0x2091031c,  0x20910319,  0x2091031a,  0x2091031b,  0x2091036d,
            0x20910371,  0x2091036e,  0x2091036f,  0x20910370,  0x209102d6,  0x209102da,  0x209102d7,  0x209102d8,  0x209102d9,  0x20910303,
            0x20910307,  0x20910304,  0x20910305,  0x20910306,  0x209102fe,  0x20910302,  0x209102ff,  0x20910300,  0x20910301,  0x20910381,
            0x20910385,  0x20910382,  0x20910383,  0x20910384,  0x20910259,  0x2091025d,  0x2091025a,  0x2091025b,  0x2091025c,  0x2091034f,
            0x20910353,  0x20910350,  0x20910351,  0x20910352,  0x209101b9,  0x209101bd,  0x209101ba,  0x209101bb,  0x209101bc,  0x20910354,
            0x20910358,  0x20910355,  0x20910356,  0x20910357,  0x209101b4,  0x209101b8,  0x209101b5,  0x209101b6,  0x209101b7,  0x209101a5,
            0x209101a9,  0x209101a6,  0x209101a7,  0x209101a8,  0x20910196,  0x2091019a,  0x20910197,  0x20910198,  0x20910199,  0x20910359,
            0x2091035d,  0x2091035a,  0x2091035b,  0x2091035c,  0x209101a0,  0x209101a4,  0x209101a1,  0x209101a2,  0x209101a3,  0x20910187,
            0x2091018b,  0x20910188,  0x20910189,  0x2091018a,  0x2091019b,  0x2091019f,  0x2091019c,  0x2091019d,  0x2091019e,  0x2091019b,
            0x2091019f,  0x2091019c,  0x2091019d,  0x2091019e,  0x20910065,  0x20910069,  0x20910066,  0x20910067,  0x20910068,  0x20910038,
            0x2091003c,  0x20910039,  0x2091003a,  0x2091003b,  0x2091015f,  0x20910163,  0x20910160,  0x20910161,  0x20910162,  0x209100a1,
            0x209100a5,  0x209100a2,  0x209100a3,  0x209100a4,  0x2091002e,  0x20910032,  0x2091002f,  0x20910030,  0x20910031,  0x20910033,
            0x20910037,  0x20910034,  0x20910035,  0x20910036,  0x2091007e,  0x20910082,  0x2091007f,  0x20910080,  0x20910081,  0x20910010,
            0x20910014,  0x20910011,  0x20910012,  0x20910013,  0x20910024,  0x20910028,  0x20910025,  0x20910026,  0x20910027,  0x20910029,
            0x2091002d,  0x2091002a,  0x2091002b,  0x2091002c,  0x20910006,  0x2091000a,  0x20910007,  0x20910008,  0x20910009,  0x2091008d,
            0x20910091,  0x2091008e,  0x2091008f,  0x20910090,  0x20910015,  0x20910019,  0x20910016,  0x20910017,  0x20910018,  0x2091006a,
            0x2091006e,  0x2091006b,  0x2091006c,  0x2091006d,  0x20910345,  0x20910349,  0x20910346,  0x20910347,  0x20910348,  0x2091001a,
            0x2091001e,  0x2091001b,  0x2091001c,  0x2091001d,  0x20910088,  0x2091008c,  0x20910089,  0x2091008a,  0x2091008b,  0x20910051,
            0x20910055,  0x20910052,  0x20910053,  0x20910054,  0x20910056,  0x2091005a,  0x20910057,  0x20910058,  0x20910059,  0x2091004c,
            0x20910050,  0x2091004d,  0x2091004e,  0x2091004f,  0x20910047,  0x2091004b,  0x20910048,  0x20910049,  0x2091004a,  0x20910060,
            0x20910064,  0x20910061,  0x20910062,  0x20910063,  0x2091005b,  0x2091005f,  0x2091005c,  0x2091005d,  0x2091005e,  0x2091003d,
            0x20910041,  0x2091003e,  0x2091003f,  0x20910040,  0x20910042,  0x20910046,  0x20910043,  0x20910044,  0x20910045,  0x20910204,
            0x20910208,  0x20910205,  0x20910206,  0x20910207,  0x209102a4,  0x209102a8,  0x209102a5,  0x209102a6,  0x209102a7,  0x2091025e,
            0x20910262,  0x2091025f,  0x20910260,  0x20910261,  0x20910268,  0x2091026c,  0x20910269,  0x2091026a,  0x2091026b,  0x20910231,
            0x20910235,  0x20910232,  0x20910233,  0x20910234,  0x20910097,  0x2091009b,  0x20910098,  0x20910099,  0x2091009a,  0x20910178,
            0x2091017c,  0x20910179,  0x2091017a,  0x2091017b,  0x2091017d,  0x20910181,  0x2091017e,  0x2091017f,  0x20910180,  0x20910092,
            0x20910096,  0x20910093,  0x20910094,  0x20910095,  0x20140011,  0x20110011,  0x10110013,  0x30110017,  0x10110019,  0x2011001c,
            0x20110021,  0x10110025,  0x10110061,  0x50110062,  0x50110063,  0x40110064,  0x40110065,  0x40110066,  0x40110067,  0x40110068,
            0x40110069,  0x4011006a,  0x201100a1,  0x201100a2,  0x201100a3,  0x201100a4,  0x201100aa,  0x201100ab,  0x301100b1,  0x301100b2,
            0x301100b3,  0x301100b4,  0x301100b5,  0x401100c1,  0x401100c2,  0x201100c4,  0x20930001,  0x20930002,  0x10930003,  0x10930004,
            0x10930005,  0x10930006,  0x20930007,  0x10930011,  0x10930012,  0x10930015,  0x10930016,  0x10930017,  0x10930021,  0x10930022,
            0x10930023,  0x10930024,  0x10930025,  0x10930026,  0x20930031,  0x20930032,  0x30930033,  0x10220001,  0x10220002,  0x10220003,
            0x10220004,  0x20220005,  0x20220021,  0x20220022,  0x20220032,  0x20220031,  0x30210001,  0x30210002,  0x20210003,  0x10c70001,
            0x10c70002,  0x20c70003,  0x50c70004,  0x50c70005,  0x50c70006,  0x50c70007,  0x10360002,  0x10360003,  0x10360004,  0x10360005,
            0x50360006,  0x1031001f,  0x10310001,  0x10310003,  0x10310020,  0x10310005,  0x10310021,  0x10310007,  0x1031000a,  0x10310022,
            0x1031000d,  0x1031000e,  0x10310024,  0x10310012,  0x10310014,  0x10310015,  0x10310025,  0x10310018,  0x1031001a,  0x20640001,
            0x10640002,  0x10640003,  0x10640005,  0x10640006,  0x10790001,  0x10790002,  0x10790003,  0x10790004,  0x107a0001,  0x107a0002,
            0x107a0004,  0x20030001,  0x20030002,  0x40030003,  0x40030004,  0x40030005,  0x20030006,  0x20030007,  0x20030008,  0x40030009,
            0x4003000a,  0x4003000b,  0x2003000c,  0x2003000d,  0x2003000e,  0x4003000f,  0x40030010,  0x40030011,  0x20050023,  0x20050030,
            0x30050001,  0x40050002,  0x40050003,  0x40050024,  0x40050025,  0x40050004,  0x40050005,  0x5005002a,  0x5005002b,  0x40050006,
            0x10050007,  0x10050008,  0x10050009,  0x1005000a,  0x1005000b,  0x2005000c,  0x4005000d,  0x4005000e,  0x40050026,  0x40050027,
            0x4005000f,  0x40050010,  0x5005002c,  0x5005002d,  0x40050011,  0x10050012,  0x10050013,  0x10050014,  0x10050015,  0x10050016,
            0x20050017,  0x40520001,  0x20520002,  0x20520003,  0x20520004,  0x10520005,  0x10730001,  0x10730002,  0x10730003,  0x10730004,
            0x10740001,  0x10740002,  0x10740004,  0x40530001,  0x20530002,  0x20530003,  0x20530004,  0x10530005,  0x10530006,  0x10750001,
            0x10750002,  0x10750003,  0x10750004,  0x10760001,  0x10760002,  0x10760004,  0x10650001,  0x10650002,  0x3065000a,  0x3065000b,
            0x3065000c,  0x5065000d,  0x5065000e,  0x5065000f,  0x50650010,  0x50650011,  0x50650012,  0x50650013,  0x50650014,  0x50650015,
            0x50650016,  0x50650017,  0x50650018,  0x10770001,  0x10770002,  0x10770003,  0x10770004,  0x10780001,  0x10780002,  0x10780004
        };
        const int vprm[]={
            FU1, FU1, FU1, FU1, FU1, FI4, FI4, FU4, FU1, FI4,
            FI4, FU4, FU1, FI4, FI4, FU4, FU1, FI4, FI4, FU4,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU2,
            FU2, FU2, FU4, FU1, FU2, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU2, FU1, FU1,
            FU1, FU1, FU1, FR8, FR8, FR4, FR4, FR4, FR4, FR4,
            FR4, FR4, FU1, FU1, FU1, FI1, FU1, FU1, FU2, FU2,
            FU2, FU2, FU2, FI4, FU4, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU2, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU2, FU2, FU1, FU1,
            FU1, FU1, FU8, FU8, FU8, FU8, FU1, FU1, FU1, FU1,
            FU8, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FI4, FI4, FI4, FI1, FI1, FI1, FI4,
            FI4, FI4, FI1, FI1, FI1, FU4, FU4, FU4, FU1, FU1,
            FI2, FU4, FU4, FU4, FU4, FU4, FU4, FR8, FR8, FI4,
            FU1, FU1, FU1, FU1, FU1, FU1, FU4, FU4, FU4, FU4,
            FU4, FU4, FR8, FR8, FI4, FU1, FU1, FU1, FU1, FU1,
            FU1, FU4, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU4, FU1, FU1, FU1, FU1, FU1, FU1,
            FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU1, FU2, FU2,
            FU2, FU8, FU8, FU8, FU8, FU8, FU8, FU8, FU8, FU8,
            FU8, FU8, FU8, FU1, FU1, FU1, FU1, FU1, FU1, FU1
        };

        if (strncmp(args[j],"CFG-",4)) return 0;

        for (k=0;*vcmd[k];k++) {
            if (!strcmp(args[j]+4,vcmd[k])) break;
        }

        if (!*vcmd[k]) return 0;

        setU4(q,(unsigned long) vid[k]);
        q+=4;

        /* Set value */
        switch (vprm[k]) {
            case FU1 : setU1(q,(unsigned char )atoi(args[j+1])); q+=1; break;
            case FU2 : setU2(q,(unsigned short)atoi(args[j+1])); q+=2; break;
            case FU4 : setU4(q,(unsigned long )atoi(args[j+1])); q+=4; break;
            /* case FU8 : setU8(q,(unsigned long long)atoi(args[j+2])); q+=8; break; */
            case FI1 : setI1(q,(signed char   )atoi(args[j+1])); q+=1; break;
            case FI2 : setI2(q,(signed short  )atoi(args[j+1])); q+=2; break;
            case FI4 : setI4(q,(signed long   )atoi(args[j+1])); q+=4; break;
            case FR4 : setR4(q,(float         )atof(args[j+1])); q+=4; break;
            case FR8 : setR8(q,(double        )atof(args[j+1])); q+=8; break;
            case FS32: sprintf((char *)q,"%-32.32s",args[j+1]); q+=32; break;
            default  : setU1(q,(unsigned char )atoi(args[j+1])); q+=1; break;
        }

    }
    n=(int)(q-buff)+2;
    setU2(buff+4,(unsigned short)(n-8));
    setcs(buff,n);
    
    trace(5,"gen_ubx: buff=\n"); traceb(5,buff,n);
    return n;
}
