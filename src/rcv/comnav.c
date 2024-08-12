/*------------------------------------------------------------------------------
* comnav.c : ComNav receiver functions
*
*          Copyright (C) 2007-2017 by T.TAKASU, All rights reserved.
*
* reference :
*     [1] ComNav, CNT-OEM-RM001, Rev 1.5 COMNAV OEM BOARD REFERENCE MANUAL, 2018

*
* history : 2007/10/08 1.0 new

*-----------------------------------------------------------------------------*/
#include "rtklib.h"

#define CNAVSYNC1   0xAA        /* cnav message start sync code 1 */
#define CNAVSYNC2   0x44        /* cnav message start sync code 2 */
#define CNAVSYNC3   0x12        /* cnav message start sync code 3 */

#define CNAVHLEN    28          /* cnav message header length (bytes) */

#define ID_ALMANAC  73          /* message id: cnav decoded almanac */
#define ID_GLOALMANAC 718       /* message id: cnav glonass decoded almanac */
#define ID_GLOEPHEMERIS 723     /* message id: cnav glonass ephemeris */
#define ID_IONUTC   8           /* message id: cnav iono and utc data */
#define ID_RANGE    43          /* message id: cnav range measurement */
#define ID_RANGECMP 140         /* message id: cnav range compressed */
#define ID_RAWALM   74          /* message id: cnav raw almanac */
#define ID_RAWEPHEM 41          /* message id: cnav raw ephemeris */
#define ID_RAWWAASFRAME 287     /* message id: cnav raw waas frame */

#define ID_QZSSIONUTC 1347      /* message id: oem6 qzss ion/utc parameters */
#define ID_QZSSRAWEPHEM 1330    /* message id: oem6 qzss raw ephemeris */
#define ID_QZSSRAWSUBFRAME 1331 /* message id: oem6 qzss raw subframe */
#define ID_RAWSBASFRAME 973     /* message id: oem6 raw sbas frame */
#define ID_GALEPHEMERIS 1122    /* message id: oem6 decoded galileo ephemeris */
#define ID_GALALMANAC 1120      /* message id: oem6 decoded galileo almanac */
#define ID_GALCLOCK 1121        /* message id: oem6 galileo clockinformation */
#define ID_GALIONO  1127        /* message id: oem6 decoded galileo iono corrections */
#define ID_GALFNAVRAWPAGE 1413  /* message id: oem6 raw galileo f/nav paga data */
#define ID_GALINAVRAWWORD 1414  /* message id: oem6 raw galileo i/nav word data */
#define ID_RAWCNAVFRAME 1066    /* message id: oem6 raw cnav frame data */
#define ID_BDSEPHEMERIS 1696    /* message id: oem6 decoded bds ephemeris */

#define WL1         0.1902936727984
#define WL2         0.2442102134246
#define MAXVAL      8388608.0

#define OFF_FRQNO   -7          /* F/W ver.3.620 */

/* get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((unsigned char *)(p)))
#define I1(p) (*((signed char *)(p)))
static unsigned short U2(unsigned char *p) {unsigned short u; memcpy(&u,p,2); return u;}
static unsigned int   U4(unsigned char *p) {unsigned int   u; memcpy(&u,p,4); return u;}
static int            I4(unsigned char *p) {int            i; memcpy(&i,p,4); return i;}
static float          R4(unsigned char *p) {float          r; memcpy(&r,p,4); return r;}
static double         R8(unsigned char *p) {double         r; memcpy(&r,p,8); return r;}

/* extend sign ---------------------------------------------------------------*/
static int exsign(unsigned int v, int bits)
{
    return (int)(v&(1<<(bits-1))?v|(~0u<<bits):v);
}
/* adjust weekly rollover of gps time ----------------------------------------*/
static gtime_t adjweek(gtime_t time, double tow)
{
    double tow_p;
    int week;
    tow_p=time2gpst(time,&week);
    if      (tow<tow_p-302400.0) tow+=604800.0;
    else if (tow>tow_p+302400.0) tow-=604800.0;
    return gpst2time(week,tow);
}
/* get observation data index ------------------------------------------------*/
static int obsindex(obs_t *obs, gtime_t time, int sat)
{
    int i,j;
    
    if (obs->n>=MAXOBS) return -1;
    for (i=0;i<obs->n;i++) {
        if (obs->data[i].sat==sat) return i;
    }
    obs->data[i].time=time;
    obs->data[i].sat=sat;
    for (j=0;j<NFREQ+NEXOBS;j++) {
        obs->data[i].L[j]=obs->data[i].P[j]=0.0;
        obs->data[i].D[j]=0.0;
        obs->data[i].SNR[j]=obs->data[i].LLI[j]=0;
        obs->data[i].code[j]=CODE_NONE;
    }
    obs->n++;
    return i;
}
/* decode cnav tracking status -------------------------------------------------
* deocode cnav tracking status
* args   : unsigned int stat I  tracking status field
*          int    *sys   O      system (SYS_???)
*          int    *code  O      signal code (CODE_L??)
*          int    *track O      tracking state
*                         (cnav/5)
*                         0=L1 idle                   8=L2 idle
*                         1=L1 sky search             9=L2 p-code align
*                         2=L1 wide freq pull-in     10=L2 search
*                         3=L1 narrow freq pull-in   11=L2 pll
*                         4=L1 pll                   12=L2 steering
*                         5=L1 reacq
*                         6=L1 steering
*                         7=L1 fll
*                         (oem6)
*                         0=idle                      7=freq-lock loop
*                         2=wide freq band pull-in    9=channel alignment
*                         3=narrow freq band pull-in 10=code search
*                         4=phase lock loop          11=aided phase lock loop
*          int    *plock O      phase-lock flag   (0=not locked, 1=locked)
*          int    *clock O      code-lock flag    (0=not locked, 1=locked)
*          int    *parity O     parity known flag (0=not known,  1=known)
*          int    *halfc O      phase measurement (0=half-cycle not added,
*                                                  1=added)
* return : signal frequency (0:L1,1:L2,2:L5,3:L6,4:L7,5:L8,-1:error)
* notes  : refer [1][3]
*-----------------------------------------------------------------------------*/
static int decode_trackstat(unsigned int stat, int *sys, int *code, int *track,
                            int *plock, int *clock, int *parity, int *halfc)
{
    int satsys,sigtype,freq=0;
    
    *track =stat&0x1F;
    *plock =(stat>>10)&1;
    *parity=(stat>>11)&1;
    *clock =(stat>>12)&1;
    satsys =(stat>>16)&7;
    *halfc =(stat>>28)&1;
    sigtype=(stat>>21)&0x1F;
    
    switch (satsys) {
        case 0: *sys=SYS_GPS; break;
        case 1: *sys=SYS_GLO; break;
        case 2: *sys=SYS_SBS; break;
        case 3: *sys=SYS_GAL; break; /* OEM6 */
        case 4: *sys=SYS_CMP; break; /* OEM6 F/W 6.400 */
        case 5: *sys=SYS_QZS; break; /* OEM6 */
        default:
            trace(2,"cnav unknown system: sys=%d\n",satsys);
            return -1;
    }
    if (*sys==SYS_GPS||*sys==SYS_QZS) {
        switch (sigtype) {
            case  0: freq=0; *code=CODE_L1C; break; /* L1C/A */
            case  2: freq=2; *code=CODE_L5I; break; /* L5 */
            case  5: freq=0; *code=CODE_L1P; break; /* L1P */
            case  9: freq=1; *code=CODE_L2D; break; /* L2P codeless */
            case 14: freq=2; *code=CODE_L5I; break; /* L5I */
            case 17: freq=1; *code=CODE_L2X; break; /* L2C(M+L) */
            default: freq=-1; break;
        }
    }
    else if (*sys==SYS_GLO) {
        switch (sigtype) {
            case  0: freq=0; *code=CODE_L1C; break; /* L1C/A */
            case  1: freq=1; *code=CODE_L2C; break; /* L2C/A (OEM6) */
            case  5: freq=1; *code=CODE_L2C; break; /* L2C */
            default: freq=-1; break;
        }
    }
    else if (*sys==SYS_GAL) {
        switch (sigtype) {
            case  1: freq=0; *code=CODE_L1B; break; /* E1B  (OEM6) */
            case  2: freq=0; *code=CODE_L1X; break; /* E1C  (OEM6) */
            case 12: freq=2; *code=CODE_L5X; break; /* E5aQ (OEM6) */
            case 17: freq=4; *code=CODE_L7X; break; /* E5bQ (OEM6) */
            case 20: freq=5; *code=CODE_L8X; break; /* AltBOCQ (OEM6) */
            default: freq=-1; break;
        }
    }
    else if (*sys==SYS_CMP) {
        switch (sigtype) {
            case  0: freq=0; *code=CODE_L1I; break; /* B1 with D1 (OEM6) */
            case  1: freq=1; *code=CODE_L7I; break; /* B2 with D1 (OEM6) */
            case  4: freq=0; *code=CODE_L1I; break; /* B1 with D2 (OEM6) */
            case  5: freq=1; *code=CODE_L7I; break; /* B2 with D2 (OEM6) */
            default: freq=-1; break;
        }
    }
    else if (*sys==SYS_SBS) {
        switch (sigtype) {
            case  0: freq=0; *code=CODE_L1C; break; /* L1C/A */
            case  6: freq=2; *code=CODE_L5I; break; /* L5I (OEM6) */
            default: freq=-1; break;
        }
    }
    if (freq<0) {
        trace(2,"cnav signal type error: sys=%d sigtype=%d\n",*sys,sigtype);
        return -1;
    }
    return freq;
}
/* check code priority and return obs position -------------------------------*/
static int checkpri(const char *opt, int sys, int code, int freq)
{
    int nex=NEXOBS; /* number of extended obs data */
    
    if (sys==SYS_GPS) {
        if (strstr(opt,"-GL1P")&&freq==0) return code==CODE_L1P?0:-1;
        if (strstr(opt,"-GL2X")&&freq==1) return code==CODE_L2X?1:-1;
        if (code==CODE_L1P) return nex<1?-1:NFREQ;
        if (code==CODE_L2X) return nex<2?-1:NFREQ+1;
    }
    else if (sys==SYS_GLO) {
        if (strstr(opt,"-RL2C")&&freq==1) return code==CODE_L2C?1:-1;
        if (code==CODE_L2C) return nex<1?-1:NFREQ;
    }
    else if (sys==SYS_GAL) {
        if (strstr(opt,"-EL1B")&&freq==0) return code==CODE_L1B?0:-1;
        if (code==CODE_L1B) return nex<1?-1:NFREQ;
        if (code==CODE_L7Q) return nex<2?-1:NFREQ+1;
        if (code==CODE_L8Q) return nex<3?-1:NFREQ+2;
    }
    return freq<NFREQ?freq:-1;
}
/* decode rangecmpb ----------------------------------------------------------*/
static int decode_rangecmpb(raw_t *raw)
{
    double psr,adr,adr_rolls,lockt,tt,dop,snr,wavelen;
    int i,index,nobs,prn,sat,sys,code,freq,pos;
    int track,plock,clock,parity,halfc,lli;
    char *msg;
    unsigned char *p=raw->buff+CNAVHLEN;
    
    trace(3,"decode_rangecmpb: len=%d\n",raw->len);
    
    nobs=U4(p);
    
    if (raw->outtype) {
        msg=raw->msgtype+strlen(raw->msgtype);
        sprintf(msg," nobs=%2d",nobs);
    }
    if (raw->len<CNAVHLEN+4+nobs*24) {
        trace(2,"cnav rangecmpb length error: len=%d nobs=%d\n",raw->len,nobs);
        return -1;
    }
    for (i=0,p+=4;i<nobs;i++,p+=24) {
        
        /* decode tracking status */
        if ((freq=decode_trackstat(U4(p),&sys,&code,&track,&plock,&clock,
                                   &parity,&halfc))<0) continue;
        
        /* obs position */
        if ((pos=checkpri(raw->opt,sys,code,freq))<0) continue;
        
        prn=U1(p+17);
        if (sys==SYS_GLO) prn-=37;
        
        if (!(sat=satno(sys,prn))) {
            trace(3,"cnav rangecmpb satellite number error: sys=%d,prn=%d\n",sys,prn);
            continue;
        }
        if (sys==SYS_GLO&&!parity) continue; /* invalid if GLO parity unknown */
        
        dop=exsign(U4(p+4)&0xFFFFFFF,28)/256.0;
        psr=(U4(p+7)>>4)/128.0+U1(p+11)*2097152.0;
        
        if ((wavelen=satwavelen(sat,freq,&raw->nav))<=0.0) {
            if (sys==SYS_GLO) wavelen=CLIGHT/(freq==0?FREQ1_GLO:FREQ2_GLO);
            else wavelen=lam_carr[freq];
        }
        adr=I4(p+12)/256.0;
        adr_rolls=(psr/wavelen+adr)/MAXVAL;
        adr=-adr+MAXVAL*floor(adr_rolls+(adr_rolls<=0?-0.5:0.5));
        
        lockt=(U4(p+18)&0x1FFFFF)/32.0; /* lock time */
        if (lockt<2) parity=0; /* pseudo-parity */
        
        if (raw->tobs[sat-1][pos].time!=0) {
            tt=timediff(raw->time,raw->tobs[sat-1][pos]);
            lli=(lockt<65535.968&&lockt-raw->lockt[sat-1][pos]+0.05<=tt)?LLI_SLIP:0;
        }
        else {
            lli=0;
        }

        if (!parity) lli|=LLI_HALFC;
#if 0
        if (halfc  ) lli|=LLI_HALFA;
#else
        if (halfc!=raw->halfc[sat-1][pos]) lli|=LLI_SLIP;
#endif
        raw->tobs [sat-1][pos]=raw->time;
        raw->lockt[sat-1][pos]=lockt;
        raw->halfc[sat-1][pos]=halfc;
        
        snr=((U2(p+20)&0x3FF)>>5)+20.0;
        if ((sys!=SYS_GAL&&!clock)||(sys==SYS_GAL&&!plock)) psr=0.0;     /* code unlock */
        if (!plock) adr=dop=0.0; /* phase unlock */
        
        if (fabs(timediff(raw->obs.data[0].time,raw->time))>1E-9) {
            raw->obs.n=0;
        }
        if ((index=obsindex(&raw->obs,raw->time,sat))>=0) {
            raw->obs.data[index].L  [pos]=adr;
            raw->obs.data[index].P  [pos]=psr;
            raw->obs.data[index].D  [pos]=(float)dop;
            raw->obs.data[index].SNR[pos]=
                0.0<=snr&&snr<255.0?(unsigned char)(snr*4.0+0.5):0;
            raw->obs.data[index].LLI[pos]=(unsigned char)lli;
            raw->obs.data[index].code[pos]=code;
#if 0
            /* L2C phase shift correction (L2C->L2P) */
            if (code==CODE_L2X) {
                raw->obs.data[index].L[pos]+=0.25;
                trace(3,"cnav L2C phase shift corrected: prn=%2d\n",prn);
            }
#endif
        }
    }
    return 1;
}
/* decode rangeb -------------------------------------------------------------*/
static int decode_rangeb(raw_t *raw)
{
    double psr,adr,dop,snr,lockt,tt;
    char *msg;
    int i,index,nobs,prn,sat,sys,code,freq,pos;
    int track,plock,clock,parity,halfc,lli,gfrq;
    unsigned char *p=raw->buff+CNAVHLEN;
    
    trace(3,"decode_rangeb: len=%d\n",raw->len);
    
    nobs=U4(p);
    
    if (raw->outtype) {
        msg=raw->msgtype+strlen(raw->msgtype);
        sprintf(msg," nobs=%2d",nobs);
    }
    if (raw->len<CNAVHLEN+4+nobs*44) {
        trace(2,"cnav rangeb length error: len=%d nobs=%d\n",raw->len,nobs);
        return -1;
    }
    for (i=0,p+=4;i<nobs;i++,p+=44) {
        
        /* decode tracking status */
        if ((freq=decode_trackstat(U4(p+40),&sys,&code,&track,&plock,&clock,
                                   &parity,&halfc))<0) continue;
        
        /* obs position */
        if ((pos=checkpri(raw->opt,sys,code,freq))<0) continue;
        
        prn=U2(p);
        if (sys==SYS_GLO) prn-=37;
        
        if (!(sat=satno(sys,prn))) {
            trace(3,"cnav rangeb satellite number error: sys=%d,prn=%d\n",sys,prn);
            continue;
        }
        if (sys==SYS_GLO&&!parity) continue; /* invalid if GLO parity unknown */
        
        gfrq =U2(p+ 2);
        psr  =R8(p+ 4);
        adr  =R8(p+16);
        dop  =R4(p+28);
        snr  =R4(p+32);
        lockt=R4(p+36);
        if (lockt<2) parity=0; /* pseudo-parity */
        
        /* set glonass frequency channel number */
        if (sys==SYS_GLO&&raw->nav.geph[prn-1].sat!=sat) {
            raw->nav.geph[prn-1].frq=gfrq-7;
        }
        if (raw->tobs[sat-1][pos].time!=0) {
            tt=timediff(raw->time,raw->tobs[sat-1][pos]);
            lli=lockt-raw->lockt[sat-1][pos]+0.05<=tt?LLI_SLIP:0;
        }
        else {
            lli=0;
        }
        if (!parity) lli|=LLI_HALFC;
        if (halfc  ) lli|=LLI_HALFA;
        raw->tobs [sat-1][pos]=raw->time;
        raw->lockt[sat-1][pos]=lockt;
        raw->halfc[sat-1][pos]=halfc;
        
        if (!clock) psr=0.0;     /* code unlock */
        if (!plock) adr=dop=0.0; /* phase unlock */
        
        if (fabs(timediff(raw->obs.data[0].time,raw->time))>1E-9) {
            raw->obs.n=0;
        }
        if ((index=obsindex(&raw->obs,raw->time,sat))>=0) {
            raw->obs.data[index].L  [pos]=-adr;
            raw->obs.data[index].P  [pos]=psr;
            raw->obs.data[index].D  [pos]=(float)dop;
            raw->obs.data[index].SNR[pos]=
                0.0<=snr&&snr<255.0?(unsigned char)(snr*4.0+0.5):0;
            raw->obs.data[index].LLI[pos]=(unsigned char)lli;
            raw->obs.data[index].code[pos]=code;
#if 0
            /* L2C phase shift correction */
            if (code==CODE_L2X) {
                raw->obs.data[index].L[pos]+=0.25;
                trace(3,"cnav L2C phase shift corrected: prn=%2d\n",prn);
            }
#endif
        }
    }
    return 1;
}
/* decode rawephemb ----------------------------------------------------------*/
static int decode_rawephemb(raw_t *raw)
{
    unsigned char *p=raw->buff+CNAVHLEN;
    eph_t eph={0};
    int prn,sat;
    
    trace(3,"decode_rawephemb: len=%d\n",raw->len);
    
    if (raw->len<CNAVHLEN+102) {
        trace(2,"cnav rawephemb length error: len=%d\n",raw->len);
        return -1;
    }
    prn=U4(p);
    if (!(sat=satno(SYS_GPS,prn))) {
        trace(2,"cnav rawephemb satellite number error: prn=%d\n",prn);
        return -1;
    }
    if (decode_frame(p+ 12,&eph,NULL,NULL,NULL,NULL)!=1||
        decode_frame(p+ 42,&eph,NULL,NULL,NULL,NULL)!=2||
        decode_frame(p+ 72,&eph,NULL,NULL,NULL,NULL)!=3) {
        trace(2,"cnav rawephemb subframe error: prn=%d\n",prn);
        return -1;
    }
    if (!strstr(raw->opt,"-EPHALL")) {
        if (eph.iode==raw->nav.eph[sat-1].iode) return 0; /* unchanged */
    }
    eph.sat=sat;
    raw->nav.eph[sat-1]=eph;
    raw->ephsat=sat;
    trace(4,"decode_rawephemb: sat=%2d\n",sat);
    return 2;
}
/* decode ionutcb ------------------------------------------------------------*/
static int decode_ionutcb(raw_t *raw)
{
    unsigned char *p=raw->buff+CNAVHLEN;
    int i;
    
    trace(3,"decode_ionutcb: len=%d\n",raw->len);
    
    if (raw->len<CNAVHLEN+108) {
        trace(2,"cnav ionutcb length error: len=%d\n",raw->len);
        return -1;
    }
    for (i=0;i<8;i++) raw->nav.ion_gps[i]=R8(p+i*8);
    raw->nav.utc_gps[0]=R8(p+72);
    raw->nav.utc_gps[1]=R8(p+80);
    raw->nav.utc_gps[2]=U4(p+68);
    raw->nav.utc_gps[3]=U4(p+64);
    raw->nav.leaps =I4(p+96);
    return 9;
}
/* decode rawwaasframeb ------------------------------------------------------*/
static int decode_rawwaasframeb(raw_t *raw)
{
    unsigned char *p=raw->buff+CNAVHLEN;
    int i,prn;
    
    trace(3,"decode_rawwaasframeb: len=%d\n",raw->len);
    
    if (raw->len<CNAVHLEN+48) {
        trace(2,"cnav rawwaasframeb length error: len=%d\n",raw->len);
        return -1;
    }
    prn=U4(p+4);
    
    if (MINPRNQZS_S<=prn&&prn<=MAXPRNQZS_S) {
        prn+=10; /* QZSS SAIF PRN -> QZSS PRN */
    }
    else if (prn<MINPRNSBS||MAXPRNSBS<prn) return 0;
    
    raw->sbsmsg.tow=(int)time2gpst(raw->time,&raw->sbsmsg.week);
    raw->sbsmsg.prn=prn;
    for (i=0,p+=12;i<29;i++,p++) raw->sbsmsg.msg[i]=*p;
    return 3;
}
/* decode rawsbasframeb ------------------------------------------------------*/
static int decode_rawsbasframeb(raw_t *raw)
{
    trace(3,"decode_rawsbasframeb: len=%d\n",raw->len);
    
    /* format same as rawwaasframeb */
    return decode_rawwaasframeb(raw);
}
/* decode gloephemerisb ------------------------------------------------------*/
static int decode_gloephemerisb(raw_t *raw)
{
    unsigned char *p=raw->buff+CNAVHLEN;
    geph_t geph={0};
    char *msg;
    double tow,tof,toff;
    int prn,sat,week;
    
    trace(3,"decode_gloephemerisb: len=%d\n",raw->len);
    
    if (raw->len<CNAVHLEN+144) {
        trace(2,"cnav gloephemerisb length error: len=%d\n",raw->len);
        return -1;
    }
    prn        =U2(p)-37;
    
    if (raw->outtype) {
        msg=raw->msgtype+strlen(raw->msgtype);
        sprintf(msg," prn=%3d",prn);
    }
    if (!(sat=satno(SYS_GLO,prn))) {
        trace(2,"cnav gloephemerisb prn error: prn=%d\n",prn);
        return -1;
    }
    geph.frq   =U2(p+  2)+OFF_FRQNO;
    week       =U2(p+  6);
    tow        =floor(U4(p+8)/1000.0+0.5); /* rounded to integer sec */
    toff       =U4(p+ 12);
    geph.iode  =U4(p+ 20)&0x7F;
    geph.svh   =U4(p+ 24);
    geph.pos[0]=R8(p+ 28);
    geph.pos[1]=R8(p+ 36);
    geph.pos[2]=R8(p+ 44);
    geph.vel[0]=R8(p+ 52);
    geph.vel[1]=R8(p+ 60);
    geph.vel[2]=R8(p+ 68);
    geph.acc[0]=R8(p+ 76);
    geph.acc[1]=R8(p+ 84);
    geph.acc[2]=R8(p+ 92);
    geph.taun  =R8(p+100);
    geph.gamn  =R8(p+116);
    tof        =U4(p+124)-toff; /* glonasst->gpst */
    geph.age   =U4(p+136);
    geph.toe=gpst2time(week,tow);
    tof+=floor(tow/86400.0)*86400;
    if      (tof<tow-43200.0) tof+=86400.0;
    else if (tof>tow+43200.0) tof-=86400.0;
    geph.tof=gpst2time(week,tof);
    
    if (!strstr(raw->opt,"-EPHALL")) {
        if (fabs(timediff(geph.toe,raw->nav.geph[prn-1].toe))<1.0&&
            geph.svh==raw->nav.geph[prn-1].svh) return 0; /* unchanged */
    }
    geph.sat=sat;
    raw->nav.geph[prn-1]=geph;
    raw->ephsat=sat;
    return 2;
}
/* decode qzss rawephemb -----------------------------------------------------*/
static int decode_qzssrawephemb(raw_t *raw)
{
    unsigned char *p=raw->buff+CNAVHLEN,*q;
    eph_t eph={0};
    char *msg;
    int i,prn,id,sat;
    
    trace(3,"decode_qzssrawephemb: len=%d\n",raw->len);
    
    if (raw->len<CNAVHLEN+44) {
        trace(2,"cnav qzssrawephemb length error: len=%d\n",raw->len);
        return -1;
    }
    prn=U4(p);
    id =U4(p+4);
    
    if (raw->outtype) {
        msg=raw->msgtype+strlen(raw->msgtype);
        sprintf(msg," prn=%3d id=%d",prn,id);
    }
    if (!(sat=satno(SYS_QZS,prn))) {
        trace(2,"cnav qzssrawephemb satellite number error: prn=%d\n",prn);
        return -1;
    }
    if (id<1||3<id) return 0;
    
    q=raw->subfrm[sat-1]+(id-1)*30;
    for (i=0;i<30;i++) *q++=p[8+i];
    
    if (id<3) return 0;
    if (decode_frame(raw->subfrm[sat-1]   ,&eph,NULL,NULL,NULL,NULL)!=1||
        decode_frame(raw->subfrm[sat-1]+30,&eph,NULL,NULL,NULL,NULL)!=2||
        decode_frame(raw->subfrm[sat-1]+60,&eph,NULL,NULL,NULL,NULL)!=3) {
        return 0;
    }
    if (!strstr(raw->opt,"-EPHALL")) {
        if (eph.iodc==raw->nav.eph[sat-1].iodc&&
            eph.iode==raw->nav.eph[sat-1].iode) return 0; /* unchanged */
    }
    eph.sat=sat;
    raw->nav.eph[sat-1]=eph;
    raw->ephsat=sat;
    trace(4,"decode_qzssrawephemb: sat=%2d\n",sat);
    return 2;
}
/* decode qzss rawsubframeb --------------------------------------------------*/
static int decode_qzssrawsubframeb(raw_t *raw)
{
    unsigned char *p=raw->buff+CNAVHLEN;
    eph_t eph={0};
    char *msg;
    int prn,sat;
    
    trace(3,"decode_qzssrawsubframeb: len=%d\n",raw->len);
    
    if (raw->len<CNAVHLEN+44) {
        trace(2,"cnav qzssrawsubframeb length error: len=%d\n",raw->len);
        return -1;
    }
    prn=U4(p);
    
    if (raw->outtype) {
        msg=raw->msgtype+strlen(raw->msgtype);
        sprintf(msg," prn=%3d",prn);
    }
    if (!(sat=satno(SYS_QZS,prn))) {
        trace(2,"cnav qzssrawephemb satellite number error: prn=%d\n",prn);
        return -1;
    }
    if (decode_frame(p+12,&eph,NULL,NULL,NULL,NULL)!=1||
        decode_frame(p+42,&eph,NULL,NULL,NULL,NULL)!=2||
        decode_frame(p+72,&eph,NULL,NULL,NULL,NULL)!=3) {
        return 0;
    }
    if (!strstr(raw->opt,"-EPHALL")) {
        if (eph.iodc==raw->nav.eph[sat-1].iodc&&
            eph.iode==raw->nav.eph[sat-1].iode) return 0; /* unchanged */
    }
    eph.sat=sat;
    raw->nav.eph[sat-1]=eph;
    raw->ephsat=sat;
    trace(4,"decode_qzssrawsubframeb: sat=%2d\n",sat);
    return 2;
}
/* decode qzssionutcb --------------------------------------------------------*/
static int decode_qzssionutcb(raw_t *raw)
{
    unsigned char *p=raw->buff+CNAVHLEN;
    int i;
    
    trace(3,"decode_qzssionutcb: len=%d\n",raw->len);
    
    if (raw->len<CNAVHLEN+108) {
        trace(2,"cnav qzssionutcb length error: len=%d\n",raw->len);
        return -1;
    }
    for (i=0;i<8;i++) raw->nav.ion_qzs[i]=R8(p+i*8);
    raw->nav.utc_qzs[0]=R8(p+72);
    raw->nav.utc_qzs[1]=R8(p+80);
    raw->nav.utc_qzs[2]=U4(p+68);
    raw->nav.utc_qzs[3]=U4(p+64);
    raw->nav.leaps =I4(p+96);
    return 9;
}
/* decode galephemerisb ------------------------------------------------------*/
static int decode_galephemerisb(raw_t *raw)
{
    eph_t eph={0};
    unsigned char *p=raw->buff+CNAVHLEN;
    double tow,sqrtA,af0_fnav,af1_fnav,af2_fnav,af0_inav,af1_inav,af2_inav,tt;
    char *msg;
    int prn,rcv_fnav,rcv_inav,svh_e1b,svh_e5a,svh_e5b,dvs_e1b,dvs_e5a,dvs_e5b;
    int toc_fnav,toc_inav,week,sel_nav=0;
    
    trace(3,"decode_galephemerisb: len=%d\n",raw->len);
    
    if (raw->len<CNAVHLEN+220) {
        trace(2,"cnav galephemrisb length error: len=%d\n",raw->len);
        return -1;
    }
    prn       =U4(p);   p+=4;
    rcv_fnav  =U4(p)&1; p+=4;
    rcv_inav  =U4(p)&1; p+=4;
    svh_e1b   =U1(p)&3; p+=1;
    svh_e5a   =U1(p)&3; p+=1;
    svh_e5b   =U1(p)&3; p+=1;
    dvs_e1b   =U1(p)&1; p+=1;
    dvs_e5a   =U1(p)&1; p+=1;
    dvs_e5b   =U1(p)&1; p+=1;
    eph.sva   =U1(p);   p+=1+1; /* SISA */
    eph.iode  =U4(p);   p+=4;   /* IODNav */
    eph.toes  =U4(p);   p+=4;
    sqrtA     =R8(p);   p+=8;
    eph.deln  =R8(p);   p+=8;
    eph.M0    =R8(p);   p+=8;
    eph.e     =R8(p);   p+=8;
    eph.omg   =R8(p);   p+=8;
    eph.cuc   =R8(p);   p+=8;
    eph.cus   =R8(p);   p+=8;
    eph.crc   =R8(p);   p+=8;
    eph.crs   =R8(p);   p+=8;
    eph.cic   =R8(p);   p+=8;
    eph.cis   =R8(p);   p+=8;
    eph.i0    =R8(p);   p+=8;
    eph.idot  =R8(p);   p+=8;
    eph.OMG0  =R8(p);   p+=8;
    eph.OMGd  =R8(p);   p+=8;
    toc_fnav  =U4(p);   p+=4;
    af0_fnav  =R8(p);   p+=8;
    af1_fnav  =R8(p);   p+=8;
    af2_fnav  =R8(p);   p+=8;
    toc_inav  =U4(p);   p+=4;
    af0_inav  =R8(p);   p+=8;
    af1_inav  =R8(p);   p+=8;
    af2_inav  =R8(p);   p+=8;
    eph.tgd[0]=R8(p);   p+=8; /* BGD: E5A-E1 (s) */
    eph.tgd[1]=R8(p);         /* BGD: E5B-E1 (s) */
    eph.iodc  =eph.iode;
    eph.svh   =(svh_e5b<<7)|(dvs_e5b<<6)|(svh_e5a<<4)|(dvs_e5a<<3)|
               (svh_e1b<<1)|dvs_e1b;
    
    /* ephemeris selection (0:INAV,1:FNAV) */
    if      (strstr(raw->opt,"-GALINAV")) sel_nav=0;
    else if (strstr(raw->opt,"-GALFNAV")) sel_nav=1;
    else if (!rcv_inav&&rcv_fnav) sel_nav=1;
    
    eph.A     =sqrtA*sqrtA;
    eph.f0    =sel_nav?af0_fnav:af0_inav;
    eph.f1    =sel_nav?af1_fnav:af1_inav;
    eph.f2    =sel_nav?af2_fnav:af2_inav;
    eph.code  =sel_nav?2:1; /* data source 1:I/NAV E1B,2:F/NAV E5a-I */
    
    if (raw->outtype) {
        msg=raw->msgtype+strlen(raw->msgtype);
        sprintf(msg," prn=%3d iod=%3d toes=%6.0f",prn,eph.iode,eph.toes);
    }
    if (!(eph.sat=satno(SYS_GAL,prn))) {
        trace(2,"oemv galephemeris satellite error: prn=%d\n",prn);
        return -1;
    }
    tow=time2gpst(raw->time,&week);
    eph.week=week; /* gps-week = gal-week */
    eph.toe=gpst2time(eph.week,eph.toes);
    
    /* for week-handover problem */
    tt=timediff(eph.toe,raw->time);
    if      (tt<-302400.0) eph.week++;
    else if (tt> 302400.0) eph.week--;
    eph.toe=gpst2time(eph.week,eph.toes);
    eph.toc=adjweek(eph.toe,sel_nav?toc_fnav:toc_inav);
    eph.ttr=adjweek(eph.toe,tow);
    
    if (!strstr(raw->opt,"-EPHALL")) {
        if (raw->nav.eph[eph.sat-1].iode==eph.iode&&
            raw->nav.eph[eph.sat-1].code==eph.code) return 0; /* unchanged */
    }
    raw->nav.eph[eph.sat-1]=eph;
    raw->ephsat=eph.sat;
    return 2;
}
/* decode galalmanacb --------------------------------------------------------*/
static int decode_galalmanacb(raw_t *raw)
{
    alm_t alm={0};
    unsigned char *p=raw->buff+CNAVHLEN;
    double dsqrtA,sqrtA=sqrt(29601297.0);
    int prn,rcv_fnav,rcv_inav,svh_e1b,svh_e5a,svh_e5b,ioda;
    
    trace(3,"decode_galalmanacb: len=%d\n",raw->len);
    
    if (raw->len<CNAVHLEN+100) {
        trace(2,"cnav galephemrisb length error: len=%d\n",raw->len);
        return -1;
    }
    prn     =U4(p);   p+=4;
    rcv_fnav=U4(p)&1; p+=4;
    rcv_inav=U4(p)&1; p+=4;
    svh_e1b =U1(p)&3; p+=1;
    svh_e5a =U1(p)&3; p+=1;
    svh_e5b =U1(p)&3; p+=1+1;
    ioda    =U4(p);   p+=4;
    alm.week=U4(p);   p+=4; /* gst week */
    alm.toas=U4(p);   p+=4;
    alm.e   =R8(p);   p+=8;
    alm.OMGd=R8(p);   p+=8;
    alm.OMG0=R8(p);   p+=8;
    alm.omg =R8(p);   p+=8;
    alm.M0  =R8(p);   p+=8;
    alm.f0  =R8(p);   p+=8;
    alm.f1  =R8(p);   p+=8;
    dsqrtA  =R8(p);   p+=8;
    alm.i0  =(R8(p)+56.0)*D2R;
    alm.svh =(svh_e5b<<7)|(svh_e5a<<4)|(svh_e1b<<1);
    alm.A   =(sqrtA+dsqrtA)*(sqrtA+dsqrtA);
    
    if (!(alm.sat=satno(SYS_GAL,prn))) {
        trace(2,"oemv galalmanac satellite error: prn=%d\n",prn);
        return -1;
    }
    alm.toa=gst2time(alm.week,alm.toas);
    raw->nav.alm[alm.sat-1]=alm;
    return 0;
}
/* decode galclockb ----------------------------------------------------------*/
static int decode_galclockb(raw_t *raw)
{
    unsigned char *p=raw->buff+CNAVHLEN;
    double a0,a1,a0g,a1g;
    int leaps,tot,wnt,wnlsf,dn,dtlsf,t0g,wn0g;
    
    trace(3,"decode_galclockb: len=%d\n",raw->len);
    
    if (raw->len<CNAVHLEN+64) {
        trace(2,"cnav galclockb length error: len=%d\n",raw->len);
        return -1;
    }
    a0   =R8(p); p+=8;
    a1   =R8(p); p+=8;
    leaps=I4(p); p+=4;
    tot  =U4(p); p+=4;
    wnt  =U4(p); p+=4;
    wnlsf=U4(p); p+=4;
    dn   =U4(p); p+=4;
    dtlsf=U4(p); p+=4;
    a0g  =R8(p); p+=8;
    a1g  =R8(p); p+=8;
    t0g  =U4(p); p+=4;
    wn0g =U4(p);
    
    raw->nav.utc_gal[0]=a0;
    raw->nav.utc_gal[1]=a1;
    raw->nav.utc_gal[2]=tot; /* utc reference tow (s) */
    raw->nav.utc_gal[3]=wnt; /* utc reference week */
    return 9;
}
/* decode galionob -----------------------------------------------------------*/
static int decode_galionob(raw_t *raw)
{
    unsigned char *p=raw->buff+CNAVHLEN;
    double ai[3];
    int i,sf[5];
    
    trace(3,"decode_galionob: len=%d\n",raw->len);
    
    if (raw->len<CNAVHLEN+29) {
        trace(2,"cnav galionob length error: len=%d\n",raw->len);
        return -1;
    }
    ai[0]=R8(p); p+=8;
    ai[1]=R8(p); p+=8;
    ai[2]=R8(p); p+=8;
    sf[0]=U1(p); p+=1;
    sf[1]=U1(p); p+=1;
    sf[2]=U1(p); p+=1;
    sf[3]=U1(p); p+=1;
    sf[4]=U1(p);
    
    for (i=0;i<3;i++) raw->nav.ion_gal[i]=ai[i];
    return 9;
}
/* decode galfnavrawpageb ----------------------------------------------------*/
static int decode_galfnavrawpageb(raw_t *raw)
{
    unsigned char *p=raw->buff+CNAVHLEN;
    unsigned char buff[27];
    int i,sigch,satid,page;
    
    trace(3,"decode_galfnavrawpageb: len=%d\n",raw->len);
    
    if (raw->len<CNAVHLEN+35) {
        trace(2,"cnav galfnavrawpageb length error: len=%d\n",raw->len);
        return -1;
    }
    sigch=U4(p); p+=4;
    satid=U4(p); p+=4;
    for (i=0;i<27;i++) {
        buff[i]=U1(p); p+=1;
    }
    page=getbitu(buff,0,6);

    char tstr[40];
    trace(3,"%s E%2d FNAV     (%2d) ",time2str(raw->time,tstr,0),satid,page);
    traceb(3,buff,27);
    
    return 0;
}
/* decode galinavrawwordb ----------------------------------------------------*/
static int decode_galinavrawwordb(raw_t *raw)
{
    unsigned char *p=raw->buff+CNAVHLEN;
    unsigned char buff[16];
    gtime_t time=raw->time;
    char *sig;
    int i,sigch,satid,sigtype,type,week=0,tow=0;
    
    trace(3,"decode_galinavrawwordb: len=%d\n",raw->len);
    
    if (raw->len<CNAVHLEN+28) {
        trace(2,"cnav galinavrawwordb length error: len=%d\n",raw->len);
        return -1;
    }
    sigch  =U4(p); p+=4;
    satid  =U4(p); p+=4;
    sigtype=U4(p); p+=4;
    
    switch (sigtype) {
        case 10433: sig="E1 "; break;
        case 10466: sig="E5A"; break;
        case 10499: sig="E5B"; break;
        default: sig="???"   ; break;
    }
    for (i=0;i<16;i++) {
        buff[i]=U1(p); p+=1;
    }
    type=getbitu(buff,0,6);
    if (type==0&&getbitu(buff,6,2)==2) {
        week=getbitu(buff, 96,12); /* gst week */
        tow =getbitu(buff,108,20);
        time=gst2time(week,tow);
    }
    char tstr[40];
    trace(3,"%s E%2d INAV-%s (%2d) ",time2str(time,tstr,0),satid,sig,type);
    traceb(3,buff,16);
    
    return 0;
}
/* decode rawcnavframeb ------------------------------------------------------*/
static int decode_rawcnavframeb(raw_t *raw)
{
    unsigned char *p=raw->buff+CNAVHLEN;
    unsigned char buff[38];
    int i,sigch,prn,frmid;
    
    trace(3,"decode_rawcnavframeb: len=%d\n",raw->len);
    
    if (raw->len<CNAVHLEN+50) {
        trace(2,"cnav rawcnavframeb length error: len=%d\n",raw->len);
        return -1;
    }
    sigch=U4(p); p+=4;
    prn  =U4(p); p+=4;
    frmid=U4(p); p+=4;
    
    for (i=0;i<38;i++) {
        buff[i]=U1(p); p+=1;
    }
    char tstr[40];
    trace(3,"%s PRN=%3d FRMID=%2d ",time2str(raw->time,tstr,0),prn,frmid);
    traceb(3,buff,38);
    
    return 0;
}
/* decode bdsephemerisb ------------------------------------------------------*/
static int decode_bdsephemerisb(raw_t *raw)
{
    eph_t eph={0};
    unsigned char *p=raw->buff+CNAVHLEN;
    double ura,sqrtA;
    char *msg;
    int prn,toc;
    
    trace(3,"decode_bdsephemerisb: len=%d\n",raw->len);
    
    if (raw->len<CNAVHLEN+196) {
        trace(2,"cnav bdsephemrisb length error: len=%d\n",raw->len);
        return -1;
    }
    prn       =U4(p);   p+=4;
    eph.week  =U4(p);   p+=4;
    ura       =R8(p);   p+=8;
    eph.svh   =U4(p)&1; p+=4;
    eph.tgd[0]=R8(p);   p+=8; /* TGD1 for B1 (s) */
    eph.tgd[1]=R8(p);   p+=8; /* TGD2 for B2 (s) */
    eph.iodc  =U4(p);   p+=4; /* AODC */
    toc       =U4(p);   p+=4;
    eph.f0    =R8(p);   p+=8;
    eph.f1    =R8(p);   p+=8;
    eph.f2    =R8(p);   p+=8;
    eph.iode  =U4(p);   p+=4; /* AODE */
    eph.toes  =U4(p);   p+=4;
    sqrtA     =R8(p);   p+=8;
    eph.e     =R8(p);   p+=8;
    eph.omg   =R8(p);   p+=8;
    eph.deln  =R8(p);   p+=8;
    eph.M0    =R8(p);   p+=8;
    eph.OMG0  =R8(p);   p+=8;
    eph.OMGd  =R8(p);   p+=8;
    eph.i0    =R8(p);   p+=8;
    eph.idot  =R8(p);   p+=8;
    eph.cuc   =R8(p);   p+=8;
    eph.cus   =R8(p);   p+=8;
    eph.crc   =R8(p);   p+=8;
    eph.crs   =R8(p);   p+=8;
    eph.cic   =R8(p);   p+=8;
    eph.cis   =R8(p);
    eph.A     =sqrtA*sqrtA;
    eph.sva   =uraindex(ura);
    
    if (raw->outtype) {
        msg=raw->msgtype+strlen(raw->msgtype);
        sprintf(msg," prn=%3d iod=%3d toes=%6.0f",prn,eph.iode,eph.toes);
    }
    if (!(eph.sat=satno(SYS_CMP,prn))) {
        trace(2,"oemv bdsephemeris satellite error: prn=%d\n",prn);
        return -1;
    }
    eph.toe=bdt2gpst(bdt2time(eph.week,eph.toes)); /* bdt -> gpst */
    eph.toc=bdt2gpst(bdt2time(eph.week,toc));      /* bdt -> gpst */
    eph.ttr=raw->time;
    
    if (!strstr(raw->opt,"-EPHALL")) {
        if (timediff(raw->nav.eph[eph.sat-1].toe,eph.toe)==0.0&&
            raw->nav.eph[eph.sat-1].iode==eph.iode&&
            raw->nav.eph[eph.sat-1].iodc==eph.iodc) return 0; /* unchanged */
    }
    raw->nav.eph[eph.sat-1]=eph;
    raw->ephsat=eph.sat;
    return 2;
}
        
/* decode cnav message -------------------------------------------------------*/
static int decode_cnav(raw_t *raw)
{
    double tow;
    int msg,week,type=U2(raw->buff+4);
    
    trace(3,"decode_cnav: type=%3d len=%d\n",type,raw->len);
    
    /* check crc32 */
    if (rtk_crc32(raw->buff,raw->len)!=U4(raw->buff+raw->len)) {
        trace(2,"cnav crc error: type=%3d len=%d\n",type,raw->len);
        return -1;
    }
    msg =(U1(raw->buff+6)>>4)&0x3;
    if (!(week=U2(raw->buff+14))) {
        return -1;
    }
    week=adjgpsweek(week);
    tow =U4(raw->buff+16)*0.001;
    raw->time=gpst2time(week,tow);
    
    if (raw->outtype) {
        char tstr[40];
        sprintf(raw->msgtype,"CNAV %4d (%4d): msg=%d %s",type,raw->len,msg,
                time2str(gpst2time(week,tow),tstr,2));
    }
    if (msg!=0) return 0; /* message type: 0=binary,1=ascii */
    
    switch (type) {
        case ID_RANGECMP      : return decode_rangecmpb      (raw);
        case ID_RANGE         : return decode_rangeb         (raw);
        case ID_RAWEPHEM      : return decode_rawephemb      (raw);
        case ID_RAWWAASFRAME  : return decode_rawwaasframeb  (raw);
        case ID_RAWSBASFRAME  : return decode_rawsbasframeb  (raw);
        case ID_IONUTC        : return decode_ionutcb        (raw);
        case ID_GLOEPHEMERIS  : return decode_gloephemerisb  (raw);
        case ID_QZSSRAWEPHEM  : return decode_qzssrawephemb  (raw);
        case ID_QZSSRAWSUBFRAME: return decode_qzssrawsubframeb(raw);
        case ID_QZSSIONUTC    : return decode_qzssionutcb    (raw);
        case ID_GALEPHEMERIS  : return decode_galephemerisb  (raw);
        case ID_GALALMANAC    : return decode_galalmanacb    (raw);
        case ID_GALCLOCK      : return decode_galclockb      (raw);
        case ID_GALIONO       : return decode_galionob       (raw);
        case ID_GALFNAVRAWPAGE: return decode_galfnavrawpageb(raw);
        case ID_GALINAVRAWWORD: return decode_galinavrawwordb(raw);
        case ID_RAWCNAVFRAME  : return decode_rawcnavframeb  (raw);
        case ID_BDSEPHEMERIS  : return decode_bdsephemerisb  (raw);
    }
    return 0;
}
/* sync header ---------------------------------------------------------------*/
static int sync_cnav(unsigned char *buff, unsigned char data)
{
    buff[0]=buff[1]; buff[1]=buff[2]; buff[2]=data;
    return buff[0]==CNAVSYNC1&&buff[1]==CNAVSYNC2&&buff[2]==CNAVSYNC3;
}
/* input comnav raw data from stream ----------------------------------------
* fetch next comnav raw data and input a mesasge from stream
* args   : raw_t *raw   IO     receiver raw data control struct
*          unsigned char data I stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 3: input sbas message,
*                  9: input ion/utc parameter)
*
* notes  : to specify input options for cnav, set raw->opt to the following
*          option strings separated by spaces.
*
*          -EPHALL : input all ephemerides
*          -GL1P   : select 1P for GPS L1 (default 1C)
*          -GL2X   : select 2X for GPS L2 (default 2W)
*          -RL2C   : select 2C for GLO L2 (default 2P)
*          -EL2C   : select 2C for GAL L2 (default 2C)
*          -GALINAV: use I/NAV for GAL ephemeris
*          -GALFNAV: use F/NAV for GAL ephemeris
*
*-----------------------------------------------------------------------------*/
extern int input_cnav(raw_t *raw, unsigned char data)
{
    trace(5,"input_cnav: data=%02x\n",data);
    
    /* synchronize frame */
    if (raw->nbyte==0) {
        if (sync_cnav(raw->buff,data)) raw->nbyte=3;
        return 0;
    }
    raw->buff[raw->nbyte++]=data;
    
    if (raw->nbyte==10&&(raw->len=U2(raw->buff+8)+CNAVHLEN)>MAXRAWLEN-4) {
        trace(2,"cnav length error: len=%d\n",raw->len);
        raw->nbyte=0;
        return -1;
    }
    if (raw->nbyte<10||raw->nbyte<raw->len+4) return 0;
    raw->nbyte=0;
    
    /* decode cnav message */
    return decode_cnav(raw);
}
/* input comnav raw data from file ------------------------------------------
* fetch next comnav raw data and input a message from file
* args   : raw_t  *raw   IO     receiver raw data control struct
*          int    format I      receiver raw data format (STRFMT_???)
*          FILE   *fp    I      file pointer
* return : status(-2: end of file, -1...9: same as above)
*-----------------------------------------------------------------------------*/
extern int input_cnavf(raw_t *raw, FILE *fp)
{
    int i,data;
    
    trace(4,"input_cnavf:\n");
    
    /* synchronize frame */
    if (raw->nbyte==0) {
        for (i=0;;i++) {
            if ((data=fgetc(fp))==EOF) return -2;
            if (sync_cnav(raw->buff,(unsigned char)data)) break;
            if (i>=4096) return 0;
        }
    }
    if (fread(raw->buff+3,7,1,fp)<1) return -2;
    raw->nbyte=10;
    
    if ((raw->len=U2(raw->buff+8)+CNAVHLEN)>MAXRAWLEN-4) {
        trace(2,"cnav length error: len=%d\n",raw->len);
        raw->nbyte=0;
        return -1;
    }
    if (fread(raw->buff+10,raw->len-6,1,fp)<1) return -2;
    raw->nbyte=0;
    
    /* decode cnav message */
    return decode_cnav(raw);
}
    
