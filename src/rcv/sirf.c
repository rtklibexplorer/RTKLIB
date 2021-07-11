/*------------------------------------------------------------------------------
* sirf.c : SiRF III/IV receiver dependent functions
*
*    Copyright (C) 2013 A.Illarionov, All rights reserved.
*
*    [1] SiRFstarIV One Socket Protocol Interface Control Document 
*
* version : $Revision:$ $Date:$
* history : 2013/05/26 1.0  first version
*	    2019/10/28 2.0  S. Juhl improved the six year old first version.
*	    2020/03/22 2.1  S. Juhl Bugfixes, Auto-Learning Q Indicator for Carrier Phase,
                                    Data alignment of subframes (TBD: MID 14)
            2020/04/26 2.5  S. Juhl Fixed Ephemeris Data acquisition
	    2021/06/02 2.6  S. Juhl code fishing from skytraq.c, fixing minor issues
	                            Inside SiRF: https://www.dexsilicium.com/GSC3f-datasheet.pdf
	    2021/06/08 X.X  S. Juhl added circular buffer
*-- Query Firmware Version -----------------------------------------------------*
echo -en "\xA0\xA2\x00\x02\x84\x00\x00\x84\xb0\xb3" > /dev/ttyUSB0
Stock-ROM GSD4e_4.1.2-P1 R+ 11/15/2011 319-Nov 15 2011-23:04:55
Patch-ROM GSD4e_4.1.2-P1_RPATCH.10- 04/25/2019 115

Qualcomm Software Release Notice // Date: April 25, 2019
Patch version: GSD4e_4.1.2-P1_RPATCH_10
GSD4e_4.1.2-P1_RPATCH_10.pd2 (Full version string: GSD4e_4.1.2-P1_RPATCH.10- 04/25/2019 115)
Changes for GSD4e_4.1.2-P1_RPATCH_10: fix almanac reference week for GPS week rollover

* -- MID 19 Navigation Parameters
A0A2|0041|1300000014020000000400000401010000000004006420000000000000000000000000000003E8000003E80000000000000001D4C0000075300A00000001000001|04D7|B0B3
*-------------------------------------------------------------------------------*/
#include "rtklib.h"
#include "stat.h"

#define SIRFSYNC1	0xA0    /* SiRF binary message start sync code 1		*/
#define SIRFSYNC2	0xA2    /* SiRF binary message start sync code 2 		*/
#define SIRFSYNCEND1	0xB0    /* SiRF binary message end sync code 1 			*/
#define SIRFSYNCEND2	0xB3    /* SiRF binary message end sync code 2 			*/

#define MID_SRFMTDO	0x04	/* SiRF Measured Tracker Data Out			*/
#define MID_SRFCLOCK	0x07    /* SiRF binary clock status 				*/
#define MID_SRFGEOCLK   0x29    /* SiRF Geodetic Navigation Data			*/
#define MID_SRF50BPS	0x08    /* SiRF binary 50BPS data 				*/ 
#define MID_SRF50BPSHQ	0x38    /* SiRF binary 50BPS data 				*/ 
#define MID_SRFNLMEAS	0x1c    /* SiRF binary navlib measurement data 			*/
#define MID_SRFALM	0x0e	/* SiRF Almanac Data (Response to Poll) 		*/
#define MID_SRFEPH	0x0f	/* SiRF Ephemeris Data (Response to Poll) 		*/
#define MID_SRFSVSTATE  0x1e	/* SiRF Navigation Library SV State Data		*/
#define MID_SRFAUXDATA	0x40	/* SiRF Navigation Library Auxiliary Measurement Data	*/
#define MID_SRFTCXO	0x5d	/* SiRF Temperature Value Output			*/
#define SIRFMAXRAWLEN	2047    /* max length of SiRF binary raw message 		*/

#define MAXSTR      5                  /* max number of streams */

static int decode_sirfgen(raw_t *);

/* double L[NFREQ+NEXOBS]; // observation data carrier-phase (cycle) */
/* double P[NFREQ+NEXOBS]; // observation data pseudorange (m) */
/* float  D[NFREQ+NEXOBS]; // observation data doppler frequency (Hz) */

/* NOTE Empirische Message Sequence 
Sequence 93 7 2 4 41 30 64 28 65 
 7 MID_SRFCLOCK		Clock Status Data		  
 2 --- INFO ---		Measure Navigation Data Out
 4 MID_SRFMTDO		Measured Tracker Data Out		(AuxData TrackStat + Elev)
41 MID_SRFGEOCLK	Geodetic Navigaton Data 		-> TOW, Week, Clock Bias and drift
30 MID_SRFSVSTATE	Navigation Library SV State Data	-> Update Pseudo Range with ionospheric delay 
64 Navigation Library Messages (Auxiliary Measurement Data)	-> Pseudo Range & Co
28 MID_SRFNLMEAS	Navigation Library Measurement Data
65 --- INFO ---		GPIO State Output
93 MID_SRFTCXO		SiRF Temperature Value Output		-> precise clock bias and drift
*/

typedef struct {        /* MID 4 + MID28 + MID30 + MID64 data 				*/
    int sat;            /* satellite number 						*/
    int trackStat;	/* Measured Tracker Data Out - Status				*/
    int TimeTag28;	/* TimeTag28 							*/
    gtime_t time;	/* GPS Software Time % clock bias = GPS time of measurement 	*/
    double dtime;	/* GPS Software Time as Double Type				*/
    double PseudoR;	/* Pseudorange [m] 						*/
    double CarrFreq28;	/* Carrier Frequency [m/s] 					*/
    double CarrPhase28; /* Carrier Phase [m] 						*/
    int TimeTrack;	/* Time in Track [m/s] 						*/
    int SyncFlag;	/* Sync Flaggs							*/
    int cno;		/* reserved for Std dev.					*/
    int cn[130];
    int DeltaRange;	/* Delta Range Interval						*/
    int ExtraTime;	/* Extrapolation Time						*/
    int PhErrCnt;	/* Phase Error Count						*/
    double px,py,pz;	/* position X,Y,Z 						*/
    double vx,vy,vz;	/* velocity X,Y,Z 						*/
    double clkbias;	/* Clock Bias 							*/
    int clkdrift;	/* Clock drift							*/
    int iono;		/* float Ionospheric Delay 						*/
    int extStat;	/* Extended Status: SF Sync verified, Possible Cycle Slip, SF Sync lost, Multipath, Weak Frame */
    int RecovStat;	/* Weak Bit Sync, False Lock, wrong BitSync, wrong FrameSync, Bad PrePos  */
    int TimeTag64;	/* Measurement Time Tag */
    double CarrPhase64;	/* Carrier Phase double */
    double CarrFreq64;	/* Carrier Freq */
    signed short CarrAcc;	/* Carrier Acceleration (Doppler Rate) */
    signed short PRNoise;	/* Pseudorange Noise (one sigma, normalized and left-shifted 16 bits */
    signed short dRngQ;		/* Delta Range Quality						*/
    signed short PhLockQ;	/* Phase Lock Quality, normalized and left-shifted 8 bits */
    float QIRatio;
    float elev;		/* elevation angle from MID 4	*/
    double tgd,f0,f1,f2;	/* Subframe clock data */
    double SmoothCode;
    double dtt;		/* delta Time Tag */
    int amdstat;	/* Auxiliary Measurement Data Status Bit Fields 			*/
    int amdestat;	/* Auxiliary Measurement Data Extended Status Bit Field definitions 	*/
    int amdrstat;	/* Auxiliary Measurement Data Recovery Status Bit Fields 		*/
    float MeanDR;	/* Mean Delta Range Time					*/
    double prs;		/* smoothed pseudorange */
} auxData_t;

/* NOTE Doppler Ref.: https://www.u-blox.com/sites/default/files/products/documents/GPS-Compendium_Book_%28GPS-X-02007%29.pdf
        As a result of the Doppler Effect (satellites and receivers are in relative motion to one another) the transmitted
        signals can be shifted by up to +/-5000 Hz at the point of reception.
        Furthermore, the local reference frequency may have also an offset which adds to the frequency span that needs to be searched. 
        1 ppm of frequency error in the local oscillator corresponds to 1.575 kHz Doppler shift

        To convert Drift m/s to Hz: Drift (m/s) * L1(Hz) / c = Drift (Hz).

ROM Patching Application note
https://y1cj3stn5fbwhv73k0ipk1eg-wpengine.netdna-ssl.com/wp-content/uploads/2017/09/Telit_SiRF_StarIV_ROM_Patching_Application_Note_r1.pdf

// Global Variabels ------------------------------------------------------------*/
int VerData = 0;
int AuxMeasData = 0;
int InfoCnt0=0;
int InfoCnt1=0;
int TimeDebug=0;			/* Debug GPS TOW in various MIDs */
unsigned int MBuf[100];
unsigned int MC=0;
float QImin=100; 
float QImax=0;
double ClkBiasTCXO = 96250/10e+9;	/* MID 93.18 calculated clock bias of TCXO 	*/
double rcvbias;
double rcvdrift;
int lweek;
double GeoTOW=0;
float altMSL=0.0;			/* MID 41 for tropo correction			*/
unsigned int M28acq=0;			/* MID 28 ACQ Clock */
double M28TOW=0;
double M28fclk=0;
auxData_t auxData[MAXSAT];
static const gtime_t time0 = {0};

/* Processing Options => alles auf 0 Position ok?*/
int TropoDelay=0;	/* Tropospheric compensation 				*/
int IonoDelay=0;	/* Ionospheric compensation 				*/
int CarFrqBC=0;		/* Carrier frequency clock bias correction 		*/
int RelCorr=0;		/* Relativistic correction				*/
int TTAdj=0;		/* Adjust TOW based on internal Time Tag		*/
int PEthres=3;		/* Number of phase errors considered as cycle slip	*/
int CSmth=0;		/* Code smoothing					*/
int PSmth=0;		/* Pseudorange smoothing				*/
float RTSres=0.001;	/* RTCM Time Stamp Resolution				*/

/* Import global stream variable */
extern strsvr_t strsvr;                /* stream server */

/* extract fields (big-endian) -----------------------------------------------------*/
#define U1(p) (*((unsigned char *)(p)))
#define I1(p) (*((signed char *)(p)))	/* char */

static unsigned short U2(unsigned char *p)
{
    unsigned short value;
    unsigned char *q=(unsigned char *)&value+1;
    int i;
    for (i=0;i<2;i++) *q--=*p++;
    return value;
}
static unsigned int U4(unsigned char *p)
{
    unsigned int value;
    unsigned char *q=(unsigned char *)&value+3;
    int i;
    for (i=0;i<4;i++) *q--=*p++;
    return value;
}
static float R4(unsigned char *p)
{
    float value;
    unsigned char *q=(unsigned char *)&value+3;
    int i;
    for (i=0;i<4;i++) *q--=*p++; 
    return value;
}
static double R8(unsigned char *p, unsigned gsw230)
{
    double value;
    unsigned char *q;
    int i;
    if (gsw230) {
    	double value;
    	unsigned char *q=(unsigned char *)&value+7;
    	int i;
    	for (i=0;i<8;i++) *q--=*p++;
    	return value;
    } else {
    	q = (unsigned char *)&value+3;
    	for (i=0;i<4;i++) *q--=*p++;
    	q = (unsigned char *)&value+7;
    	for (i=0;i<4;i++) *q--=*p++;
    	return value;
    }
}
static double R8ST(unsigned char *p)
{
    double value;
    unsigned char *q=(unsigned char *)&value+7;
    int i;
    for (i=0;i<8;i++) *q--=*p++;
    return value;
}
static signed short S2(unsigned char *p) { 
    signed short value;
    unsigned char *q=(unsigned char *)&value+1;
    int i; for (i=0;i<2;i++) *q--=*p++;
    return value;
}
static signed int  S4(unsigned char *p) {
    signed int value;
    unsigned char *q=(unsigned char *)&value+3;
    int i;
    for (i=0;i<4;i++) *q--=*p++;
    return value;
}

/* Find maximum between two numbers. */
int max(int num1, int num2) { return (num1 > num2 ) ? num1 : num2; }

/* checksum ------------------------------------------------------------------*/
static int chksum(const unsigned char *buff, int len)
{
    int i;
    unsigned short sum=0;

    if (len<8) return 0;
    for (i=4;i<len-4;i++) sum=0x7fff&(sum+buff[i]);
    return (sum>>8)==buff[len-4]&&(sum&0xFF)==buff[len-3];
}
/* 8-bit week -> full week ---------------------------------------------------*/
static void adj_utcweek(gtime_t time, double *utc)
{
    int week;

    if (utc[3]>=256.0) return;
    time2gpst(time,&week);
    utc[3]+=week/256*256;
    if      (utc[3]<week-128) utc[3]+=256.0;
    else if (utc[3]>week+128) utc[3]-=256.0;
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

static void printBits(unsigned char num)
{
   int bit;
   for(bit=8*sizeof(num)-1; bit>=0; bit--) {
        (num & (1 << bit)) ? putchar('1') : putchar('0');
   }
}

char *Bits(unsigned char num) {     
    char ausgabe[19];    
    char *zeiger;        
    int bit;
    strcpy(ausgabe,"");
    for(bit=8*sizeof(num)-1; bit>=0; bit--) {
        strcat(ausgabe,((num & (1 << bit)) ? "1": "0"));
    }
    /* sprintf(ausgabe, "# %d #", num);*/
    zeiger=ausgabe;        
    return(zeiger);   
}  

/* NOTE Print Message sequence */
static void printMBuf(signed int status) {
    unsigned int i,debug=0;
    char IType[13];
    if (MC>0) {
        if (debug) {
            switch (status) {
                case 1: strcpy(IType,"I=OBS");  break;
                case 2: strcpy(IType,"I=EPH");  break;
                case 3: strcpy(IType,"I=SBAS"); break;
                case 9: strcpy(IType,"I=ION");  break;
                default: sprintf(IType, "I=%d", status);
            }
            fprintf(stderr,"%s [INFO ] %s MID Sequence[%2d]=",time_str(utc2gpst(timeget()),0),IType,MC); /* fprintf(stderr,"|MBuf[%2d]=",MC); */
            for (i=0;i<MC;i++) { fprintf(stderr," %d",MBuf[i]); } 
            fprintf(stderr,"\n");
        }
        MC=0;
    }
}

/* flush observation data buffer ---------------------------------------------*/
static int flushobuf(raw_t *raw) {
  int i, j, n=0, dPL=0, dPLi=0, debug=0;

  trace(3, "flushobuf: n=%d\n", raw->obuf.n);

  /* copy observation data buffer */
  for (i = 0; i < raw->obuf.n && i < MAXOBS; i++) {
    if (!satsys(raw->obuf.data[i].sat, NULL)) continue;
    if (raw->obuf.data[i].time.time == 0) continue;
    if (raw->obuf.data[i].D[0]==0&&raw->obuf.data[i].L[0]==0&&raw->obuf.data[i].P[0]==0) continue;
/*    if (raw->obuf.data[i].P[0]!=0&&raw->obuf.data[i].L[0]!=0) {
         dPL+=abs(((raw->obuf.data[i].P[0]+0*auxData[raw->obuf.data[i].sat].prs)-raw->obuf.data[i].L[0]*CLIGHT/FREQL1)/(CLIGHT/FREQL1));
         dPLi++; } */
    if (debug) { 
        fprintf(stderr,"%s [INFO ] OBUF[%2d]: sat=%2d LLI=%d TAERS=%s|%s|%s D=%7.1f L=%9.0f P=%12.3f P/L'=%+4.1fppm el=%.1f dP=%+7.3f (%3.1fCy) ps=%f\n",
        time_str(utc2gpst(timeget()),0),i,raw->obuf.data[i].sat,
        raw->obuf.data[i].LLI[0],Bits(auxData[raw->obuf.data[i].sat].trackStat),
        Bits(auxData[raw->obuf.data[i].sat].amdstat),Bits(auxData[raw->obuf.data[i].sat].amdestat), /*Bits(auxData[raw->obuf.data[i].sat].amdrstat), */
        raw->obuf.data[i].D[0],raw->obuf.data[i].L[0],raw->obuf.data[i].P[0],
        1000000*(1-raw->obuf.data[i].P[0]/(raw->obuf.data[i].L[0]*CLIGHT/FREQL1)),
        auxData[raw->obuf.data[i].sat].elev,
        (raw->obuf.data[i].P[0]+0*auxData[raw->obuf.data[i].sat].prs)-raw->obuf.data[i].L[0]*CLIGHT/FREQL1,
        ((raw->obuf.data[i].P[0]+0*auxData[raw->obuf.data[i].sat].prs)-raw->obuf.data[i].L[0]*CLIGHT/FREQL1)/(CLIGHT/FREQL1),
        auxData[raw->obuf.data[i].sat].prs
        );
    }
    raw->obs.data[n++] = raw->obuf.data[i];
  } 
/*  if (dPLi>0) {fprintf(stderr,"%s [INFO ] dPL=%.1f=%+5.3fm=%+.9fs bias=%.6fs\n",time_str(utc2gpst(timeget()),0),(float)dPL/dPLi,
  (double)dPL/dPLi*CLIGHT/FREQL1,
  (double)(dPL/dPLi*CLIGHT/FREQL1)/CLIGHT,
  rcvbias
  ); } */
  raw->obs.n = n;

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
  raw->obuf.n=0; for (i = 0; i < MAXSAT; i++) { raw->prCA[i] = raw->dpCA[i] = 0.0; } 
  return n > 0 ? 1 : 0;
}

/* NOTE DECODE MID 4 Measured Tracker Data Out ---------------------------------- 
        + Tropospheric Delay */
static int decode_sirfmtdo(raw_t *raw)
{
    unsigned int i,j,ch,sat,TrackStat,debug=0; 	/* NOTE Set to 1 for debug output	*/
    float tTD,elev;
    unsigned char *p;
    obsd_t *obsd;

    p=raw->buff+4; ch=U1(p+1);
    if (ch>=MAXOBS) {
        trace(2,"SiRF mid#4 wrong channel: ch=%d\n",ch);
        return -1;
    }
    /* NOTE Save Tracking status for further processing */
    for(j=0; j<=11; j++) { 				
        sat=satno(SYS_GPS,U1(p+8+j*15)); 	        
        if (sat) {
            /* Search PRN is observation buffer */
            for (i=0;i<raw->obuf.n&&i<MAXOBS;i++) { if (raw->obuf.data[i].sat==sat) break; }
            TrackStat=U1(p+12+j*15); 
            elev=(float)U1(p+10+j*15)/2;
            if (i<=MAXOBS) { 
                obsd=&raw->obuf.data[i]; 

                /* Approximation of troposheric / atmospheric delay according to Spilker (1994)	*/
                if (TropoDelay&&elev!=0&&altMSL!=0) { 
                    tTD=2.44*1.0121*exp(-0.000133*altMSL)/(0.121+sin(elev*PI/180))/CLIGHT; 
                    if (debug) fprintf(stderr,"SAT %d: Tropo Delay=%.6fs dL=%f dP=%f\n",sat,tTD,tTD*FREQL1,tTD*CLIGHT); 
                    obsd->L[0]-=tTD*FREQL1; obsd->P[0]-=tTD*CLIGHT; 
                } 

                /* Check Tracking Stat Flags */
                if (debug) fprintf(stderr,"%s [INFO ] MID  4: sat=%2d Status=%s Code=%d Costa=%d\n",time_str(utc2gpst(timeget()),0),sat,Bits(TrackStat),(TrackStat&0x20)>>5,(TrackStat&0x08)>>3); 
                obsd->D[0]*=(TrackStat&0x10)>>4;	/* Bit 4: Carrier pullin has been completed (Costas lock)	*/
                obsd->P[0]*=(TrackStat&2)>>1;		/* delta range in MID 28 is also valid				*/
                auxData[sat].MeanDR*=(TrackStat&2)>>1;	/* ... Mean Delta Range either					*/
                obsd->L[0]*=(TrackStat&0x08)>>3;	/* Bit 3: Subframe synchronization has been completed		*/
                obsd->L[0]*=(TrackStat&0x10)>>4;	/* Bit 4: Carrier pullin has been completed (Costas lock)	*/
                if (obsd->LLI[0]&1) obsd->L[0]=obsd->P[0]=0.0;
            }
            auxData[sat].trackStat=TrackStat;
            auxData[sat].elev=elev;			/* Save Elevation for Tropo correction	*/
            }						/* if (auxData[sat].tgd!=0) fprintf(stderr,"\nMID 41 AUXDATA sat=%d tgd=%f elev=%d",sat,auxData[sat].tgd,U1(p+10+j*15)); */
    }
    if (TimeDebug) fprintf(stderr,"%s [INFO ] MID  4: GPS TOW=%.2f\n",time_str(utc2gpst(timeget()),0),(double)U4(p+3)/100); 
    return 0;
}    

/* NOTE DECODE Generic Clock Data (from MID 7, add MID 41, too?)
        - U-Blox RXM-RAWX Carrier phase locktime counter (maximum 64500ms) -> SiRF value limited to 30000ms
        - SV Clock Drift Correction still missing!  

        Time of receipt, time of transmission and resulting pseudo range correction 
        GPS Software Time – Clock Bias = Time of Receipt = GPS Time. 
        GPS Software Time – Pseudorange (sec) = Time of Transmission = GPS Time. 

   NOTE GPS Clock Frequency  MID 93.18 provides Clock Drift Uncertainty 
        fprintf(stderr,"\nGPS Clock:%f kHz, RepBias:%8.3f µs, ",(FREQL1+drift)*16/1540/1e+3,U4(p+12)/1e+3);
        fprintf(stderr,"CalcBias = %8.3f µs, ",1e6*drift/FREQL1);  fprintf(stderr,"DeltaTime = %.3f s, ",tow-U4(p+16)/1000);  

   NOTE The resolution of the time stamps in the RTCM format is 0.001 seconds. */
static double ClkUpdate(raw_t *raw, int week, double tow, double clkbias, double clkdrift) {
    int i;
    double tt,tadj=0.0,toff=0.0,tn;
    gtime_t time;
    obsd_t *obsd;

    time=gpst2time(week,tow-clkbias);    	/* Adjust the GPS Software time by subtracting clock bias 			*/
    tadj=RTSres; if (tadj>0.0) {			/* The resolution of the time stamps in the RTCM format is 0.001 seconds. 	*/
        tn=time2gpst(time,&week)/tadj;
        toff=(tn-floor(tn+0.5))*tadj;
        time=timeadd(time,-toff);
    } 
    tt=timediff(time,raw->time);
    raw->time=time; 

    /* Receiver clock adjustments */
    toff+=clkbias; /*+*/
    for (i=0;i<raw->obuf.n&&i<MAXOBS;i++) {
        obsd=&raw->obuf.data[i];			/* obsd->timevalid=timestat; Only provided by MID 7 */
        obsd->D[0]-=(clkdrift+CarFrqBC*clkbias/FREQL1); /* To adjust the reported carrier frequency do the following: (->D Hz)
                                                           Corrected Carrier Frequency (m/s) = Reported Carrier Frequency (m/s) - Clock Drift (Hz)*C / 1575420000 Hz. */
        obsd->D[0]*=-1.0;                       	/* positive sign for approaching satellites (change sign for compatibility reasons) */
        obsd->L[0]-=(toff+0.0*auxData[obsd->sat].MeanDR)*FREQL1;			/* Adjust carrier phase [m] by subtracting clock bias times speed of light/GPS L1 frequency. 
                                                           Carrier phase has been converted from meter to cycles by *L1/c, so FREQL1 remains    */
        obsd->P[0]-=(toff+0.0*auxData[obsd->sat].MeanDR)*CLIGHT; 			/* Adjust pseudorange by subtracting clock bias times the speed of light 		*/
        /*obsd->L[0]*=0.999997; obsd->P[0]*=0.999997;	NICE TRY */
        /*obsd->L[0]-=fmod(obsd->L[0],CLIGHT/FREQL1)*CLIGHT/FREQL1; 	/* Cut off residuals of cycles -> NO IMPROVEMENT			*/        
        if (TTAdj) { obsd->time=timeadd(time,-auxData[obsd->sat].dtt); }         /* obsd->time=time; */
        else { obsd->time=raw->time; } 

        /* NOTE When carrier phase is locked, the delta-range interval is measured for a period of time before the measurement time. By subtracting the time in this field,
                reported in milliseconds, from the reported measurement time (Time Tag or GPS Software Time) the middle of the measurement interval will be computed. 
           => However, the results get worse, when used :( */
        /*obsd->time=timeadd(time,-0.0*auxData[obsd->sat].MeanDR);*/
    }
    return tt;
}

static double ClkUpd(raw_t *raw, int week, double tow, double clkbias, double clkdrift, int mid) {
    double ClkFreq,tt=0.0,debug=0;
    /* fprintf(stderr,"%s [INFO ] CLK MID %d: week=%d tow=%.3f bias=%6f, drift=%6f\n",time_str(utc2gpst(timeget()),0),mid,week,tow,clkbias,clkdrift); */
    rcvbias=clkbias; rcvdrift=clkdrift; lweek=week;
    if (mid==41) { /* Use Only MID 41 for Clock Corrections?! */
        tt = ClkUpdate(raw, week, tow, clkbias, clkdrift);
        /* NOTE 1) Clock Drift is a direct result of the GPS crystal frequency, so it is reported in Hz. Rate of change of the Clock Bias. */
        /* NOTE 2) Clock Drift in CSR receivers is directly related to the frequency of the GPS clock, derived from the GPS crystal.
                   From the reported frequency, you can compute the GPS clock frequency, and you can predict the next clock bias. 
                   Clock Frequency = (GPS L1 Frequency + Clock Drift) * Crystal Factor / 1540 						
                
           NOTE 3) Clock drift also appears as a Doppler bias in Carrier Frequency reported in Message ID 28.
                   MID 7 contains the clock bias that must be considered:
                   a) Adjust the GPS Software time by subtracting clock bias, -> GPS Software Time – Clock Bias = Time of Receipt = GPS Time.
                      Note: GPS Software Time – Pseudorange (sec) = Time of Transmission = GPS Time. 
                            Adjust SV position in MID 30 by (GPS Time MID 30 – Time of Transmission) * Vsat.                
                   b) adjust pseudorange by subtracting clock bias times the speed of light, and 
                   c) adjust carrier phase by subtracting clock bias times speed of light/GPS L1 frequency. 

                   To adjust the reported carrier frequency do the following:
                   Corrected Carrier Frequency (m/s) = Reported Carrier Frequency (m/s) - Clock Drift (Hz)*C / 1575420000 Hz.

                   For a nominal clock drift value of 96.25 kHz (equal to a GPS Clock frequency of 24.5535 MHz), the correction
                   value is 18315.766 m/s. */
        ClkFreq=(FREQL1+clkdrift)*16/1540;
        /*    if (TimeDebug) { */
        if (debug) fprintf(stderr,"%s [INFO ] MID %2d: drift=%fHz bias=%fs ClkFreq=%fHz CalcBias=%.3fµs\n",time_str(utc2gpst(timeget()),0),mid,clkdrift,clkbias,ClkFreq,clkdrift/FREQL1*1000000); 
    }
    return tt;
}

/* NOTE DECODE MID 7 Clock Status Data		
MID #7 contains the clock bias that must be considered. 
DONE - 1) Adjust the GPS Software time by subtracting clock bias,
DONE - 2) adjust pseudorange by subtracting clock bias times the speed of light, 
DONE - 3) and adjust carrier phase by subtracting clock bias times speed of light/GPS L1 frequency. 

For a nominal clock drift value of 96.25 kHz (equal to a GPS Clock frequency of 24.5535 MHz),
the correction value is 18315.766 m/s.

NOTE Appendix A: Reported clock bias and clock bias computed using the above
     formula will likely agree only to within a few nanoseconds because the
     actual measurement interval may be slightly more or less than an exact
     second, and the clock drift is only reported to a (truncated) 1 Hz 
     resolution. 
NOTE Clock Drift in CSR receivers is directly related to the frequency of the GPS clock, 
     derived from the GPS crystal. Clock drift can be used as a Doppler bias for Carrier
     Frequency calculation, reported in Message ID 28. 
NOTE Clock drift is only reported to a truncated 1 Hz resolution! */
static int decode_sirfclock(raw_t *raw)
{
    unsigned char *p=raw->buff+4,drift,wn;
    double bias,tow,tt,etow; 

    trace(4,"decode_sirfclock: len=%d\n",raw->len);
    
    if (raw->outtype) {
        sprintf(raw->msgtype,"SiRF RAW   (%4d): nsat=%d",raw->len,U1(p+2));
    }
    if (raw->len!=28) {
        trace(2,"SiRF mid#7 length error: len=%d\n",raw->len);
        return -1;
    }
    wn=U2(p+1); tow=(double)U4(p+3)/100;/* Seconds into the current week, accounting for clock bias, when the current measurement was
                                   made. This is the true GPS time of the solution. */

    drift=U4(p+8);		/* Clock Drift in CSR receivers is directly related to the frequency of the GPS clock, 
                                   derived from the GPS crystal. Clock drift can be used as a Doppler bias for Carrier
                                   Frequency calculation, reported in Message ID 28. 
                                   NOTE Clock drift is only reported to a truncated 1 Hz resolution! */
    bias=U4(p+12)/1e+9;		/* TBD: In einem 125er Array ablegen und rsd rechnen? */
    etow=U4(p+16)/1000;
    if (TimeDebug) fprintf(stderr,"%s [INFO ] MID 7: TOW estamination:%.3f\n",time_str(utc2gpst(timeget()),0),tow-etow); 
    
    tt = ClkUpd(raw, wn, tow, bias, drift, 7); 
    return 0;
}

/* NOTE DECODE MID 28 Navigation Library Measurement Data
MID 7 contains the clock bias that must be considered. 
DONE Adjust the GPS Software time by subtracting clock bias,
DONE adjust pseudorange by subtracting clock bias times the speed of light, 
DONE and adjust carrier phase by subtracting clock bias times speed of light/GPS L1 frequency. 
DONE Corrected Carrier Frequency (m/s) = Reported Carrier Frequency (m/s) – Clock Drift (Hz)*C / 1575420000 Hz.

NOTE Carrier frequency may be interpreted as the measured Doppler on the received signal. The value is reported in metres per second but can
     be converted to hertz using the Doppler equation:
     Doppler frequency / Carrier frequency = Velocity / Speed of light, where Doppler frequency is in Hz; Carrier frequency = 1,575,420,000 Hz;
     Velocity is in m/s; Speed of light = 299,792,458 m/s.
     Note that the computed Doppler frequency contains a bias equal to the current clock drift as reported in Message ID 7. This bias, nominally
     96.250 kHz, is equivalent to over 18 km/s.

NOTE The reported Time of Measurement, Pseudorange and Carrier Phase are all uncorrected values.
NOTE GPS Software Time – Clock Bias = Time of Receipt = GPS Time. 
     
     GPS Software Time – Pseudorange (sec) = Time of Transmission = GPS Time. 
     Adjust SV position in MID 30 by (GPS Time MID 30 – Time of Transmission) * Vsat.      		*/
/* NOTE Skytray Equivalent: Skytraw raw measurement (0xDD) */   
static int decode_sirfnlmeas(raw_t *raw)
{
    unsigned char *p=raw->buff+0,ind; /* +4 -> +0 */
    double pr1,cp1,df1,tow,cpstd;  
    int i,prn,sys,sat,lli,lockt,f3,debug=0;
    obsd_t *obsd;
    double dt,cm=0.0;
    unsigned int dc,pherrcnt,ch;
    unsigned gsw230=strstr(raw->opt,"-GSW230")!=NULL;
    float mdr;
    
    trace(4,"decode_sirfnlmeas: len=%d\n",raw->len);
    if (raw->len!=64)	{ trace(2,"SiRF mid#28 length error: len=%d\n",raw->len); return -1; }
    p=raw->buff+4; ch=U1(p+1);
    if (ch>=MAXOBS)	{ trace(2,"SiRF mid#28 wrong channel: ch=%d\n",ch); return -1; }

    /* Fill Observation Data Structure */
    prn=U1(p+6);	/* Satellite ID */

    /* Referencing output buffer */
    if (debug) fprintf(stderr,"%s [INFO ] MID 28: obuf.n=%d ch=%2d prn=%2d",time_str(utc2gpst(timeget()),0),raw->obuf.n,ch,prn); 
    obsd=&raw->obuf.data[ch];
    if (ch>=raw->obuf.n) { raw->obuf.n=ch+1; if (debug) fprintf(stderr," obuf.n=%2d\n",ch+1); }

    /* Satellite System and PRN */
    if      (MINPRNGPS<=prn&&prn<=MAXPRNGPS)         { sys=SYS_GPS; }
    else if (MINPRNGLO<=prn-64&&prn-64<=MAXPRNGLO)   { sys=SYS_GLO; prn-=64; }
    else if (MINPRNQZS<=prn&&prn<=MAXPRNQZS)         { sys=SYS_QZS; }
    else if (MINPRNCMP<=prn-200&&prn-200<=MAXPRNCMP) { sys=SYS_CMP; prn-=200; }
    else {  trace(2,"SiRF raw satellite number error: prn=%d\n",prn); return 0; }
    if (!(sat=satno(sys,prn))) { trace(2,"SiRF raw satellite number error: sys=%d prn=%d\n",sys,prn); return 0; }   
    
    /* Measurement Data Definition */
    ind=U1(p+37);	/* Measurement Indicator Skytraq + 22  */
                        /* GSW2 software this is sync flags -> Table 6.54
                            00 Not aligned
                            01 Consistent code epoch alignment
                            10 Consistent data bit alignment
                            11 No millisecond errors
                           GSW3 code this field is a duplicate of the State field from MID 4 -> Table 6.6: 
                            0  Acquisition/re-acquisition has been completed successfully
                               Bit 0 is controlled by the acquisition hardware. The rest of the bits are controlled 
                               by the tracking hardware except in SiRFstarIII receivers, where bit 2 is also controlled 
                               by the acquisition hardware.
                            1  The integrated carrier phase is valid – delta range in MID 28 is also valid.
                               Bit 1 set means that the phase relationship between the I and Q samples is being tracked.
                               When this bit is cleared, the carrier phase measurements on this channel are invalid.
                            2  Bit synchronization has been completed
                            3  Subframe synchronization has been completed
                            4  Carrier pullin has been completed (Costas lock).
                               Bit 4 set means that the Doppler corrections have been made so that the phase between
                               the I and Q samples is stable.
                            5  Code has been locked
                            6  Multiple uses.
                            7  Ephemeris data is available */
    pherrcnt=U1(p+54);	/* This is the count of the phase errors greater than 60 degrees measured in the preceding
                           second as defined for a particular channel */
    lockt=U2(p+35)/1000; /*fprintf(stderr,"TT=%d %04x %d",auxData[obsd->sat].TimeTrack,U2(p+35),U2(p+35));*/
    tow=R8(p+7,gsw230);	/* GPS Software Time ... will be updated by Sub ClkUpd, subtracting ClkBias provided by MID 41 */

    /* Check Acq clock update ... to be used with TimeTag of MID64 */                                                           
    if ( M28acq != U4(p+2)) { 	
        dt=tow - M28TOW; dc=U4(p+2) - M28acq; 
        M28fclk= (double)dt/dc; M28acq = U4(p+2); M28TOW = tow;
        if (TimeDebug) fprintf(stderr,"%s [INFO ] MID 28: GPS TOW=%.2f TimeTag=%u\n",time_str(utc2gpst(timeget()),0),M28TOW,M28acq); 
    }

    /* Pseudorange, without corrections [m], smoothed by carrier phase!	*/
    pr1=R8(p+15,gsw230);				
    cp1=R8(p+27,gsw230);			  	/* Carrier Phase [m], report as Cycles					
                                                           NOTE Discontinuity in this value generally means a cycle slip and renormalization to pseudorange.*/
    f3=RB_push(raw->cphase[sat][0],cp1*FREQL1/CLIGHT-rcvbias*FREQL1,&cpstd);		/* NOTE Calculate standard deviation of carrier phase for cycle slip detection */

    cp1*=FREQL1/CLIGHT;                  		/* Carrier Phase convert fron m to Cycles */
    /*cp1-=floor((cp1+1E9)/2E9)*2E9; 			// -10^9 < cp1 < 10^9, NOTE clock bias will be substracted in other sub */
    if (strstr(raw->opt,"-INVCP")) { cp1*=-1.0; } 	/* receiver dependent options */ 
    df1=R4(p+23)*FREQL1/CLIGHT; 			/* NOTE Carrier Frequency [m/s] -> Hz; Doppler Calculation in Sub ClkUpd */
    mdr=(float)(U2(p+50)&0xfffc)/1000; 			/* When carrier phase is locked, the delta-range interval is measured for a period of time before
                                                           the measurement time. By subtracting the time in this field, reported in milliseconds, from the
                                                           reported measurement time (Time Tag or GPS Software Time) the middle of the measurement interval will be computed. */
    auxData[obsd->sat].MeanDR=mdr;    			/* fprintf(stderr,"%s [INFO ] MID 28: SAT%2d Mean DRT=%fs\n",time_str(utc2gpst(timeget()),0),sat,mdr); */

    /* Calculate Signal to Noise ratio */
    for (i=120;i>0;i--) auxData[obsd->sat].cn[i+10]=auxData[obsd->sat].cn[i];
    obsd->SNR[0]=U1(p+38); auxData[obsd->sat].cn[1]=U1(p+38);
    for (i=39;i<=47;i++) {				/* Kleinstes Signalrauschverhältnis ermitteln und abspeichern		*/
        auxData[obsd->sat].cn[i-37]=U1(p+i);
        if (U1(p+i)<obsd->SNR[0]) obsd->SNR[0]=U1(p+i);
    }
    for (i=1;i<130;i++) cm+=auxData[obsd->sat].cn[i];
    obsd->SNR[0]=(double)4*cm/130; 
    
    /* NOTE Cycle Slip Detection F1) MID 28 Sync Flag (Bit1+2 -> see Table 6.54 OSP Manual)
                                 F2) MID 28 Phase Error Count (>3 in 1 Sekunde)
                                 F3) MID 28 Carrier Phase Nalimov Outlier Detection
                                 H1) MID 28 StdDev Carrier Phase >5m (>85 for research reasons)
                                 H2) MID 28 Carrier phase locked less than 8s 					
                                 + MID 64.2 Navigation Library (NL) Auxiliary Measurement Data Extended Status Bit (later in process) 
       NOTE LLI Specification according IGS (2013)
             1  Lost lock between previous and current observation: Cycle Slip possible
             2  Half-cycle ambiguity/slip possible
             3  dismiss satellite */
    lli=((ind&6)>>1<2?1:0||pherrcnt>=PEthres?1:0||f3)+2*(cpstd>85?1:0||lockt<10?1:0);
    lockt*=lli&1?0:1;		/* Reset locktime, if cycle slip detected */

    if (debug) fprintf(stderr,"%s [INFO ] MID 28: CycleSlip sat=%2d F=%d|%d|%d LT=%2ds H=%d|%d cno=%.1f cpstd=%.2fm lli=%d\n",time_str(utc2gpst(timeget()),0),sat,
    (ind&6)>>1<2?1:0,	/* F1 SyncStat */
    pherrcnt>3?1:0, 	/* F2 Phase Error Count > 3 auxData[sat].PhErrCnt>1?1:0, */
    f3,			/* F3 Discontinuity in this value generally means a cycle slip and renormalization to pseudorange. */
    lockt,		/* Locktime in s */
    cpstd>5?1:0,	/* Std. Abweichung >5 -> Half Cycle Slip */
    lockt<8?1:0,        /* Locktime <8s -> Half Cycle	*/
    (double)cm/130,cpstd,lli);
    /* cpstd=cpstd<=9?cpstd:9;	/ limit to 9 to fit RINEX format */
    /* cp1=lli&1?0:1;	/ Reset pseudo range also */

    /* Check Acq clock update ... to be used with TimeTag of MID64 */
    if ( M28acq != U4(p+2)) { 
        dt=R8(p+7,gsw230) - M28TOW; dc=U4(p+2) - M28acq; M28fclk=(double)dt/dc;
        M28acq = U4(p+2); M28TOW = R8(p+7,gsw230);
        if (TimeDebug) fprintf(stderr,"%s [INFO ] MID 28: GPS TOW=%.2f TimeTag=%u\n",time_str(utc2gpst(timeget()),0),M28TOW,M28acq); 
    }

    /* Data Mapping */
    obsd->sat=sat; obsd->code[0]=sys==SYS_CMP?CODE_L1I:CODE_L1C; /* obsd->qualL[0]=cpstd; */
    obsd->LLI[0]=lli;
    obsd->P[0]=pr1; /* Pseudorange, without corrections [m], smoothed by carrier phase!	*/
    obsd->D[0]=df1; /* Doppler (Carrier) Frequency */
    obsd->L[0]=cp1; /* Carrier Phase [m], report as Cycles */

    /* NOTE MID 28 does not provide week or clock bias information 
            GPS software time minus clock bias = GPS time of measurement */
    raw->lockt[sat-1][0]=lockt; 			/* Gibt es nicht obsd->lockt[0]=lockt; */
    return 0;
}

/* NOTE DECODE MID 30 Navigation Library SV State Data
   NOTE The data in MID 30 reports the computed satellite position and velocity at the specified GPS time!
   NOTE When using MID 30 SV position, adjust for difference between GPS TIME MID 30 and
        Time of Transmission (see note in MID 28), 
   NOTE Iono delay is not included in pseudorange in MID 28  
   NOTE GPS Software Time – Pseudorange (sec) = Time of Transmission = GPS Time. 
        Adjust SV position in MID 30 by (GPS Time MID 30 – Time of Transmission) * Vsat. 
   NOTE The ionospheric delay is inversely proportional to the square of the signal frequency.
        https://gssc.esa.int/navipedia/index.php/Klobuchar_Ionospheric_Model 
   NOTE Satellite Clock Error and Orbital Solution Error Estimation for Precise Navigation Applications
        https://www.researchgate.net/publication/272877529_Satellite_Clock_Error_and_Orbital_Solution_Error_Estimation_for_Precise_Navigation_Applications */
static int decode_sirfsvstate(raw_t *raw)
{
    int sat,i;
    double vx,vy,vz,v,px,py,pz,r,Dly=0.0;
    unsigned char *p;
    unsigned gsw230=strstr(raw->opt,"-GSW230")!=NULL;
    double IonoDly;
    obsd_t *obsd;

    trace(4,"decode_sirfsvstate: len=%d\n",raw->len); 
    if (raw->len!=91) {
        trace(2,"SiRF mid#30 length error: len=%d\n",raw->len);
        fprintf(stderr,"SiRF mid#30 length error: len=%d\n",raw->len);
        return -1;
    } 
    p=raw->buff+4; sat=satno(SYS_GPS,U1(p+1)); /* fprintf(stderr,"MID 30 sat=%d->obs[%d]?",sat,MAXOBS); */
    
    /* if (TimeDebug) fprintf(stderr,"\nMID30 time=%.3f SAT %2d EFV=%02X Bias=%fs Drift=%ds/s",R8(p+2,gsw230),sat,U1(p+70),R8(p+58,gsw230),U4(p+66)); */
    if (TimeDebug) fprintf(stderr,"%s [INFO ] MID 30: GPS TOW=%.2f\n",time_str(utc2gpst(timeget()),0),R8(p+2,gsw230)); 
    
    /* f=sig_idx(sys,code); sig_freq(sys,f,frqid-7); ... Sekunden*/
    IonoDly=(double)S4(p+79)/FREQL1;
    
    /* Search PRN is observation buffer and update pseudo range accordingly */
    for (i=0;i<raw->obuf.n&&i<MAXOBS;i++) { if (raw->obuf.data[i].sat==sat) break; }
    if (IonoDelay&&i<=MAXOBS&&IonoDly!=0) { Dly=IonoDly/CLIGHT; }

    /* NOTE EXPERIMENTAL: Calculate Doppler Rate from SAT velocity */
    vx=R8(p+34,gsw230); vy=R8(p+42,gsw230); vz=R8(p+50,gsw230); v=sqrt(vx*vx+vy*vy+vz*vz);	/* v = sqrt(pow(auxData[dst->sat].vx,2)+pow(auxData[dst->sat].vy,2)+pow(auxData[dst->sat].vz,2)); */
    px=R8(p+10,gsw230); py=R8(p+18,gsw230); pz=R8(p+26,gsw230); r=sqrt(px*px+py*py+pz*pz);
    /* fprintf(stderr,"sat=%2d v=%6.1fm/s v/c=%f Calculated Doppler=%.1fHz\n ",sat,v,v/CLIGHT,v/CLIGHT*FREQL1); */

    /* Apply relativistic correction v=3,87km/s; h=20200km ;d=12756km; -> d*v/CLIGHT= 32956km²/299792,458km/s ~ 425m */
    if (RelCorr&&i<=MAXOBS) Dly+=2*r*v/pow(CLIGHT,2); 
    /*if (Sagnac&&i<=MAXOBS)  Dly+=v*(dst->P[0]-tTD*CLIGHT)/pow(CLIGHT,2);	/* Sagnac effect correction */
    /* GPS Software Time – Clock Bias = Time of Receipt = GPS Time. 
       GPS Software Time – Pseudorange (sec) = Time of Transmission = GPS Time. 
       Adjust SV position in MID 30 by (GPS Time MID 30 – Time of Transmission) * Vsat. */
    
    obsd=&raw->obuf.data[i];
    obsd->L[0]-=Dly*FREQL1;	
    obsd->P[0]-=Dly*CLIGHT; 
    
    /* dst->P[0]-=fmod(dst->P[0],CLIGHT/FREQL1)*CLIGHT/FREQL1; 	// Cut off residuals of cycles			*/
    
    /* if (i<=MAXOBS) { obsd=&raw->obuf.data[i]; fprintf(stderr,"T%d=%.1f ",sat,obsd->L[0]); } */
    /* NOTE Doppler frequency / Carrier frequency = Velocity / Speed of light, where Doppler frequency is in Hz; Carrier frequency = 1,575,420,000 Hz;
            Velocity is in m/s; Speed of light = 299,792,458 m/s.
            Note that the computed Doppler frequency contains a bias equal to the current clock drift as reported in Message ID 7. This bias, nominally
            96.250 kHz, is equivalent to over 18 km/s. */

    /* NOTE Carrier frequency may be interpreted as the measured Doppler on the received signal. The value is reported in metres per second but can
            be converted to hertz using the Doppler equation:
            Doppler frequency / Carrier frequency = Velocity / Speed of light, 
            where Doppler frequency is in Hz; Carrier frequency = 1,575,420,000 Hz; Velocity is in m/s; Speed of light = 299,792,458 m/s.
            Note that the computed Doppler frequency contains a bias equal to the current clock drift as reported in Message ID 7. This bias, nominally
            96.250 kHz, is equivalent to over 18 km/s. 
     FREQL1/CLIGHT; drift=95639.018010Hz	/ Carrier Frequency [m/s] -> Hz (Doppler Extract erst mit GPS Clock!)	*/

    auxData[sat].time.sec=R8(p+2,gsw230);
    auxData[sat].px=R8(p+10,gsw230);
    auxData[sat].py=R8(p+18,gsw230);
    auxData[sat].pz=R8(p+26,gsw230);
    auxData[sat].vx=R8(p+34,gsw230);
    auxData[sat].vy=R8(p+42,gsw230);
    auxData[sat].vz=R8(p+50,gsw230);
    auxData[sat].clkbias=R8(p+58,gsw230);
    auxData[sat].clkdrift=U4(p+66);	/* R4, zum Beispiel passt aber S4 */
    auxData[sat].iono=U4(p+79);		/* R4, zum Beispiel passt aber S4 U4 da immer positiv*/
    return 0;
    
/* NOTE	When using MID 30 SV position, adjust for difference between GPS Time MID 30 and Time of Transmission
        (see the note in MID 28). Iono delay is not included in pseudorange in MID 28. 				*/
    fprintf(stderr,"\nSVid:%2d;",U1(p+1));
    fprintf(stderr,"GPSt:%1f;",R8(p+2,gsw230));
    fprintf(stderr,"X:%1f;",R8(p+10,gsw230));
    fprintf(stderr,"Y:%1f;",R8(p+18,gsw230));
    fprintf(stderr,"Z:%1f;",R8(p+26,gsw230));
    fprintf(stderr,"vx:%1f;",R8(p+34,gsw230));
    fprintf(stderr,"vy:%1f;",R8(p+42,gsw230));
    fprintf(stderr,"vz:%1f;",R8(p+50,gsw230));
    fprintf(stderr,"ClkBias:%1f;",R8(p+58,gsw230));
    fprintf(stderr,"ClkDrift:%f;",R4(p+66));
    fprintf(stderr,"EFV:%d;",U1(p+70));
    fprintf(stderr,"IonoDly:%f;",R4(p+79));
    fprintf(stderr,"MID 30 AUXDATA tgd=%f",auxData[sat].tgd);
    return 0;	/* NOTE Gibt nichts zurück ... keine Aktion / Verwertung ... */
}

/* NOTE DECODE MID 64 Navigation Library (NL) Auxiliary Measurement Data	
        + Pseudorange Smoothing
        + Pseudorange noise estaminate
        + Phase Lock accuracy estimate
        + Added Self-Learing QI-Range	
        + Code Smoothing 
        + Carrier Frequency backup (if missing in MID 28) */
static int decode_sirfauxdata(raw_t *raw) {
    int sat,sid,i,debug=0;
    unsigned ch;
    unsigned char *p;
    float QI;
    double deltaT,prsmooth,codesmooth,df1;
    unsigned int cpstd, prstd;
    bool f3; double sd=0.0;	/* Cycle slip detection by instability of carrier phase */
    obsd_t *obsd;
    
    p=raw->buff+4; ch=U1(p+2); sid=U1(p+1); if (sid!=2) decode_sirfgen(raw);
    if (raw->len!=75) {
        trace(2,"SiRF mid#64 length error: len=%d\n",raw->len);
        return -1;
    } 
    
    sat=satno(SYS_GPS,U1(p+2));
    if (ch>=MAXOBS) {
        trace(2,"SiRF mid#64 wrong channel: ch=%d\n",ch);
        if (debug) { fprintf(stderr,"SiRF mid#64 wrong channel: sat=%d ch=%d\n",sat,ch);
                     for (i=0;i<raw->len;i++) fprintf(stderr,"%02X ",U1(p+i));
                     fprintf(stderr,"\n"); }
        return -1;
    } 
    if (AuxMeasData == 0) { 
        fprintf(stderr,"%s [INFO ] Navigation Library Auxiliary Measurement Data detected.\n",time_str(utc2gpst(timeget()),0));
        AuxMeasData=1;
    }

    /* Search satellite in output buffer */
    auxData[sat].amdstat=U1(p+3); auxData[sat].amdestat=U1(p+4); auxData[sat].amdrstat=U1(p+62);
    for (i=0;i<raw->obuf.n&&i<MAXOBS;i++) { if (raw->obuf.data[i].sat==sat) break; }
    if (i<=MAXOBS) {
        obsd=&raw->obuf.data[i]; 
        
        /* Cycle Slip Detection */
        if ((U1(p+4)&4)>>2) obsd->LLI[0]|=(1 << 1);

        /* NOTE EXPERIMENTAL: Correction of time based on internal time tag */
        deltaT=(double)(U4(p+6)-M28acq)*M28fclk; deltaT=deltaT>1?0:deltaT;
        if (deltaT>0) { if (TimeDebug) fprintf(stderr,"%s [INFO ] MID 64.2: ut=%.3fs       deltaT=%.9fs sat=%2d\n",time_str(utc2gpst(timeget()),0),(double)U4(p+63)/1000,deltaT,sat); 
                        auxData[sat].dtt=deltaT; } /* timeadd(obsd->time,-deltaT */

        /* Carrier frequency repair (borrow global rcvdrift variable, last updated by MID 7) */
        df1=(double)(S4(p+18)*0.000476); if (!obsd->D[0]) obsd->D[0]=df1;

        /* Code Smoothing */
        codesmooth=(double)S4(p+30); 
        if (CSmth) { obsd->L[0]+=codesmooth; }

        /* Pseudorange smoothing -> no improvement*/    
        prsmooth=(double)S4(p+34)*pow(2,-10)*FREQL1/CLIGHT;  
        auxData[sat].prs=prsmooth;
        if (PSmth) { obsd->P[0]-=prsmooth; }

        /* Pseudorange standard deviation ... to be considered later = ... prstd = (auxData[sat].trackStat&0x20)>>5? */
        prstd=S2(p+42)/pow(2,16)/FREQL1*CLIGHT; 	/* Pseudo Range Noise Estaminate, normalized and left-shifted 16 bits */
        prstd=prstd<=9?prstd:9;  			/* limit to 9 to fit RINEX format  */
        prstd=prstd>1?prstd:1;				/* set to 1 below 0 (default)	   */
        if (prstd>0) obsd->qualP[0]=prstd;

        /* Phase Lock accuracy estimate */
        cpstd=floor((double)S2(p+46)/pow(2,8)*FREQL1/CLIGHT); cpstd=cpstd<=9?cpstd:9;  /* limit to 9 to fit RINEX format */
        if (i<=MAXOBS&&cpstd>0) obsd->qualL[0]=cpstd;
        if (i<=MAXOBS&&cpstd>5) obsd->LLI[0]|=(1 << 1);	/* 1 << ...1=Cycle Slip ...2=Half-Cyle */
        if (raw->lockt[sat-1]>=10) { 
            QI=atan((double)U2(p+52)/U2(p+50));	/* Improvement by Atan-Function? */
            auxData[sat].QIRatio=QI;  
            if (QI < QImin) QImin=QI;
            if (QI > QImax) QImax=QI;
            if (debug) fprintf(stderr,"\nSAT %2d Q=%.3f [%.2f-%.2f] Q:%.1f%%\n",sat,QI,QImin,QImax,(float)100*(auxData[sat].QIRatio-QImin)/(QImax-QImin));
        }
        
        /* DEBUG */
        if (debug) fprintf(stderr,"\nMID  4 CodeLock   =%d PR28=%.3fkm dP=%.3fm dClk=%.3fkm PRQ=%.4fm prstd=%dCy",
            (auxData[sat].trackStat&0x20)>>5, 		/* Aufgabe der MID 4 Routine! 		*/
            (double)obsd->P[0]/1000, 			/* MID 28 Pseudorange in Metern		*/
            (double)S4(p+34)*pow(2,-10)*FREQL1/CLIGHT,	/* MID 64 Pseudorange Smooting		*/
            (double)rcvbias*CLIGHT/1e6,			/* MID 41 bias adjustment		*/
            (double)S2(p+42)/pow(2,16),			/* MID 64 Pseudorange noise estaminate 	*/
            prstd);

        if (debug) fprintf(stderr,"\nMID  4 CP+DR_valid=%d DR=%dms MDRT=%.1fs drq=%.9f | CP28=%.3fkm !CP64=%.3f! dL=%.3fkm PhLQ=%.3fm DRQ=%.3fm PhErr=%2d/s cpstd=%dm dClk=%.3f",
            (auxData[sat].trackStat&0x08)>>3,auxData[sat].DeltaRange, auxData[sat].MeanDR,(double)S2(p+44)/pow(2,16), 
            auxData[sat].CarrPhase28/1000, 
            (double)S4(p+14), 				/* MID 64: Carrier Phase */
            (double)S4(p+30)*FREQL1/CLIGHT/1000, 	/* MID 64: Code Correction dL=0.131km		*/
            (double)S2(p+46)/pow(2,8)*FREQL1/CLIGHT,    /* MID 64: Phase Lock Quality PhLQ -> qualP	*/
            (double)S2(p+44)/pow(2,16)*FREQL1/CLIGHT, 	/* MID 64: Delta Range Quality DRQ		*/ 
            auxData[sat].PhErrCnt,
            cpstd,(double)auxData[sat].CarrPhase28*FREQL1/CLIGHT-(double)rcvbias/1000000000*FREQL1);
    } return 0;

    /* DEBUG TODO LIST 

       This is the count of the millisecond interrupts from the start of the
       receiver (power on) until the measurement sample is taken. The ms
       interrupts are generated by the receiver clock 

       The number of time the phase lock fell below the threshold between the
       present code phase sample and the previous code phase sample. This task is
       performed every 20 ms (max count is 50). 

    /* NOTE * WORKING INFO - DO NOT DELETE *   fprintf(stderr,"%s [INFO ] G%02d prn=%+5d drq=%+5d plq=%+5d SQ=%3d SI=%3d NF=%s|%s PE=%2d TT=%5d SF=%s CNO=%2.1f el=%2.1f CP28=%.1f\n",time_str(utc2gpst(timeget()),0),sat,
                                                 S2(p+42),S2(p+44),S2(p+46),U2(p+52),U2(p+50),Bits(U1(p+3)),Bits(U1(p+4)),auxData[sat].PhErrCnt,auxData[sat].TimeTrack,
                                                 Bits(auxData[sat].SyncFlag),(float)auxData[sat].cno/4,auxData[sat].elev,
                                                 auxData[sat].CarrPhase28); 
       NOTE The reported Time of Measurement, Pseudorange and Carrier Phase are all uncorrected values	
       NOTE Discontinuity in this value generally means a cycle slip and renormalization to pseudorange.*/

    /* NOTE PSEUDORANGE dst->P[0] Meter
       PSEUDORANGE_RMS_ERROR: The SLC shall set this field to the pseudorange RMS error, in the range from 0.5m to 112m
                              Scale 1 - 65535 = 0.000015 - 0.999985 */
    
    /* NOTE CARRIER PHASE dst->L[0] [cycles] -> Mid 64 outputs L1 Cycles, M28 reports Meters			        */
    /* NOTE MID 4: Track Stat Bit 0 01 Acquisition/re-acquisition has been completed successfully -> HalfCycle! 
                              Bit 1 02 The integrated carrier phase is valid – delta range in MID 28 is also valid ##
                              Bit 2 04 Bit synchronization has been completed
                              Bit 3 08 Subframe synchronization has been completed
                              Bit 4 10 Carrier pullin has been completed (Costas lock) NOTE Carrier Phase ->L[0]
                                       Costas loop is a classical phase-locked loop (PLL) based circuit for carrier recovery and signal demodulation.
                                       The PLL is an automatic control system that adjusts the phase of a local signal to match the phase of the input reference signal.
                              Bit 5 20 Code has been locked ##
                              Bit 7 80 Ephemeris data is available                              */
    
    if (debug)  fprintf(stderr,"\nMID  4 CP+DR_valid=%d DR=%dms MDRT=%.1fs drq=%.9f | CP28=%.3fkm !CP64=%.3f! dL=%.3fkm PhLQ=%.3fm DRQ=%.3fm PhErr=%2d/s cpstd=%dm dClk=%.3f",
    (auxData[sat].trackStat&0x08)>>3,auxData[sat].DeltaRange, auxData[sat].MeanDR,(double)S2(p+44)/pow(2,16), 
    auxData[sat].CarrPhase28/1000, (double)S4(p+14), (double)S4(p+30)*FREQL1/CLIGHT/1000, (double)S2(p+46)/pow(2,8)*FREQL1/CLIGHT, (double)S2(p+44)/pow(2,16)*FREQL1/CLIGHT, auxData[sat].PhErrCnt,
    cpstd,(double)auxData[sat].CarrPhase28*FREQL1/CLIGHT-(double)rcvbias/1000000000*FREQL1);
    if (debug) fprintf(stderr,"\nP/L ratio =%.3fppm DL=%.3fm", (double)(1-(auxData[sat].PseudoR / (auxData[sat].CarrPhase28+1*S4(p+30)*FREQL1/CLIGHT) ))*pow(10,6),
    (double)rcvbias/1000000000*FREQL1/CLIGHT
    );        /* fprintf(stderr,"\nSAT %2d P=%.0fkm pAdj=%.6fs L=%.fk LLI=%d Q=%d %.3f %.3f",dst->sat,dst->P[0]/1000,Padj/CLIGHT,dst->L[0]/1000,dst->LLI[0],dst->qualP[0],auxData[dst->sat].QIRatio,(QImax-QImin)); */

/*  MID 7 contains the clock bias that must be considered. Adjust the GPS Software time by subtracting clock bias,
    adjust pseudorange by subtracting clock bias times the speed of light, and adjust 
    carrier phase by subtracting clock bias times speed of light/GPS L1 frequency. To adjust the reported 
    carrier frequency do the following: Corrected Carrier Frequency (m/s) = Reported Carrier Frequency (m/s) – Clock Drift (Hz)*C / 1575420000 Hz */

    /* NOTE Calculate standard deviation from ... auxData[sat].CarrFreq28*FREQL1/CLIGHT
                                           or ... S4(p+18)*0.000476	*/
/*    f3=RB_push(raw->cphase[sat][0],auxData[sat].CarrFreq28*FREQL1/CLIGHT,&sd); */


    auxData[sat].extStat=U1(p+4);
    auxData[sat].SmoothCode=(float)S4(p+34);
    auxData[sat].RecovStat=U1(p+62);		/*     fprintf(stderr,"XXXXRecov=%02X,SWU=%d",U1(p+62),U4(p+63));*/
    auxData[sat].TimeTag64=U4(p+6);
    auxData[sat].CarrPhase64=S4(p+14);		/* S2(p+22) = CarrAcc -> kann NULL sein -> Fehler! S4? */
    auxData[sat].CarrFreq64=S4(p+18)*0.000476;
    auxData[sat].CarrAcc=S2(p+22); 
    auxData[sat].PRNoise=S2(p+42); 
    auxData[sat].dRngQ=S2(p+44);
    auxData[sat].PhLockQ=S2(p+46);		/*     fprintf(stderr,"XXXX %04X->%d",U2(p+46),S2(p+46)); */
    return 0;
}

/* NOTE == Epemeris ==  NOTE */
static int decode_ephem(int sat, raw_t *raw)
{
    const int debug=0;
    eph_t eph={0};
    int k,j,x1,x2,x3;

    x1=decode_frame(raw->subfrm[sat-1]   ,&eph,NULL,NULL,NULL,NULL);
    x2=decode_frame(raw->subfrm[sat-1]+30,&eph,NULL,NULL,NULL,NULL);
    x3=decode_frame(raw->subfrm[sat-1]+60,&eph,NULL,NULL,NULL,NULL);
    if (debug) { fprintf(stderr,"%s [INFO ] Ephemeris Decoding Status: %d%d%d ",time_str(utc2gpst(timeget()),0),x1,x2,x3);
            for (j=0;j<3;j++) {
            fprintf(stderr,"\nSV %2d SF%d: ",sat,j+1);
            for (k=0;k<30;k++) { if (k%3==0) fprintf(stderr,"|"); fprintf(stderr,"%02X",U1(raw->subfrm[sat-1]+j*30+k)); }
            fprintf(stderr,"|IOD"); switch (j+1) {
            case 1: fprintf(stderr,"C=%02X",U1(raw->subfrm[sat-1]+168/8)); break;
            case 2: fprintf(stderr,"E=%02X",U1(raw->subfrm[sat-1]+30+48/8)); break;
            case 3: fprintf(stderr,"E=%02X",U1(raw->subfrm[sat-1]+60+216/8)); break;
        } } 
        fprintf(stderr,"\nSV %2d t=%d eph.iode=%02X raw->nav.eph[%2d].iode=%02X\n",sat,tickget(),eph.iode,sat,raw->nav.eph[sat-1].iode);
    }

    if (!strstr(raw->opt,"-EPHALL")) {
        if (eph.iode==raw->nav.eph[sat-1].iode&&                /*) return 0; <- stock*/
            eph.iodc==raw->nav.eph[sat-1].iodc) return 0; 	/* unchanged */
    }
    eph.sat=sat;
    raw->nav.eph[sat-1]=eph;
    raw->ephsat=sat;
    return 2;
}

static void writesf(raw_t *raw, int sat, int id, int tow)	{ /*unsigned char *p) {
    // Dateizeiger erstellen*/
    int i;
    FILE *fp;

    /* Datei oeffnen */
    fp = fopen("/tmp/SF.out", "a");

    if(fp == NULL) { fprintf(stderr,"Datei konnte NICHT geoeffnet werden.\n");
	} else {
	    fprintf(fp, "S t=%10d sat=%2d SF%d ",tickget(),sat,id);
	    for (i=0; i<30; i++) { 
	        fprintf(fp, "%02X", U1(raw->subfrm[sat-1]+30*(id-1)+i));
	        if (i==2||i==5||i==8||i==11||i==14||i==17||i==20||i==23||i==26) fprintf(fp,"|"); }
	    fprintf(fp,"\n"); /* if (x!=0) { fprintf(fp,"Y\n"); } else { fprintf(fp,"N\n"); } */
    	fclose(fp);
    }
}


/* NOTE DECODE MID 8 50BPS message (... & MID 56.5)
   NOTE decode_word is only used by sirf.c, not by ublox.c
Chris Kuethe "Message 8 is generated as the data is received. It is not buffered
             on the chip. So when you enable message 8, you'll get one subframe
             every 6 seconds. Of the data received, the almanac and ephemeris are
             buffered and stored, so you can query them at will. Alas, the time
             parameters are not stored, which is really lame, as the UTC-GPS
             correction changes 1 second every few years. Maybe."		*/
static int decode_sirf50bps(raw_t *raw)
{
    int debug=0,mid,prn,sat,id,word,i,DataID,PageID,SFID,week,toa,x5,svid,x1;
    unsigned int words[10];
    unsigned char *p=raw->buff+4; 	/* strip start sequence and payload	*/
    unsigned char subfrm[30];
    eph_t eph={0};			/* NOTE raw->nav.eph[sat-1]=eph; ??? */
    raw->len=(raw->len-4-4);		/* strip checksum and end sequence 	*/

    trace(4,"decode_sirf50bps: len=%d\n",raw->len);
    mid=U1(p); prn=U1(p+2); p+=3; SFID=(U1(p+6)&0x1c)>>2;

    /* NOTE Sanity Checks */
    if (raw->len!=43) 		   { trace(2,"SiRF 50BSP length error: len=%d\n",raw->len); return 0; }
    if (!(sat=satno(SYS_GPS,prn))) { trace(2,"SiRF 50BPS satellite number error: prn=%d\n",prn); return 0; }
    if (mid==MID_SRF50BPS && VerData==1 && SFID <= 3) return 0;  /* EXIT if Verified Ephermeris Data present     */
    
    /* NOTE U-Blox does not / is unable to perform a parity check ... so why check SiRF protocol for it? */
    word=U4(p); if (!decode_word(word,subfrm)) { trace(2, "SiRF 50BPS parity error: prn=%d\n",prn); return 0; }
        
    /* NOTE Save raw data to words array, taking swapped bits into account	*/
    for (i=0;i<10;i++) { 	
        words[i]=U4(p+4*i); if (words[i]&0x40000000) words[i]^=0x3FFFFFC0;
        words[i]=(words[i]&0x3fffffc0)>>6; 
    }
        
    /* NOTE Get Subframe ID 	*/
    id=(words[1]>>2)&7; SFID=id; if (id<1||5<id) { trace(2, "subfrm id error: sat=%2d id=%d len=%d\n",sat,id,raw->len); return -1; }

    /* NOTE Save words to raw subframe buffer, use writesf(raw,sat,id,tow) for debugging purposes */
    for (i=0;i<10;i++) setbitu(raw->subfrm[sat-1]+(id-1)*30,i*24,24,words[i]);	

    switch (id) {
        case 1: /* NOTE Store SV Clock data in AuxData-Array also */ 
            x1=decode_frame(raw->subfrm[sat-1]   ,&eph,NULL,NULL,NULL,NULL);
            auxData[sat].tgd=eph.tgd[0]; auxData[sat].f0=eph.f0; auxData[sat].f1=eph.f1; auxData[sat].f2=eph.f2;
            return 0;
        case 2: return 0; /* ... already saved */
        case 3:	return decode_ephem(sat,raw); /* Decode Subframe 1,2,3 to (precise) Ephemeris Data */
        case 4: /* NOTE Almanac SV 25-32 + Ionosphere + UTC 	*/
            DataID=(words[3]&0xc00000)>>22; PageID=(words[3]&0x3f0000)>>16; svid=getbitu(raw->subfrm[sat-1]+90,50,6);
            if (debug&&(svid==56||svid==63||svid==18)) { 	printf ("SF4 PRN=%2d DID=%d PID=%d svid=%d Data=",prn,DataID,PageID,svid);
                                for (i=0;i<30;i=i+3) fprintf(stderr,"%06X ",U4(raw->subfrm[sat-1]+(id-1)*30+i)>>8);
                                fprintf(stderr,"Valid="); }
            if (decode_frame(raw->subfrm[sat-1]+90,NULL,raw->nav.alm,raw->nav.ion_gps,raw->nav.utc_gps,&raw->nav.leaps)!=4) return 0;	/* if (debug&&(svid==56||svid==63||svid==18)) { (x4==4) ? putchar('Y') : putchar('N'); }*/
            switch (svid) {
                case 63: 
                    if (debug) { fprintf(stderr," SAT %2d SF4:",prn); fprintf(stderr,"svconf=%d svh=%d\n",raw->nav.alm[sat-1].svconf,raw->nav.alm[sat-1].svh ); }
                    break;
                case 18: break;
                case 56: if (debug) { fprintf(stderr,"\nSAT %2d SF4:",prn);
                                      for (i=0;i<8;i++) fprintf(stderr,"ion[%d]=%.3f ",i,raw->nav.ion_gps[i]);
                                      for (i=0;i<4;i++) fprintf(stderr,"utc[%d]=%.3f ",i,raw->nav.utc_gps[i]);
                                      fprintf(stderr,"\nleaps=%d",raw->nav.leaps); }
                         adj_utcweek(raw->time,raw->nav.utc_gps);
                         return 9;
            }
            return 0;
        case 5:	/* NOTE Almanac SV 1-24 			*/
            DataID=(words[3]&0xc00000)>>22; PageID=(words[3]&0x3f0000)>>16; svid=getbitu(raw->subfrm[sat-1]+(id-1)*30,50,6);
            if (PageID>=1&&PageID<=24) { /* NOTE Almanac Data af0 2⁻20 af1 2-38 */
                if (debug) fprintf(stderr,"af1=%fs/s af0=%fs",(float)((words[10-1]&0x00ffe0)>>5)*P2_43,(float)(((words[10-1]&0x1c)>>2)+(8*(words[10-1]&0xff0000)>>16)*P2_31));}
            if (debug&&svid==51) { printf ("SF5 PRN=%d DID=%d PID=%d svid=%d Data=",prn,DataID,PageID,svid);
                                   for (i=0;i<30;i=i+3) fprintf(stderr,"%06X ",U4(raw->subfrm[sat-1]+(id-1)*30+i)>>8);
                                   fprintf(stderr,"Valid="); }
            x5=decode_frame(raw->subfrm[sat-1]+120,NULL,raw->nav.alm,NULL,NULL,NULL);
            if (debug&&svid!=51) { (x5==5) ? putchar('Y') : putchar('N'); fprintf(stderr,"\n"); } 
            if (svid!=51) return 0;	
            toa=(words[2]&0xff00)>>8; week=words[2]&0xff;
            raw->nav.alm[sat-1].week=week;				/* Almanac Woche setzen */
            if (debug) fprintf(stderr,"SAT %2d SF5: toas=%f week=%d, toa=%f, toa=%d\n", sat, raw->nav.alm[sat-1].toas, raw->nav.alm[sat-1].week, raw->nav.alm[sat-1].toa.sec,toa);
            return 0;
        default: 
            fprintf(stderr,"\nMID #%d: Subframe %d detected!\n",mid,id); 
            return 0;
    }
    return 0;
}
/* NOTE DECODE MID 14 Almanac Data */
static int decode_sirfalm(raw_t *raw)
{
    int prn,sat,week,id,status,x5; /* ,ckSum=0; */
    unsigned char *D=raw->buff+5;
    unsigned char subfrm[25];
    /* return 0; DISABLED? */

    id=(U1(D)&0xC0)>>6;		/* DataID */
    prn=U1(D)&0x3F;	   	/* SVid   */
    week=(U2(D+1)&0xFFC0)>>6;  	/* Week   */
    status=U2(D+1)&0x003F;	/* Status */
    
    if (!(sat=satno(SYS_GPS,prn))) {
        trace(2,"SiRF satellite number error: prn=%d\n",prn);
        return -1;
    }
    trace(4,"decode_sirfalm: sat=%2d\n",sat); /* fprintf(stderr,"\nDECODE_ALM_0x0E[%d]:id=%2d:sat=%2d:week=%d:status=%d:",raw->len-9,id,sat,week,status); printBits(status); */

    /* NOTE Check Data Alignment!	*/
    memcpy(subfrm+1,D+3,24);
    subfrm[0]=4+16; 			/* subframe id#5 setzen! */
    memcpy(raw->subfrm[sat-1]+120+5,subfrm,25);
    raw->nav.alm[sat-1].week=week;	/* Almanac Woche setzen */

    /* decode subframe #5 */
    x5=decode_frame(raw->subfrm[sat-1]+120,NULL,raw->nav.alm,NULL,NULL,NULL); /* fprintf(stderr," x5=%d\n",x5); */
    return x5=5?0:-1;
}

/* NOTE DECODE MID 15 Ephemeris Data	
        It is essential to consult with GPS-ICD documentation to become more familiar with 
        conversions. For more information, see https://www.gps.gov/technical/icwg/IS-GPS-200K.pdf
        ICD200 X(24bits) 50-bps, 24-bit data word (See GPS ICD 200) Subframe 1,2,3 Words 2-10
        SiRF   X(16bits) SiRF Data structure per subframe, D[0] -> D[14], 2 byte words            */
static int decode_sirfeph(raw_t *raw)
{
    eph_t eph={0};
    int prn,sat,j;
    unsigned char *p=raw->buff+5; 
    int x1,x2,x3;
    /* return 0; Reenabled to check */
  
    prn=U1(p);
    if (!(sat=satno(SYS_GPS,prn))) {
        trace(2,"SiRF satellite number error: prn=%d\n",prn);
        fprintf(stderr,"SiRF satellite number error: prn=%d\n",prn);
        return -1;
    }

    trace(4,"decode_sirfeph: sat=%2d\n",sat);

    for (j=0;j<3;j++) memcpy(raw->subfrm[sat-1]+j*30,p+1+j*30,30);

    fprintf(stderr,"=== MID 15 MID 15 MID 15 ===");
    x1=decode_frame(raw->subfrm[sat-1]   ,&eph,NULL,NULL,NULL,NULL);
    x2=decode_frame(raw->subfrm[sat-1]+30,&eph,NULL,NULL,NULL,NULL);
    x3=decode_frame(raw->subfrm[sat-1]+60,&eph,NULL,NULL,NULL,NULL);
    if (x1==1&&x2==2&&x3==3) fprintf(stderr,"%s [INFO ] SV %2d Update of ephemeris data\n",time_str(utc2gpst(timeget()),0),sat);
    
     if (decode_frame(raw->subfrm[sat-1]   ,&eph,NULL,NULL,NULL,NULL)!=1||
            decode_frame(raw->subfrm[sat-1]+30,&eph,NULL,NULL,NULL,NULL)!=2||
            decode_frame(raw->subfrm[sat-1]+60,&eph,NULL,NULL,NULL,NULL)!=3) return 0;

    eph.sat=sat;
    raw->nav.eph[sat-1]=eph;
    raw->ephsat=sat;
    return 2;
}


/* NOTE DECODE MID 56 Verified 50 bps Broadcast Ephemeris and Iono Data	
        MID=56 SID=35 A0A2|0007|38|23|0100000000|005C|B0B3 -> SGEE Download Initiate -> This request is sent out if new SGEE file need is observed
        Extended Ephemeris Data/SGEE Download Output  23,29,2a               
        CGEE = Client Generated Extended Ephemeris - Satelitte Downlink, 
               remembers where it was so it knows roughly where it is.             
        SGEE = Server Generated Extended Ephemeris - TCP/IP-Link ->  DISABLE! */
static int decode_sirf50bpshq(raw_t *raw)
{
    unsigned char *p=raw->buff;
    int i,sid=U1(p+5); 
    switch (sid) {
        case 5:	/* NOTE Remove SubID and hand over to decode_sirf50bps */
            if (VerData == 0) { 
                fprintf(stderr,"%s [INFO ] Verified 50bps Data Stream detected.\n",time_str(utc2gpst(timeget()),0));
                VerData=1;
            }
            trace(4,"decode_sirf50bpshq: len=%d\n",raw->len);
            memmove(raw->buff+4+1, raw->buff+4+2, raw->len-6);	
            raw->len=(raw->len-1); 
            return decode_sirf50bps(raw);
        case 35: return 0; 
        case 41: return decode_sirfgen(raw);
        case 42: return decode_sirfgen(raw);
        default:
            fprintf(stderr,"MID=%d SID=%d ",U1(p+4),sid); for (i=0;i<raw->len;i++) fprintf(stderr,"%02X",U1(p+i));
            fprintf(stderr,"\n"); return decode_sirfgen(raw);
        }
    return 0;
}


/* NOTE DECODE MID 93 Temperature Value Output	*/
static int decode_sirftcxo(raw_t *raw)
{
    unsigned char *p; 
    int sid,i,timestat,week,tt;
    double tow,clkbias,clkdrift;
    obsd_t *obsd;

    trace(4,"decode_sirftcxo: len=%d\n",raw->len);

    p=raw->buff+4; sid=U1(p+1);
    if (sid != 18) return decode_sirfgen(raw); /* Exit on SubId other than #18 */
    
    if (raw->len!=34) {
        trace(2,"SiRF mid#93 length error: len=%d\n",raw->len);
        fprintf(stderr,"SiRF mid#93 length error: len=%d\n",raw->len);
        return -1;
    }
    
    tow=(double)U4(p+2)/100;
    timestat=U1(p+8)&4>>2?1:0;			/* [0] Week is set (T/F) [1] TOW is available (T/F) [2] TOW is precise (T/F) */
    week=U2(p+6);
    ClkBiasTCXO=(double)U4(p+18)/1000000000;	/* Set global Variable */
    clkbias=(double)U4(p+18)/1000000000;
    clkdrift=(double)U4(p+10);

    if (TimeDebug) fprintf(stderr,"%s [INFO ] MID 93.18: GPS TOW=%.2f TimeStatus=%d\n",time_str(utc2gpst(timeget()),0),tow,timestat); 
    
    /* Update TOW status */
    for (i=0;i<raw->obuf.n&&i<MAXOBS;i++) {
        obsd=&raw->obuf.data[i];
        obsd->timevalid=timestat;
    }

    tt = ClkUpd(raw, week, tow, clkbias, clkdrift, 93); 
    /* MID 7 contains the clock bias that must be considered...
       For a nominal clock drift value of 96.25 kHz (equal to a GPS Clock frequency of 24.5535 MHz), the correction
       value is 18315.766 m/s.
       Note:
       GPS Software Time – Clock Bias = Time of Receipt = GPS Time. GPS Software Time – Pseudorange (sec) =
       Time of Transmission = GPS Time. 
       TODO Adjust SV position in MID 30 by (GPS Time MID 30 – Time of Transmission) * Vsat. 
    
    if ( M28acq != U4(p+2)) { 				/ Check Acq clock update ... to be used with TimeTag of MID64 /
        dt=tow - M28TOW; dc=U4(p+2) - M28acq; 
        M28fclk= (double)dt/dc; M28acq = U4(p+2); M28TOW = tow;
        if (TimeDebug) fprintf(stderr,"%s [INFO ] MID 28: GPS TOW=%.2f TimeTag=%u\n",time_str(utc2gpst(timeget()),0),M28TOW,M28acq); 
    }*/

    /* NOTE Report TCXO info every 10 secondes */
    if (abs(InfoCnt0-tickget())>50000) { InfoCnt0=tickget();
        fprintf(stderr,"%s [INFO ] Temperature = %.1f° ",time_str(utc2gpst(timeget()),0),(float)(140*U1(p+22)/255)-40);
        fprintf(stderr,"Clock Drift = %d±%dHz ",U4(p+10),U4(p+14)); 
        fprintf(stderr,"Bias = %fs\n",ClkBiasTCXO);
    }
    return flushobuf(raw);
    
    fprintf(stderr,"TOW:%6u;",U4(p+2)); fprintf(stderr,"wk:%4hu;",U2(p+6));
    fprintf(stderr,"TStatus:"); printBits(U1(p+8));
    fprintf(stderr,";ClkOff:%d;",U1(p+9));
    fprintf(stderr,"ClkDrift:%d;",U4(p+10));
    fprintf(stderr,"ClkDriftU:%d;",U4(p+14));
    fprintf(stderr,"Temperature:%.1f;",(float)(140*U1(p+22)/255)-40);
    return 0;
}


/* NOTE DECODE MID 41 Geodetic Navigation Data 
        The pulse-per-second (PPS) output provides a pulse signal for timing purposes.
        Pulse length (high state) is 200ms about 1µs synchronized to full UTC second.
        The UTC time message is generated and put into output FIFO 300ms after PPS.
        The exact time between PPS and UTC time message delivery depends on message rate,
        message queue and communication baud rate.					*/
static int decode_sirfgeoclk(raw_t *raw)	/* SKYTRAQ 0xDC decode measurement epoch */
{
    unsigned char *p=raw->buff+4;
    double tow,clkbias,clkdrift;
    int week,tt;
    
    trace(4,"decode_sirfgeoclk: len=%d\n",raw->len);
    week=U2(p+5); 
    tow	=(double)U4(p+7)*0.001; 		/* Clock Bias will be substraced by ClkUpd sub */
    clkbias=(double)U4(p+64)/CLIGHT/100;
    clkdrift=(double)S4(p+72)*FREQL1/CLIGHT/100;
    altMSL=(float)S4(p+35)/100; 
    
    /* NOTE Most precise data in MID 41! */
    tt = ClkUpd(raw, week, tow, clkbias, clkdrift, 41); 

    if (raw->outtype) {
        sprintf(raw->msgtype,"SiRF Geodetic (%4d): week=%d tow=%.3f",
                raw->len,week,tow);
    }
    if (TimeDebug) fprintf(stderr,"%s [INFO ] MID 41: drift=%5.4fHz bias=%.4fns\n",time_str(utc2gpst(timeget()),0),(double)S4(p+72)*FREQL1/CLIGHT/100,(double)U4(p+64)/CLIGHT*1E7); 

    /* NOTE Report GPS position every 5 seconds */
    if ((abs(InfoCnt1-tickget())>5000)&&((U1(p+4)&0x0f)!=0)) { InfoCnt1=tickget();	
        fprintf(stderr,"%4d/%02d/%02d %02d:%02d:%02.0f [INFO ] GPS N:%.5f° E:%.5f° H%.2fm HDOP:%.1fm Q:%d:%d\n",U2(p+11),U1(p+13),U1(p+14),U1(p+15),U1(p+16),(float)U2(p+17)/1000,(float)S4(p+23)/1e+7,(float)S4(p+27)/1e+7,(float)S4(p+31)/100,(float)U1(p+89)/5,U1(p+4)&0x0f,U1(p+88));
    }

    return 0;

    fprintf(stderr,"ClkB:%08X->%d; ClkBE:%08X->%d",U4(p+64),U4(p+64),U4(p+68),U4(p+68));
    fprintf(stderr,"ClkDr:%08X->%4d; ClkDrE:%08X->%d;",U4(p+72),S4(p+72),U4(p+76),U4(p+76));
}

/* generic decode & print SiRF message --------------------------------
MID  2 0x02 D Measure Navigation Data Out
MID  4 0x04 D Measured Tracker Data Out 				!! Info, ob Carrier Phase in MID28 valide ist ...!
MID  6 0x06 D Software Version String
MID  7 0x07 X Clock Status Data = MID_SRFCLOCK
MID  8 0x08 X 50 bps data = MID_SRF50BPS
MID  9 0x09 D CPU Throughput
MID 11 0x0B D Command Acknowledgment
MID 13 0x0D X Visible List
MID 14 0x0E X Almanac Data (Polled)
MID 15 0x0F X Ephemeris Data (Polled)
MID 18 0x12 D OkToSend
MID 19 0x13 D Navigation Parameters (Polled)
MID 28 0x1C X Navigation Library Measuerment Data 			!! => MID_SRFNLMEAS     
MID 30 0x1E D Navigation Library SV State Data				!! ClockBias, ClockDrift + IonoDelay
MID 31 0x1F - Navigation Library Initalization Data
MID 41 0x29 D Geodetic Navigation Data					!! ClockBias -> MID 7 ab 40.000.000 m nehmen!
MID 50 0x32 D SBAS Parameters
MID 51 0x33 D Tracker Load Status Report
MID 56 0x38 X Extended Ephemeris Data/SGEE Download Output 		!! => SubMsgID 5 -> Exit tp MID_SRF50BPS
MID 64 0x40 D Navigation Library Messages -> TXCO Frequenz! HEAVY
MID 65 0x41 X GPIO State Output
MID 71 0x47 D Hardware Configuration Request ?? 1 Byte ...ID 71 A0A20001470047B0B3:
MID 91 0x5B D Hardware Control Output VCTCXO on/off, AGC Gain Output -> MID 166 zum Ausschalten
MID 92 0x5C - CW Controller Output -> Interferenzen ...
MID 93 0x50 X TXCO Learning Output Response  ! Frequenz ! Wichtig !
MID225 0xE1 - Sub ID 6 = Statistics Channel (-> gefunden in OSP Info, Rev. 8!)
... Abschalten M166? NO geht nur mit 2, 4, 28, 30, 41, 255 */
static int decode_sirfgen(raw_t *raw)
{
    int i,j,w,sid,vis;
    int marray[] = {MID_SRFAUXDATA, MID_SRFCLOCK, MID_SRF50BPS, MID_SRFNLMEAS, MID_SRFALM, MID_SRFEPH, 2, 9, 12, 13, 18, 41, 51, 56, 65, 71, 91, 92, 218, 225}; 
    unsigned gsw230=strstr(raw->opt,"-GSW230")!=NULL;
    
    unsigned char *message=raw->buff+4;
    int mid=U1(message);

    /* MID im Array enthalten? -> EXIT */
    /* for(i=0;i<sizeof(marray)/sizeof(int); i++) if (mid == marray[i] && mid != MID_SRFAUXDATA ) return 0; */

    switch (mid) {
    case  65: break; /* Skip GPIO State				*/
    case  71: break; /* Skip Hardware Configuration Request  	*/
    case 225: break; /* Unknown 				*/
    case 4:	/* Measured Tracker Data Out */
        fprintf(stderr,"wk:%4hu;",S2(message+1));
        fprintf(stderr,"TOW:%6u;",U4(message+3));
        fprintf(stderr,"Chans:%d;",U1(message+7));
        fprintf(stderr,"\n");
        for(j=0; j<=11; j++) {
            fprintf(stderr,"CH#%2d:",j+1);
            fprintf(stderr,"SV_id:%2d;",U1(message+8+j*15));
            fprintf(stderr,"Azimuth:%3d;",U1(message+9+j*15));
            fprintf(stderr,"Elev:%3d;",U1(message+10+j*15));
            fprintf(stderr,"State:");
            printBits(U1(message+12+j*15));
            fprintf(stderr,";Valid_Phase:");
            ((U1(message+12+j*15) & (1 << 1)) ? putchar('Y') : putchar('N'));
            fprintf(stderr,";Subframe_Complete:");
            ((U1(message+12+j*15) & (1 << 3)) ? putchar('Y') : putchar('N'));
            fprintf(stderr,";Costas_Lock:");
            ((U1(message+12+j*15) & (1 << 4)) ? putchar('Y') : putchar('N'));
            fprintf(stderr,";Code_Lock:");
            ((U1(message+12+j*15) & (1 << 5)) ? putchar('Y') : putchar('N'));
            fprintf(stderr,"\n");
        }
        break;
    case 9:	/* CPU Throughput */
        fprintf(stderr,"SegStatMax:%4u;",U2(message+1));
        fprintf(stderr,"SegStatLat:%4u;",U2(message+3));
        fprintf(stderr,"AveTrkTime:%4u;",U2(message+5));
        fprintf(stderr,"LastMilliS:%4u;",U2(message+7));
        break;
    case 11:	/* Command Acknowledgement */
        fprintf(stderr,"ACI:%d,SID:%d;",U1(message+2),U1(message+1));
        break;
    case 13:	/* Visible List, update rate: every 2 minutes? */
        vis = U1(message+1);
        /* fprintf(stderr,"VisSV:%d\n",vis); */
        break; 
        for(j=0; j<vis; j++) {
            fprintf(stderr,"Ch%02d:SV%2d:",j,U1(message+2+j*5));
            fprintf(stderr,"Az:%d,El:%d\n",S2(message+3+j*5),S2(message+5+j*5));
        }
        break;
    case 18:	/* OkToSend */
        fprintf(stderr,"%s [INFO ] MID 18 OK to send: %s\n",time_str(utc2gpst(timeget()),0),
        (U1(message+1) == 1)?"Y":"N");
        break;
    case 30:	/* Navigation Library SV State Data */
        /* The data in MID 30 reports the computed satellite position and velocity at the specified GPS time.
        * Note: *
        When using MID 30 SV position, adjust for difference between GPS Time MID 30 and Time of Transmission
        (see the note in MID 28). Iono delay is not included in pseudorange in MID 28. */
        fprintf(stderr,"\nSVid:%2d;",U1(message+1));
        fprintf(stderr,"GPSt:%1f;",R8(message+2,gsw230));
        fprintf(stderr,"X:%1f;",R8(message+10,gsw230));
        fprintf(stderr,"Y:%1f;",R8(message+18,gsw230));
        fprintf(stderr,"Z:%1f;",R8(message+26,gsw230));
        fprintf(stderr,"vx:%1f;",R8(message+34,gsw230));
        fprintf(stderr,"vy:%1f;",R8(message+42,gsw230));
        fprintf(stderr,"vz:%1f;",R8(message+50,gsw230));
        fprintf(stderr,"ClkBias:%1f;",R8(message+58,gsw230));
        fprintf(stderr,"ClkDrift:%f;",R4(message+66));
        fprintf(stderr,"EFV:%d;",U1(message+70));
        fprintf(stderr,"IonoDly:%f;",R4(message+79));
        break;
    case 31:	/* Navigation Library Initialization Data */
        fprintf(stderr,"%s [INFO ] MID 31: ",time_str(utc2gpst(timeget()),0));
        fprintf(stderr,"AltMode=%d AltSrc=%d Alt=%f ",U1(message+2),U1(message+3),R4(message+4));
        fprintf(stderr,"age=%.1f ",(float)(lweek-U2(message+72))/52);
        fprintf(stderr,"TimeInitSrc=%d ",U1(message+74));
        fprintf(stderr,"Drift=%1f ",R8(message+75,gsw230));
        fprintf(stderr,"DriftInitSrc=%d\n",U1(message+83));
        break;
    case 41:	/* Geodetic Navigation Data */
        fprintf(stderr,"NValid:%d;",U2(message+1));
        fprintf(stderr,"NType:");
        printBits(U1(message+3));
        printBits(U1(message+4));
        fprintf(stderr,";ExtWN:%d;",U2(message+5));
        fprintf(stderr,"TOW:%d;",U4(message+7));
        fprintf(stderr,"UTC:%4d-%2d-%2d %2d:%2d:%2.1f;",U2(message+11),U1(message+13),U1(message+14),U1(message+15),U1(message+16),(float)U2(message+17)/1000);
        fprintf(stderr,"SatList:");
        printBits(U1(message+19));
        printBits(U1(message+20));      
        printBits(U1(message+21));      
        printBits(U1(message+22));      
        fprintf(stderr,"Lat:%4d;",S4(message+23));
        fprintf(stderr,"Lon:%4d;",S4(message+27));
        fprintf(stderr,"AltEps:%4d;",S4(message+31));
        fprintf(stderr,"AltMSL:%4d;",S4(message+35));
        fprintf(stderr,"Map:%d;",U1(message+39));
        fprintf(stderr,"SOG:%d;",U2(message+40));
        fprintf(stderr,"COG:%d;",U2(message+42));
        fprintf(stderr,"MagV:%d;",S2(message+44));
        fprintf(stderr,"Climb:%d;",S2(message+46));
        fprintf(stderr,"HeadR:%d;",S2(message+48));
        fprintf(stderr,"EHPE:%d;",U4(message+50));
        fprintf(stderr,"EVPE:%d;",U4(message+54));
        fprintf(stderr,"ETE:%d;",U4(message+58));
        fprintf(stderr,"ETVE:%d;",U2(message+62));
        fprintf(stderr,"ClkB:%08X->%d;",U4(message+64),U4(message+64));
        fprintf(stderr,"ClkBE:%08X->%d;",U4(message+68),U4(message+68));
        fprintf(stderr,"ClkDr:%08X->%4d;",U4(message+72),S4(message+72));
        fprintf(stderr,"ClkDrE:%08X->%d;",U4(message+76),U4(message+76));
        fprintf(stderr,"Dist:%d;",U4(message+80));
        fprintf(stderr,"DistE:%d;",U2(message+84));
        fprintf(stderr,"HeadE:%d;",U2(message+86));
        fprintf(stderr,"NrSVFix:%d;",U1(message+88));
        fprintf(stderr,"HDOP:%.1f;",(float)U1(message+89)/5);
        fprintf(stderr,"AddModeInf:");
        printBits(U1(message+90));
        break;        
    case 50:	/* SBAS Parameters */
        fprintf(stderr,"SBASsv:%3d;", U1(message+1));
        fprintf(stderr,"Mode:%3d;", U1(message+2));
        fprintf(stderr,"DGPS_Timeout:%3d;", U1(message+3));
        fprintf(stderr,"FlagBits:");
        printBits(U1(message+4));
        break;
    case 51:	/* Tracker Load Status Report ... liefert entgegen der Doku nur zwei Byte zurück*/
        fprintf(stderr,"SubID:%d;",U1(message+1));
        fprintf(stderr,"LoadState:%d;",U4(message+2));
        fprintf(stderr,"LoadError:%d;",U4(message+10));
        fprintf(stderr,"TimeTag:%d;",U4(message+14));
        break;
    case 56:	/* Extended Ephemeris Data/SGEE Download Output  23,29,2a	 	*/
                /* CGEE = Client Generated Extended Ephemeris - Sattelitte Downlink, 
                   remembers where it was so it knows roughly where it is.		*/
                /* SGEE = Server Generated Extended Ephemeris - TCP/IP-Link ->  DISABLE!*/
        break;
        sid=U1(message+1);
        fprintf(stderr,"\nMID=56.%d:",sid);
        switch (sid) {
            case 35:	/* SID=35 SGEE Download Initiate - Message		*/
                fprintf(stderr,"Start:%d;",U1(message+2));
                fprintf(stderr,"Wait_time:%4d;",U4(message+3));
                break;
            case 41:	/* SID=41 SIF Aiding Status   				*/
                fprintf(stderr,"SGEE_S:%d;",U1(message+2));
                fprintf(stderr,"CGEE_S:%d;",U1(message+3));
                fprintf(stderr,"CGEE_P:%4u;",U4(message+4));
                fprintf(stderr,"RcvTime:%4d;",U4(message+8));
                break;
            case 42:	/* SID=42 SIF Status Message  				*/
                fprintf(stderr,"SIFState:%d;",U1(message+2));
                fprintf(stderr,"cgeePredS:%d;",U1(message+3));
                fprintf(stderr,"sifAidTyp:%d;",U1(message+4));
                fprintf(stderr,"sgeeDlPr:%d;",U1(message+5));
                fprintf(stderr,"cgeePrTL:%4d;",U4(message+6));
                fprintf(stderr,"cgeePrPM:%4d;",U4(message+10));
                fprintf(stderr,"svidCGEEPrP:%d;",U1(message+14));
                fprintf(stderr,"sgeeAgeValidy:%d;",U1(message+15));
                break;
        }
        break;
    case 64:	/* Navigation Library Messages	*/
        sid=U1(message+1);
        switch (sid) {
            case 1:	/* SID=1 Auxiliary Initialization Data 	*/
                fprintf(stderr,"%s [INFO ] MID 64.%d: ",time_str(utc2gpst(timeget()),0),sid);
                fprintf(stderr,"uTime=%d:,",U4(message+2));
                fprintf(stderr,"wk=%d:",U2(message+6));
                fprintf(stderr,"tow=%d:",U4(message+8));
                fprintf(stderr,"UHP=%d:",U2(message+12));
                fprintf(stderr,"UAP=%d:",U2(message+14));
                fprintf(stderr,"sw=%d:",U1(message+16));
                fprintf(stderr,"icd=%d:",U1(message+17));
                fprintf(stderr,"hwid=%d:",U2(message+18));
                fprintf(stderr,"ClkRate=%d:",U4(message+20));
                fprintf(stderr,"FrqOff=%d:",U4(message+24));
                fprintf(stderr,"Status:");
                ((U1(message+32) & (1 << 0)) ? fprintf(stderr,"Bad") : fprintf(stderr,"Good"));
                fprintf(stderr,"Cache:");
                ((U1(message+32) & (1 << 1)) ? fprintf(stderr,"Enabled") : fprintf(stderr,"Disabled"));
                fprintf(stderr,"RTC:");
                ((U1(message+32) & (1 << 1)) ? fprintf(stderr,"Valid") : fprintf(stderr,"Invalid"));
                fprintf(stderr,"\n");
                break;
            case 2:	/* SID=2 Auxiliary Measurement Data 	*/
                fprintf(stderr,"\nMID#64:SAT=%2d",U1(message+2));
                fprintf(stderr,"SVid:%2d;",U1(message+2));
                fprintf(stderr,"Status:");
                printBits(U1(message+3));
                fprintf(stderr,";ExtStat:");
                printBits(U1(message+4));
                fprintf(stderr,";BitSyncQ:%d;",U1(message+5));
                fprintf(stderr,"TimeTag:%4d;\n",U4(message+6));
                fprintf(stderr,"CodePh:%4d;",U4(message+10));
                fprintf(stderr,"CarrPh:%4d;",S4(message+14));
                fprintf(stderr,"CarrFrq:%4d;",S4(message+18));
                fprintf(stderr,"CarrAcc:%4d;",S2(message+22));
                fprintf(stderr,"MilliS:%2d;",U2(message+24));
                fprintf(stderr,"BitNr:%2d;",U4(message+26));
                fprintf(stderr,"CodeCor:%4d;",S4(message+30));
                fprintf(stderr,"SmothCd:%4d;",S4(message+34));
                fprintf(stderr,"CodeOff:%4d;",S4(message+38));
                fprintf(stderr,"PsRN:%2d;\n",S2(message+42));
                fprintf(stderr,"DRQ:%2d;",S2(message+44));
                fprintf(stderr,"PhLckQ:%2d;",S2(message+46));
                fprintf(stderr,"msU:%2d;",S2(message+48));
                fprintf(stderr,"SumAbsI:%2d;",U2(message+50));
                fprintf(stderr,"SumAbsQ:%2d;",U2(message+52));
                fprintf(stderr,"SVBNr:%2d;",S4(message+54));
                fprintf(stderr,"MPLOS:%2d;",S2(message+58));
                fprintf(stderr,"MPODV:%2d;",S2(message+60));
                fprintf(stderr,"RecovS:%d;",U1(message+62));
                fprintf(stderr,"SWTU:%4d;",U4(message+63)); 
                
/*                fprintf(stderr,"nCarrFrq:%08X->%4d;",U4(message+18),S4(message+18));*/
/*                fprintf(stderr,":CFrq=%10dHz:",S4(message+18));
                fprintf(stderr,"CAcc=%7d:",S2(message+22));
                fprintf(stderr,"CarrPh=%11dCy:",S4(message+14));
                fprintf(stderr,"PsRN=:%6d:",S2(message+42));
                fprintf(stderr,"DRQ=%6d:",S2(message+44));
                fprintf(stderr,"PhLckQ=%6d:",S2(message+46));
                fprintf(stderr,"RecStat=");
                printBits(U1(message+62));
                fprintf(stderr,":SWTU:%3d:",U4(message+63)); 
                fprintf(stderr,":CFR=%4dHz,CFS=%f,",S4(message+18),S4(message+18)*0.000476);   *0.000476*(CLIGHT/FREQL1)); */

                fprintf(stderr," Frequency[m/s]= (%10.0f)",S4(message+18)*0.000476);	/* Hz		*/
                /*fprintf(stderr,"  Phase[cy]= %10ld",S4(message+14)); 	* Cycles	*/
                fprintf(stderr,"  Phase[cy]= %10d ",S4(message+14));
                fprintf(stderr,"  Status="); /*%d",(U1(message+3)&0x02)>>1); */
                printBits(U1(message+3));
                fprintf(stderr,":ExtStat=");
                printBits(U1(message+4));
                break;
            case 3:
                if (TimeDebug) fprintf(stderr,"%s [INFO ] MID 64.3: GPS TOW=%.3f\n",time_str(utc2gpst(timeget()),0),(double)U4(message+21)/1000); 
                break;
            default:
                fprintf(stderr,"\nDECODE_OSP:mid=%2d[%4d.%d]:",mid,sid,raw->len);
                for (i=1;i<raw->len-8;i++) fprintf(stderr,"%02X ", U1(message+i) ); 
                break;
        }
        break;
    case 70:	/* Ephemeris Status Response ... seltene Ausgabe? */ 
        sid=U1(message+1);
        fprintf(stderr,"EPH-SubID:%d;",sid);
        switch (sid) {
            case 1:	/* SID=1 EPH_RESP   			Ephemeris Status Response   				*/
                break;
            case 2:	/* SID=2 ALM_RESP   			Almanac Respone            				*/
                break;
            case 3:	/* SID=3 B_EPH_RESP 			Broadcast Ephemeris Response				*/
                break;
            case 4:	/* SID=4 TIME_FREQ_APPROX_POS_RESP	Time Frequency Approximate Position Status Response	*/
                break;
            case 5:	/* SID=5 CH_LOAD_RESP			Channel Load Response					*/
                break;
            case 6:	/* SID=6 CLIENT_STATUS_RESP		Client Status Response					*/
                break;
            case 7:	/* SID=7 OSP_REV_RESP			OSP Revision Response					*/
                break;
        } 
        break;
    case 91:	/* Hardware Control Output VCTCXO on/off, AGC Gain Output */
        fprintf(stderr,"%s [INFO ] MID 91: AGC Gain=%d",time_str(utc2gpst(timeget()),0),U1(message+2));
        break;
    case 93:	/* TXCO Learning Output Response */
        w=U1(message+1);
        fprintf(stderr,"%s [INFO ] MID 93.%d: ",time_str(utc2gpst(timeget()),0),w);
        switch (w) {
            case 1:	/* TCXO Learning Clock Model Data Base */
                fprintf(stderr,"Source:");
                printBits(U1(message+2));
                fprintf(stderr,";AgeRateU:%d;InitOffU:%d;",U1(message+3),U1(message+4));
                fprintf(stderr,"ClkDrift:%f;",R4(message+6));
                fprintf(stderr,"TempU:%d;",U2(message+10));
                fprintf(stderr,"ManWkN:%d;\n",U2(message+12));
                break;
            case 2: 	/* TCXO Learning Temperature Table  ppb, TBD: Signed statt unsigned! */
                fprintf(stderr,"FreqOffset:%4d;",S2(message+6));
                fprintf(stderr,"GlobalMin:%4d;",S2(message+8));
                fprintf(stderr,"GlobalMax:%4d;\n",S2(message+10));
                break;                
            case 18:	/* Temperature Voltage Output */
                /* for (i=1;i<raw->len-8;i++) fprintf(stderr,"%02X ", U1(message+i) ); */
                fprintf(stderr,"TOW:%6u;",U4(message+2));
                fprintf(stderr,"wk:%4hu;",U2(message+6));
                fprintf(stderr,"TStatus:");
                printBits(U1(message+8));
                fprintf(stderr,";ClkOff:%d;",U1(message+9));
                fprintf(stderr,"ClkDrift:%d;",U4(message+10));
                fprintf(stderr,"ClkDriftU:%d;",U4(message+14));
                fprintf(stderr,"ClkBias:%f;",(double)U4(message+18)/1000000000);
                ClkBiasTCXO=U4(message+18)/1000000000;
                fprintf(stderr,"Temperature:%.1f;",(float)(140*U1(message+22)/255)-40);
                break;
            default:
                fprintf(stderr,"\nMID=93.%02d:",w);
        }
        break;
    case 1225:	/* Data Log */
        fprintf(stderr,"\n225-SubID:%d;",U1(message+1)); 
        for (i=-4;i<raw->len-4;i++) fprintf(stderr,"%02X ", U1(message+i) ); 
        fprintf(stderr,"\n");
        break;
    default:
        fprintf(stderr,"\nDECODE_OSP:mid=%2d[%4d]:",mid,raw->len);
        for (i=0;i<raw->len;i++) fprintf(stderr,"%02X", U1(message-4+i) );
        fprintf(stderr,"\n");
    }
    return 0;
}

static void UpdFW(void) {
    int stat[MAXSTR]={0},byte[MAXSTR]={0},bps[MAXSTR]={0},ret=0,i;
    char strmsg[MAXSTRMSG]="";
    unsigned char buff[1024];
/*  A0 A2 00 02 - Start Sequence and Payload Length (2 bytes)
    84 00 	- Payload
    00 84 B0 B3 - Message Checksum and End Sequence */
    unsigned char PollSW[]={0xA0,0xA2,0x00,0x02,0x84,0x00,0x00,0x84,0xB0,0xB3};
 
/*  A0 A2 00 03 - Start sequence and payload length (3 bytes)
    93 00 00 - Payload
    00 92 B0 B3 - Message checksum and end sequence */
    unsigned char PollEPH[]={0xA0,0xA2,0x00,0x03,0x93,0x00,0x00,0x00,0x93,0xB0,0xB3};

/*  A0 A2 00 02 – Start Sequence and Payload Length (2 bytes)
    98 00 – Payload
    00 98 B0 B3 – Message Checksum and End Sequence 
    Response:
    MID 19: A0A200411300000011020000000400000000010000000000006420000000000002000000000000000003E8000003E80000000000000001D4C0000075300A0000000100000104CDB0B3 */
    unsigned char PollNAV[]={0xA0,0xA2,0x00,0x02,0x98,0x00,0x00,0x98,0xB0,0xB3};
    unsigned char PollNavPar[]={0xA0,0xA2,0x00,0x02,0xb2,0x0b,0x00,0xbd,0xB0,0xB3};
    unsigned char Testmode[]={0xA0,0xA2,0x00,0x08, 0x96,0x1E,0x51,0x00,0x06,0x00,0x1E,0x00, 0x01,0x29,0xB0,0xB3};
    unsigned char PatchStart[]={0xA0,0xA2,0x00,0x01,0xB2,0x28,0x00,0x28,0xB0,0xB3};
    unsigned char PatchEnd[]={0xA0,0xA2,0x00,0x01,0xB2,0x26,0x00,0x26,0xB0,0xB3};
    unsigned char FlashUpd[]={0xA0,0xA2,0x00,0x02,0x94,0x00,0x00,0x94,0xB0,0xB3};
    unsigned char FlashEnd[]={0xA0,0xA2,0x00,0x02,0x84,0x00,0x00,0x84,0xb0,0xb3};

    char * buffer = 0;
    unsigned char *bi=buffer;
    long length;
    FILE * f = fopen ("/opt/RTKLIB/SiRF/GSD4e_4.1.2_P1_RPATCH_10.pd2","rb"); /* HARDCODED for testing purposes */

    if (f) { /* read rom patch into buffer */
        fseek (f, 0, SEEK_END);
        length = ftell (f);
        fseek (f, 0, SEEK_SET);
        buffer = malloc (length);
        if (buffer) { fread (buffer, 1, length, f); }
        fclose (f);
    } /* Verification: fprintf(stderr,"L=%ld ...",length); for (i=0;i<10;i++) fprintf(stderr,"%02X ", U1(buffer+i) ); fprintf(stderr,"\n"); */

/*  ret=strwrite(strsvr.stream,FlashUpd,sizeof(FlashUpd));
  fprintf(stderr,"STATUS=%d\n",ret);

  ret=strwrite(strsvr.stream,buffer,length);
  fprintf(stderr,"STATUS=%d\n",ret);

  ret=strwrite(strsvr.stream,FlashEnd,sizeof(FlashEnd));
  fprintf(stderr,"STATUS=%d\n",ret); */

  /*ret=strwrite(strsvr.stream,Testmode,sizeof(Testmode));
  fprintf(stderr,"STATUS=%d\n",ret); */

}

/* NOTE DECODE MID 19 Navigation Parameters	
DECODE_OSP:mid=19[  73]:
A0A2004113
00000011020000000400000000000000000000006420000000000002000000000000000003E8000003E80000000000000001D4C0000075300A00000001000001
04CCB0B3*/
static int decode_sirfnavpar(raw_t *raw)
{
    int PosCalcMode,AltHoldMode,AltHoldSrc,DegMode,DegTO,DRTO,TrkSmth,StatNav,TSVLsq,DOPMM,NavPowM,DGPSSrc,DGPSM,DGPSTO,LPPTF,UTEn;
    int LPPCEn,LPMaxAcq,LPMaxOff,APMEn,NumFix,TimBtwnFix,HVErrMax,RspTim,DtyCycPrio;
    double AltSrcInp,NavElevM,LPOT,LPIntv,UTIntv;
    unsigned char *p; 
    trace(4,"decode_sirfnavpar: len=%d\n",raw->len);
    p=raw->buff+4; 
    
    if (raw->len!=73) {
        trace(2,"SiRF mid#19 length error: len=%d\n",raw->len);
        fprintf(stderr,"SiRF mid#19 length error: len=%d\n",raw->len);
        return -1;
    }
    PosCalcMode=U1(p+4); 	/* MID 136 */
    AltHoldMode=U1(p+5);
    AltHoldSrc=U1(p+6);
    AltSrcInp=S2(p+7);
    DegMode=U1(p+9);
    DegTO=U1(p+10);
    DRTO=U1(p+11);
    TrkSmth=U1(p+12);
    StatNav=U1(p+13);		/* MID 143 */
    TSVLsq=U1(p+14);
    DOPMM=U1(p+19);		/* MID 137 */
    NavElevM=S2(p+20);		/* MID 139 */
    NavPowM=U1(p+22);		/* MID 140 */
    DGPSSrc=U1(p+27);		/* MID 133 */
    DGPSM=U1(p+28);		/* MID 138 */
    DGPSTO=U1(p+29);
    LPPTF=U1(p+34);		/* MID 151 */
    LPOT=S4(p+35);
    LPIntv=S4(p+39);
    UTEn=U1(p+43);
    UTIntv=S4(p+44);
    LPPCEn=U1(p+48);
    LPMaxAcq=U4(p+49);
    LPMaxOff=U4(p+53);
    APMEn=U1(p+57);
    NumFix=U2(p+58);
    TimBtwnFix=U2(p+60);
    HVErrMax=U1(p+62);
    RspTim=U1(p+63);
    DtyCycPrio=U1(p+64);
    fprintf(stderr,"%s [INFO ] MID 19: PosCalcMode=%02X AltHoldMode=%02x AH_Src=%02X AH_Input=%.1f\n",time_str(utc2gpst(timeget()),0),PosCalcMode,AltHoldMode,AltHoldSrc,AltSrcInp); 
    fprintf(stderr,"%s [INFO ] MID 19: DegradedMode=%02X Timeout=%ds Dead Reckoning Timeout=%d\n",time_str(utc2gpst(timeget()),0),DegMode,DegTO,DRTO); 
    fprintf(stderr,"%s [INFO ] MID 19: TrackSmooth=%02X Static=%02X 3SV least sq=%d\n",time_str(utc2gpst(timeget()),0),TrkSmth,StatNav,TSVLsq); 
    fprintf(stderr,"%s [INFO ] MID 19: DOP Mask Mode=%02X Min.Elev=%2.1f MinPower=%d\n",time_str(utc2gpst(timeget()),0),DOPMM,NavElevM,NavPowM); 
    fprintf(stderr,"%s [INFO ] MID 19: LP=%02X On-time=%.2f Interval=%.2f PwrCycEn=%02X MaxAcqT=%d MaxOffT=%d\n",time_str(utc2gpst(timeget()),0),LPPTF,LPOT,LPIntv,LPPCEn,LPMaxAcq,LPMaxOff); 
    fprintf(stderr,"%s [INFO ] MID 19: User Tasks=%02X Interval=%.2f APM/Pwr Duty Cycle=%02X\n",time_str(utc2gpst(timeget()),0),UTEn,UTIntv,APMEn); 
    fprintf(stderr,"%s [INFO ] MID 19: Number of fixes=%d Time between fixes=%d H/V max. error=%ds\n",time_str(utc2gpst(timeget()),0),NumFix,TimBtwnFix,HVErrMax); 
    fprintf(stderr,"%s [INFO ] MID 19: Response Time=%d Time/Accu Prio=%02X\n",time_str(utc2gpst(timeget()),0),RspTim,DtyCycPrio);  
}

/* NOTE Decode MID 178 Tracker Configuration
DECODE_OSP:mid=178[ 120]:A0A20070B2
0C0001C200000000620000006000000000000003FFFFFFFFFF4583400003FC03FC0004003E0000007C0000000000000000000017C1180000000000010100010001030100000101040101000100100000000201010000030102000103FF0000000000000000000A0000000000000000 
0D2DB0B3*/


/* DECODE SiRF raw messages -------------------------------------------- */
static int decode_sirf(raw_t *raw)
{
    unsigned char *p=raw->buff;
    int mid=U1(p+4),sid,i;
    signed int status=0;
    

    trace(3,"decode_sirf: mid=%2d\n",mid);

    if (raw->buff[raw->len-2]!=SIRFSYNCEND1
            ||raw->buff[raw->len-1]!=SIRFSYNCEND2) {
        trace(2,"SiRF sync error");
        return -1;
    }
    if (!chksum(raw->buff,raw->len)) {
        trace(2,"SiRF message checksum error: mid=%d len=%d\n",mid,raw->len);
        return -1;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype,"SiRF %2d (%4d):",mid,raw->len);
    }
    /* NOTE Check Message Sequence */
    if (MC==0) { MBuf[0]=mid; MC++; } 
    else if (mid != MBuf[0]) {
        MC++; for (i=MC;i>0;i--) MBuf[i]=MBuf[i-1]; 
        MBuf[0]=mid;
    }

    switch (mid) {
        case MID_SRFCLOCK:	status=decode_sirfclock(raw);	break;
        case MID_SRFGEOCLK:	status=decode_sirfgeoclk(raw);	break;
        case MID_SRF50BPS:	status=decode_sirf50bps(raw);	break;
        case MID_SRF50BPSHQ:	status=decode_sirf50bpshq(raw); break;
        case MID_SRFNLMEAS:	status=decode_sirfnlmeas(raw);	break;
        case MID_SRFEPH:	status=decode_sirfeph(raw);	break;
        case MID_SRFALM:	status=decode_sirfalm(raw);	break;
        case MID_SRFSVSTATE:	status=decode_sirfsvstate(raw); break;
        case MID_SRFTCXO:	status=decode_sirftcxo(raw);	break;
        case MID_SRFMTDO:	status=decode_sirfmtdo(raw);	break;
        case MID_SRFAUXDATA:	status=decode_sirfauxdata(raw); break;
        case 19: 		status=decode_sirfnavpar(raw); 	break;
        case 2:	/* - MID 2, solution data: ECEF X, Y, Z, vX, vY, vZ, week, TOW and satellites used */
            if (TimeDebug) fprintf(stderr,"%s [INFO ] MID  2: GPS TOW=%.2f\n",time_str(utc2gpst(timeget()),0),(double)U4(p+28)/100); 
            /* for (i=1;i<=5;i++) fprintf(stderr,"%02X ", U1(p+i) );
            p=p+4;
            fprintf(stderr,"X:%8d;Y:%8d;Z:%8d",S4(p+1),S4(p+5),S4(p+9));
            fprintf(stderr,"vX:%4hd;vY:%4hd;vZ:%4hd;",S2(p+13),S2(p+15),S2(p+17));
            fprintf(stderr,";HDOP2:%d;",U1(p+20)); 
            fprintf(stderr,"Mode1:"); printBits(U1(p+19));
            fprintf(stderr,"Mode2:"); printBits(U1(p+21));
            fprintf(stderr,";wk:%4hu;",U2(p+22));
            fprintf(stderr,"TOW:%6u;",U4(p+24));
            fprintf(stderr,"SVs:%2d",U1(p+28)); */
            status=0; break;
        case 6:			/* MID  6 Software Version String 				*/
            fprintf(stderr,"%s [INFO ] Firmware = %s CV=%s\n",time_str(utc2gpst(timeget()),0),p+7,p+7+2+52); 
            UpdFW(); 
            status=0; break;
        case 255:		/* MID 255 ASCII Development Data Output */
            raw->buff[U2(p+2)+4]=0; fprintf(stderr,"%s [DEBUG] %s\n",time_str(utc2gpst(timeget()),0),p+5); 
            status=0; break;
        case 11: status=0;break;/* MID 11 Command Acknowledgment A0A2|0003|0B|8400|008F|B0B3 	*/
        case 12:		/* MID 12 Command Negative Acknowledgment			*/
            fprintf(stderr,"%s [WARN!] ",time_str(utc2gpst(timeget()),0));
            fprintf(stderr,"Check command script in terms of MID %d (=0x%2X)! ",U1(p+5),U1(p+5));
            /* for (i=0;i<raw->len;i++) fprintf(stderr,"%02X ",U1(p+i));  */
            fprintf(stderr,"\n");
            status=0; break;
        case 50:		/* MID 50 SBAS Parameters					*/
/*            if (U1(p+5)==0) { status=0; break; }	// Check SBAS PRN */
            fprintf(stderr,"%s [INFO ] ",time_str(utc2gpst(timeget()),0));
            /*[INFO ]A0A2000D32 000012000000000000000000 0044B0B3 
                             +4 +5+6+7                         
            for (i=0;i<raw->len;i++) fprintf(stderr,"%02X",U1(p+i)); */
            fprintf(stderr,"SBAS PRN:%3d ", U1(p+5));
            fprintf(stderr,"Integretiy:%3d ", U1(p+6));
            fprintf(stderr,"Timeout:%3d ", U1(p+7));
            /* fprintf(stderr,"FlagBits:"); printBits(U1(p+8)); */
            fprintf(stderr,"Timeout:");
            ((U1(p+8) & (1 << 1)) ? putchar('D') : putchar('U'));
            fprintf(stderr," Health:");
            ((U1(p+8) & (1 << 2)) ? putchar('Y') : putchar('N'));
            fprintf(stderr," Correction:");
            ((U1(p+8) & (1 << 3)) ? putchar('Y') : putchar('N'));
            fprintf(stderr," SBAS PRN:");
            ((U1(p+8) & (1 << 3)) ? putchar('D') : putchar('U'));
            fprintf(stderr,"\n");
            status=0; break;
        case 75:		/* MID 75 ACK/NACK/ERROR Notification A0A2|0007|4B|01|9400FA0000|01DA|B0B3 */
            sid=U1(p+5);
            fprintf(stderr,"%s [WARN ] ",time_str(utc2gpst(timeget()),0));
            /* for (i=5;i<raw->len-4;i++) fprintf(stderr,"%02X ",U1(p+i));  51 [NACK ] 01 49 04 FA 00 00 */
            fprintf(stderr,"Check command script in terms of MID %d (=0x%2X)!\n",U1(p+6),U1(p+6));
            status=0;break;
        default: 	    	
            status=decode_sirfgen(raw);
    }
    if (status>0) printMBuf(status);
    return status;
}
/* sync code -----------------------------------------------------------------*/
static int sync_sirf(unsigned char *buff, unsigned char data)
{
    buff[0]=buff[1]; buff[1]=data;
    return buff[0]==SIRFSYNC1&&buff[1]==SIRFSYNC2;
}
/* input SiRF Star raw message from stream -----------------------------------
 * input next SiRF Star raw message from stream
 * args   : raw_t *raw   IO     receiver raw data control struct
 *          unsigned char data I stream data (1 byte)
 * return : status (-1: error message,   0: no message, 1: input observation data,
 *                   2: input ephemeris, 3: input sbas message,
 *                   9: input ion/utc parameter)
 *-----------------------------------------------------------------------------*/
extern int input_sirf(raw_t *raw, unsigned char data)
{
    trace(5,"input_sirf: data=%02x\n",data);

    /* synchronize frame */
    if (raw->nbyte==0) {
        if (!sync_sirf(raw->buff,data)) return 0;
        raw->nbyte=2;
        return 0;
    }
    /* circular_buf_init(auxData[0].cbuf, 10);
    fprintf(stderr,"SIZE=%ld\n",circular_buf_capacity(auxData[0].cbuf)); */

    raw->buff[raw->nbyte++]=data;

    if (raw->nbyte<4) return 0;

    if (raw->nbyte==4) {
        if ((raw->len=U2(raw->buff+2)+8)>SIRFMAXRAWLEN) {
            trace(2,"SiRF length error: len=%d\n",raw->len);
            raw->nbyte=0;
            return -1;
        }
    }
    if (raw->nbyte<4||raw->nbyte<raw->len) return 0;
    raw->nbyte=0;

    /* decode sirf raw message */
    /* writesf(raw->buff); */
    return decode_sirf(raw);
}
/* input superstar 2 raw message from file -------------------------------------
 * input next superstar 2 raw message from file
 * args   : raw_t  *raw   IO     receiver raw data control struct
 *          FILE   *fp    I      file pointer
 * return : status(-2: end of file, -1...9: same as above)
 *-----------------------------------------------------------------------------*/
extern int input_sirff(raw_t *raw, FILE *fp)
{
    int i,data;

    trace(4,"input_sirff:\n");

    /* synchronize frame */
    if (raw->nbyte==0) {
        for (i=0;;i++) {
            if ((data=fgetc(fp))==EOF) return -2;
            if (sync_sirf(raw->buff,(unsigned char)data)) break;
            if (i>=4096) return 0;
        }
    }
    if (fread(raw->buff+2,1,2,fp)<2) return -2;
    raw->nbyte=4;

    if ((raw->len=U2(raw->buff+2)+8)>SIRFMAXRAWLEN) {
        trace(2,"SiRF length error: len=%d\n",raw->len);
        raw->nbyte=0;
        return -1;
    }
    if (fread(raw->buff+4,1,raw->len-4,fp)<(size_t)(raw->len-4)) return -2;
    raw->nbyte=0;

    /* decode SiRF raw message */
    return decode_sirf(raw);
}

