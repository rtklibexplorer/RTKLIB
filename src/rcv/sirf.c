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
*-- Query Firmware Version -----------------------------------------------------*
echo -en "\xA0\xA2\x00\x02\x84\x00\x00\x84\xb0\xb3" > /dev/ttyUSB0
ROM GSD4e_4.1.2-P1 R+ 11/15/2011 319-Nov 15 2011-23:04:55
GSD4e_4.1.2-P1_RPATCH_10.pd2
* -- MID 19 Navigation Parameters
A0A2|0041|1300000014020000000400000401010000000004006420000000000000000000000000000003E8000003E80000000000000001D4C0000075300A00000001000001|04D7|B0B3
*-------------------------------------------------------------------------------*/
#include "rtklib.h"

#define SIRFSYNC1	0xA0    /* SiRF binary message start sync code 1		*/
#define SIRFSYNC2	0xA2    /* SiRF binary message start sync code 2 		*/
#define SIRFSYNCEND1	0xB0    /* SiRF binary message end sync code 1 			*/
#define SIRFSYNCEND2	0xB3    /* SiRF binary message end sync code 2 			*/

#define MID_SRFMTDO	0x04	/* SiRF Measured Tracker Data Out			*/
#define MID_SRFCLOCK	0x07    /* SiRF binary clock status 				*/
#define MID_SRFGEOCLK   0x29    /* SiRF Geodetic Navigation Data			*/
#define MID_SRF50BPS	0x08    /* SiRF binary 50BPS data 				*/ 
#define MID_SRFNLMEAS	0x1c    /* SiRF binary navlib measurement data 			*/
#define MID_SRFALM	0x0e	/* SiRF Almanac Data (Response to Poll) 		*/
#define MID_SRFEPH	0x0f	/* SiRF Ephemeris Data (Response to Poll) 		*/
#define MID_SRF50BPSHQ	0x38    /* SiRF binary 50BPS data 				*/ 
#define MID_SRFSVSTATE  0x1e	/* SiRF Navigation Library SV State Data		*/
#define MID_SRFAUXDATA	0x40	/* SiRF Navigation Library Auxiliary Measurement Data	*/
#define MID_SRFTCXO	0x5d	/* SiRF Temperature Value Output			*/
#define SIRFMAXRAWLEN	2047    /* max length of SiRF binary raw message 		*/

/* double L[NFREQ+NEXOBS]; /* observation data carrier-phase (cycle) */
/* double P[NFREQ+NEXOBS]; /* observation data pseudorange (m) */
/* float  D[NFREQ+NEXOBS]; /* observation data doppler frequency (Hz) */

/* NOTE Empirische Message Sequence 
4 Measured Tracker Data Out
2 Measure Navigation Data Out
9 CPU Throughput
7 Clock Status Data  
2,9,7 Repetiton
93 TCXO Learning Output Response
50 SBAS Parameters 
51 Tracker Load Status Report
65 GPIO State Output
93,50,51,65 Repetition
8 50 BPS Data
28 Navigation Library Measurement Data
64 Navigation Library Messages (Auxiliary Measurement Data)
91 AGC Gain Output
30 Navigation Library SV State Data
41 Geodetic Navigaton Data */

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
    int MeanDRTime;	/* Mean Delta Range Time					*/
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
    long CarrPhase64;	/* Carrier Phase double */
    double CarrFreq64;	/* Carrier Freq */
    signed short CarrAcc;	/* Carrier Acceleration (Doppler Rate) */
    signed short PRNoise;	/* Pseudorange Noise (one sigma, normalized and left-shifted 16 bits */
    signed short dRngQ;		/* Delta Range Quality						*/
    signed short PhLockQ;	/* Phase Lock Quality, normalized and left-shifted 8 bits */
    float QIRatio;
    float elev;		/* elevation angle from MID 4	*/
} auxData_t;

/* Global Variabels ------------------------------------------------------------*/
int VerData = 0;
int lastTOW=-1;
int lastClkB=-1;
int lastClkD=-1;
int Cnt=0;
int InfoCnt=0;
unsigned int MBuf[100];
unsigned int MC=0;
float QImin=100; 
float QImax=0;
double ClkBias=0;			/* MID 7     Clock Status			*/
double ClkBiasTCXO = 96250/10e+9;	/* MID 93.18 calculated clock bias of TCXO 	*/
float altMSL=0.0;			/* MID 41 for tropo correction			*/
int SFcnt[]={0,0,0,0,0};

auxData_t auxData[MAXSAT]; 

/* get fields (big-endian) -----------------------------------------------------*/
#define U1(p) (*((unsigned char *)(p)))
#define I1(p) (*((char *)(p)))

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
    int i;
    unsigned char *q=(unsigned char *)&value+3;
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
    }else {
    q = (unsigned char *)&value+3;
    for (i=0;i<4;i++) *q--=*p++;
    q = (unsigned char *)&value+7;
    for (i=0;i<4;i++) *q--=*p++;
    return value;
    }
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

/* NOTE Print Message sequence */
static void printMBuf(void) {
    unsigned int i,debug=0;
    if (MC>0) {
        if (debug) { 
            printf("MBuf[%2d]=",MC); for (i=0;i<MC;i++) { printf("_%d",MBuf[i]); } 
            printf("\n");
        }
        MC=0;
    }
}

/* checksum ------------------------------------------------------------------*/
static int chksum(const unsigned char *buff, int len)
{
    int i;
    unsigned short sum=0;

    if (len<8) return 0;
    for (i=4;i<len-4;i++) sum=0x7fff&(sum+buff[i]);
    return (sum>>8)==buff[len-4]&&(sum&0xFF)==buff[len-3];
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
    int marray[] = {MID_SRFAUXDATA, 65, MID_SRFCLOCK, MID_SRF50BPS, MID_SRFNLMEAS, MID_SRFALM, MID_SRFEPH, 50, 9, 51, 56, 13, 41, 91, 2, 92, 71,18, 225, 218, 12}; 
    unsigned gsw230=strstr(raw->opt,"-GSW230")!=NULL;
    
    unsigned char *message=raw->buff+4;
    int mid=U1(message);

    /* MID im Array enthalten? -> EXIT */
    for(i=0;i<sizeof(marray)/sizeof(int); i++) if (mid == marray[i] && mid != MID_SRFAUXDATA ) return 0; 
    if (mid != MID_SRFAUXDATA) {
        printf("\nDECODE_OSP:mid=%2d[%4d]:",mid,raw->len);
        for (i=0;i<raw->len;i++) printf("%02X", U1(message-4+i) );
        printf(":"); 
    } 

    switch (mid) {
    case 2:	/* - MID 2, solution data: X, Y, Z, vX, vY, vZ, week, TOW and satellites used */
        for (i=1;i<=5;i++) printf("%02X ", U1(message+i) );
        printf("X:%8d;",S4(message+1));
        printf("Y:%8d;",S4(message+5));
        printf("Z:%8d;",S4(message+9));
        printf("vX:%4hd;",S2(message+13));
        printf("vY:%4hd;",S2(message+15));
        printf("vZ:%4hd;",S2(message+17));
        printf("Mode1:");
        printBits(U1(message+19));
        printf(";HDOP2:%d;",U1(message+20)); 
        printf("Mode2:");
        printBits(U1(message+21));
        printf(";wk:%4hu;",U2(message+22));
        printf("TOW:%6u;",U4(message+24));
        printf("SVs:%2d",U1(message+28));
        break;
    case 4:	/* Measured Tracker Data Out */
        printf("wk:%4hu;",S2(message+1));
        printf("TOW:%6u;",U4(message+3));
        printf("Chans:%d;",U1(message+7));
        printf("\n");
        for(j=0; j<=11; j++) {
            printf("CH#%2d:",j+1);
            printf("SV_id:%2d;",U1(message+8+j*15));
            printf("Azimuth:%3d;",U1(message+9+j*15));
            printf("Elev:%3d;",U1(message+10+j*15));
            printf("State:");
            printBits(U1(message+12+j*15));
            printf(";Valid_Phase:");
            ((U1(message+12+j*15) & (1 << 1)) ? putchar('Y') : putchar('N'));
            printf(";Subframe_Complete:");
            ((U1(message+12+j*15) & (1 << 3)) ? putchar('Y') : putchar('N'));
            printf(";Costas_Lock:");
            ((U1(message+12+j*15) & (1 << 4)) ? putchar('Y') : putchar('N'));
            printf(";Code_Lock:");
            ((U1(message+12+j*15) & (1 << 5)) ? putchar('Y') : putchar('N'));
            printf("\n");
        }
        break;
    case 9:	/* CPU Throughput */
        printf("SegStatMax:%4u;",U2(message+1));
        printf("SegStatLat:%4u;",U2(message+3));
        printf("AveTrkTime:%4u;",U2(message+5));
        printf("LastMilliS:%4u;",U2(message+7));
        break;
    case 11:	/* Command Acknowledgement */
        printf("ACI:%d,SID:%d;",U1(message+2),U1(message+1));
        break;
    case 13:	/* Visible List, update rate: every 2 minutes? */
        vis = U1(message+1);
        printf("VisSV:%d\n",vis);
        for(j=0; j<vis; j++) {
            printf("Ch%02d:SV%2d:",j,U1(message+2+j*5));
            printf("Az:%d,El:%d\n",S2(message+3+j*5),S2(message+5+j*5));
        }
        break;
    case 18:	/* OkToSend? */
        printf("OkToSend:");
        (U1(message+1) == 1) ? putchar('Y') : putchar('N');
        printf("\n");
        break;
    case 30:	/* Navigation Library SV State Data */
        /* The data in MID 30 reports the computed satellite position and velocity at the specified GPS time.
        * Note: *
        When using MID 30 SV position, adjust for difference between GPS Time MID 30 and Time of Transmission
        (see the note in MID 28). Iono delay is not included in pseudorange in MID 28. */
        printf("\nSVid:%2d;",U1(message+1));
        printf("GPSt:%1f;",R8(message+2,gsw230));
        printf("X:%1f;",R8(message+10,gsw230));
        printf("Y:%1f;",R8(message+18,gsw230));
        printf("Z:%1f;",R8(message+26,gsw230));
        printf("vx:%1f;",R8(message+34,gsw230));
        printf("vy:%1f;",R8(message+42,gsw230));
        printf("vz:%1f;",R8(message+50,gsw230));
        printf("ClkBias:%1f;",R8(message+58,gsw230));
        printf("ClkDrift:%f;",R4(message+66));
        printf("EFV:%d;",U1(message+70));
        printf("IonoDly:%f;",R4(message+79));
        break;
    case 31:	/* Navigation Library Initialization Data */
        printf("\nAltMode=%d:AltSrc=%d:Alt=%f:",U1(message+2),U1(message+3),R4(message+4));
        printf("wk=%d:",U2(message+72));
        printf("TimeInitSrc=%d:",U1(message+74));
        printf("Drift=%1f:",R8(message+75,gsw230));
        printf("DriftInitSrc=%d\n",U1(message+83));
        break;
    case 41:	/* Geodetic Navigation Data */
        printf("NValid:%d;",U2(message+1));
        printf("NType:");
        printBits(U1(message+3));
        printBits(U1(message+4));
        printf(";ExtWN:%d;",U2(message+5));
        printf("TOW:%d;",U4(message+7));
        printf("UTC:%4d-%2d-%2d %2d:%2d:%2.1f;",U2(message+11),U1(message+13),U1(message+14),U1(message+15),U1(message+16),(float)U2(message+17)/1000);
        printf("SatList:");
        printBits(U1(message+19));
        printBits(U1(message+20));      
        printBits(U1(message+21));      
        printBits(U1(message+22));      
        printf("Lat:%4d;",S4(message+23));
        printf("Lon:%4d;",S4(message+27));
        printf("AltEps:%4d;",S4(message+31));
        printf("AltMSL:%4d;",S4(message+35));
        printf("Map:%d;",U1(message+39));
        printf("SOG:%d;",U2(message+40));
        printf("COG:%d;",U2(message+42));
        printf("MagV:%d;",S2(message+44));
        printf("Climb:%d;",S2(message+46));
        printf("HeadR:%d;",S2(message+48));
        printf("EHPE:%d;",U4(message+50));
        printf("EVPE:%d;",U4(message+54));
        printf("ETE:%d;",U4(message+58));
        printf("ETVE:%d;",U2(message+62));
        printf("ClkB:%08X->%d;",U4(message+64),U4(message+64));
        printf("ClkBE:%08X->%d;",U4(message+68),U4(message+68));
        printf("ClkDr:%08X->%4d;",U4(message+72),S4(message+72));
        printf("ClkDrE:%08X->%d;",U4(message+76),U4(message+76));
        printf("Dist:%d;",U4(message+80));
        printf("DistE:%d;",U2(message+84));
        printf("HeadE:%d;",U2(message+86));
        printf("NrSVFix:%d;",U1(message+88));
        printf("HDOP:%.1f;",(float)U1(message+89)/5);
        printf("AddModeInf:");
        printBits(U1(message+90));
        break;        
    case 50:	/* SBAS Parameters */
        printf("SBASsv:%3d;", U1(message+1));
        printf("Mode:%3d;", U1(message+2));
        printf("DGPS_Timeout:%3d;", U1(message+3));
        printf("FlagBits:");
        printBits(U1(message+4));
        break;
    case 51:	/* Tracker Load Status Report ... liefert entgegen der Doku nur zwei Byte zurück*/
        printf("SubID:%d;",U1(message+1));
        printf("LoadState:%d;",U4(message+2));
        printf("LoadError:%d;",U4(message+10));
        printf("TimeTag:%d;",U4(message+14));
        break;
    case 56:	/* Extended Ephemeris Data/SGEE Download Output  23,29,2a	 	*/
                /* CGEE = Client Generated Extended Ephemeris - Sattelitte Downlink, 
                   remembers where it was so it knows roughly where it is.		*/
                /* SGEE = Server Generated Extended Ephemeris - TCP/IP-Link ->  DISABLE!*/
        sid=U1(message+1);
        printf("\nMID=56.%d:",sid);
        switch (sid) {
            case 35:	/* SID=35 SGEE Download Initiate - Message		*/
                printf("Start:%d;",U1(message+2));
                printf("Wait_time:%4d;",U4(message+3));
                break;
            case 41:	/* SID=41 SIF Aiding Status   				*/
                printf("SGEE_S:%d;",U1(message+2));
                printf("CGEE_S:%d;",U1(message+3));
                printf("CGEE_P:%4u;",U4(message+4));
                printf("RcvTime:%4d;",U4(message+8));
                break;
            case 42:	/* SID=42 SIF Status Message  				*/
                printf("SIFState:%d;",U1(message+2));
                printf("cgeePredS:%d;",U1(message+3));
                printf("sifAidTyp:%d;",U1(message+4));
                printf("sgeeDlPr:%d;",U1(message+5));
                printf("cgeePrTL:%4d;",U4(message+6));
                printf("cgeePrPM:%4d;",U4(message+10));
                printf("svidCGEEPrP:%d;",U1(message+14));
                printf("sgeeAgeValidy:%d;",U1(message+15));
                break;
        }
        break;
    case 64:	/* Navigation Library Messages	*/
        sid=U1(message+1);
        switch (sid) {
            case 1:	/* SID=1 Auxiliary Initialization Data 	*/
                printf("\nMID=64.%d:",mid);
                printf("uTime=%d:,",U4(message+2));
                printf("wk=%d:",U2(message+6));
                printf("tow=%d:",U4(message+8));
                printf("UHP=%d:",U2(message+12));
                printf("UAP=%d:",U2(message+14));
                printf("sw=%d:",U1(message+16));
                printf("icd=%d:",U1(message+17));
                printf("hwid=%d:",U2(message+18));
                printf("ClkRate=%d:",U4(message+20));
                printf("FrqOff=%d:",U4(message+24));
                printf("Status:");
                ((U1(message+32) & (1 << 0)) ? printf("Bad") : printf("Good"));
                printf("Cache:");
                ((U1(message+32) & (1 << 1)) ? printf("Enabled") : printf("Disabled"));
                printf("RTC:");
                ((U1(message+32) & (1 << 1)) ? printf("Valid") : printf("Invalid"));
                break;
            case 2:	/* SID=2 Auxiliary Measurement Data 	*/
                if (U1(message+4) != 2 ) return 0;
                printf("\nMID#64:SAT=%2d",U1(message+2));
                printf("SVid:%2d;",U1(message+2));
                printf("Status:");
                printBits(U1(message+3));
                printf(";ExtStat:");
                printBits(U1(message+4));
                printf(";BitSyncQ:%d;",U1(message+5));
                printf("TimeTag:%4d;\n",U4(message+6));
                printf("CodePh:%4d;",U4(message+10));
                printf("CarrPh:%4d;",S4(message+14));
                printf("CarrFrq:%4d;",S4(message+18));
                printf("CarrAcc:%4d;",S2(message+22));
                printf("MilliS:%2d;",U2(message+24));
                printf("BitNr:%2d;",U4(message+26));
                printf("CodeCor:%4d;",S4(message+30));
                printf("SmothCd:%4d;",S4(message+34));
                printf("CodeOff:%4d;",S4(message+38));
                printf("PsRN:%2d;\n",S2(message+42));
                printf("DRQ:%2d;",S2(message+44));
                printf("PhLckQ:%2d;",S2(message+46));
                printf("msU:%2d;",S2(message+48));
                printf("SumAbsI:%2d;",U2(message+50));
                printf("SumAbsQ:%2d;",U2(message+52));
                printf("SVBNr:%2d;",S4(message+54));
                printf("MPLOS:%2d;",S2(message+58));
                printf("MPODV:%2d;",S2(message+60));
                printf("RecovS:%d;",U1(message+62));
                printf("SWTU:%4d;",U4(message+63)); 
                
/*                printf("nCarrFrq:%08X->%4d;",U4(message+18),S4(message+18));*/
/*                printf(":CFrq=%10dHz:",S4(message+18));
                printf("CAcc=%7d:",S2(message+22));
                printf("CarrPh=%11dCy:",S4(message+14));
                printf("PsRN=:%6d:",S2(message+42));
                printf("DRQ=%6d:",S2(message+44));
                printf("PhLckQ=%6d:",S2(message+46));
                printf("RecStat=");
                printBits(U1(message+62));
                printf(":SWTU:%3d:",U4(message+63)); 
                printf(":CFR=%4dHz,CFS=%f,",S4(message+18),S4(message+18)*0.000476);   *0.000476*(CLIGHT/FREQL1)); */

                printf(" Frequency[m/s]= (%10.0f)",S4(message+18)*0.000476);	/* Hz		*/
                /*printf("  Phase[cy]= %10ld",S4(message+14)); 	* Cycles	*/
                printf("  Phase[cy]= %10d ",S4(message+14));
                printf("  Status="); /*%d",(U1(message+3)&0x02)>>1); */
                printBits(U1(message+3));
                printf(":ExtStat=");
                printBits(U1(message+4));
                break;
            default:
                printf("\nDECODE_OSP:mid=%2d[%4d.%d]:",mid,sid,raw->len);
                for (i=1;i<raw->len-8;i++) printf("%02X ", U1(message+i) ); 
                break;
        }
        break;
    case 70:	/* Ephemeris Status Response ... seltene Ausgabe? */ 
        sid=U1(message+1);
        printf("SubID:%d;",sid);
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
        printf("SubID:%d;",U1(message+1));
        printf("AGC_Gain:%d;",U1(message+2));
        break;
    case 93:	/* TXCO Learning Output Response */
        w=U1(message+1);
        switch (w) {
            case 1:	/* TCXO Learning Clock Model Data Base */
                printf("\nMID=93.%02d:",w);
                printf("Source:");
                printBits(U1(message+2));
                printf(";AgeRateU:%d;InitOffU:%d;",U1(message+3),U1(message+4));
                printf("ClkDrift:%f;",R4(message+6));
                printf("TempU:%d;",U2(message+10));
                printf("ManWkN:%d;",U2(message+12));
                break;
            case 2: 	/* TCXO Learning Temperature Table  ppb, TBD: Signed statt unsigned! */
                printf("\nMID=93.%02d:",w);
                printf("FreqOffset:%4d;",S2(message+6));
                printf("GlobalMin:%4d;",S2(message+8));
                printf("GlobalMax:%4d;",S2(message+10));
                break;                
            case 18:	/* Temperature Voltage Output */
                printf("\nMID=93.%02d:",w);
                /* for (i=1;i<raw->len-8;i++) printf("%02X ", U1(message+i) ); */
                printf("TOW:%6u;",U4(message+2));
                printf("wk:%4hu;",U2(message+6));
                printf("TStatus:");
                printBits(U1(message+8));
                printf(";ClkOff:%d;",U1(message+9));
                printf("ClkDrift:%d;",U4(message+10));
                printf("ClkDriftU:%d;",U4(message+14));
                printf("ClkBias:%f;",U4(message+18)/1.0e9);
                ClkBiasTCXO=U4(message+18)/1.0e9;
                printf("Temperature:%.1f;",(float)(140*U1(message+22)/255)-40);
                break;
            default:
                printf("\nMID=93.%02d:",w);
        }
        break;
    case 225:	/* Data Log */
        printf("SubID:%d;",U1(message+1));    
        for (i=1;i<raw->len-8;i++) printf("%02X ", U1(message+i) ); 
        
    default:
        /* for (i=1;i<raw->len-8;i++) printf("%02X ", U1(message+i) );  */
        printf("\n");
    }
    return 0;
}

/* NOTE DECODE MID 4 Measured Tracker Data Out ---------------------------------- */
static int decode_sirfmtdo(raw_t *raw)
{
    unsigned debug=0;					/* NOTE Set to 1 for debug output	*/
    unsigned int i,j,ch,sat;
    unsigned char *p;

    p=raw->buff+4; ch=U1(p+1);
    if (ch>=MAXOBS) {
        trace(2,"SiRF mid#4 wrong channel: ch=%d\n",ch);
        return -1;
    }
    for(j=0; j<=11; j++) { 				/* NOTE Save Tracking status for further processing */
        sat=satno(SYS_GPS,U1(p+8+j*15)); 	        
        if (sat) {
            auxData[sat].trackStat=U1(p+12+j*15);
            auxData[sat].elev=(float)U1(p+10+j*15)/2;	/* Save Elevation for Tropo correction	*/
            }
    }

    if (debug) {					/* NOTE For Debugging purpose only 	*/
        printf("\nDECODE_OSP:mid=%2d[%4d]:",U1(p),raw->len);
        for (i=0;i<raw->len;i++) printf("%02X", U1(p-4+i) );
        printf(":"); 

        for(j=0; j<=11; j++) {
            if (j==0) printf("\nMID 4 Debug Output\n------------------\nwk:%4hu;TOW:%6u;Chans:%d\n",S2(p+1),U4(p+3),U1(p+7));
            printf("CH#%2d:",j+1);
            printf("SV_id:%2d;",U1(p+8+j*15));
            printf("Azimuth:%3d;",U1(p+9+j*15));
            printf("Elev:%3d;",U1(p+10+j*15));
            printf("State:%2x:",U1(p+12+j*15));
            printBits(U1(p+12+j*15));
            printf(";Valid_Phase:");
            ((U1(p+12+j*15) & (1 << 1)) ? putchar('Y') : putchar('N'));
            printf(";Subframe_Complete:");
            ((U1(p+12+j*15) & (1 << 3)) ? putchar('Y') : putchar('N'));
            printf(";Costas_Lock:");
            ((U1(p+12+j*15) & (1 << 4)) ? putchar('Y') : putchar('N'));
            printf(";Code_Lock:");
            ((U1(p+12+j*15) & (1 << 5)) ? putchar('Y') : putchar('N'));
            printf(";Ephemeris available:");
            ((U1(p+12+j*15) & (1 << 7)) ? putchar('Y') : putchar('N'));
            printf("\n");
            }
    }
    return 0;
}    

/* NOTE DECODE Generic Clock Data (from MID 7, add MID 41, too?)
        - U-Blox RXM-RAWX Carrier phase locktime counter (maximum 64500ms)			        
   NOTE GPS Clock Frequency  MID 93.18 provides Clock Drift Uncertainty 
        printf("\nGPS Clock:%f kHz, RepBias:%8.3f µs, ",(FREQL1+drift)*16/1540/1e+3,U4(p+12)/1e+3);
        printf("CalcBias = %8.3f µs, ",1e6*drift/FREQL1);  printf("DeltaTime = %.3f s, ",tow-U4(p+16)/1000);  */
static int decode_clock(raw_t *raw,double rcBias, double drift,unsigned week,double tow)
{
    const double EGC=3.986005E+14; 
    const double F=-2*sqrt(EGC)/CLIGHT/CLIGHT;	/* Earth's universal gravitational parameters 	*/
    unsigned char *p;
    obsd_t *src,*dst;
    int i,j,n=0,mid,cs=0;
    double bias,Padj,Ladj,tt,tgd=0,dt,tsv,td,tTD=0,tID=0;
    gtime_t time, time0={0},toc,ts,time1;
    
    p=raw->buff+4; mid=U1(p); time=gpst2time(week,tow); 			/* printf("\nM7:time.sec:%f",time.sec); */
    /*time=gpst2time(week,(float)U4(p+16)/1000);				/* SiRF   Estimated GPS time is the time estimated when the measurements were made. */
    time=timeadd(time,-rcBias); /*printf("- bias %f = %f\n",rcBias,time);	/* U-Blox RXM-RAW uses Measurement time of week in receiver
                                                                                          local time approximately aligned to the GPS time system. */
  
    /* NOTE MAIN PROCESSING LOOP */
/*    if (MAXOBS < 4) return 0; */
    for (i=0;i<raw->obuf.n&&i<MAXOBS;i++) {
        src=&raw->obuf.data[i];
        if (!satsys(src->sat,NULL)) continue;				/* Satellit gehört zum L1-System? 	printf("DE=%f ",fabs(tow+RawBias-src->time.sec)); */
        if (fabs(tow+rcBias-src->time.sec)>0.01) continue;
        dst=&raw->obs.data[n++]; *dst=*src;				/* referenzierte Kopie des Originals		*/
        raw->obs.data[n].sat =dst->sat; raw->obs.data[n].time=time;	/* Copy Sat PRN Code + Time			*/
        dst->time=gpst2time(week,src->time.sec-rcBias);			/* Subtract Clock Bias to get time of receipt 	*/
        raw->time=dst->time;
        
        /* NOTE Approximation of atmospheric delay according to Spilker (1994)	*/
        if (auxData[dst->sat].elev>0) { 
            tTD=2.44*1.0121*exp(-0.000133*altMSL)/(0.121+sin(auxData[dst->sat].elev*PI/180));
        } else { tTD=0; }	/* printf("\nSAT %d: Tropo Delay: %.1fm\n",dst->sat,tTD); */

/*        if (auxData[dst->sat].iono>0) { 
            tID=(40.3*1E+16)/FREQL1/FREQL1*auxData[dst->sat].iono;
            printf("\nSAT %d: Iono Delay: %.1f %d",dst->sat,tID*CLIGHT,auxData[dst->sat].iono);
            }  /*rintf("IONO=%d,%d,%.6fkm",auxData[dst->sat].iono,FREQL1,(float)auxData[dst->sat].iono/FREQL1/FREQL1); }*/
        
        /* Time of receipt, time of transmission and resulting pseudo range correction */
        /* NOTE SV Clock Drift Correction still missing!  printf("\n[%d] sat=%2d RcvClkBi=%.6fs ",n,dst->sat,bias); */
        /*      GPS Software Time – Clock Bias = Time of Receipt = GPS Time. 
                GPS Software Time – Pseudorange (sec) = Time of Transmission = GPS Time. */

        /* NOTE MID 28 Adjust Carrier Frequency [Hz]
                Corrected Carrier Frequency (m/s) = Reported Carrier Frequency (m/s) – Clock Drift (Hz)*C / 1575420000 Hz	*/
        if (auxData[dst->sat].trackStat & (1 << 4)) dst->D[0]-=drift; 
        dst->D[0]*=-1;								/* Mandatory change of sign proved by comparison with u-Blox dataset		*/

        Padj=tTD/CLIGHT+rcBias; 
        if (auxData[dst->sat].trackStat & (1 << 1)) { 				/* If LOCKED, adjust carrier phase by receiver clock bias			*/
            if (dst->sat==raw->nav.eph[dst->sat-1].sat) { 			/* If AVAILABLE, apply group delay correcetion (acc. IS-GPS-200K 20.3.3.3.2)	*/
                Padj+=0*raw->nav.eph[dst->sat-1].tgd[0] * (1-sqrt(FREQL1)/(sqrt(FREQL1)-sqrt(FREQL2))); } 	
                                                                                /* printf("\nSV %2d TGD -%3.3fm\n",dst->sat,raw->nav.eph[dst->sat-1].tgd[0]*CLIGHT); } 	*/
            /* Standard Deviation of pseudo range measurement */
            dst->qualP[0]=(float)15*(auxData[dst->sat].QIRatio-QImin)/(QImax-QImin);	/* MID 64.2 Auto-Learning Quality Level of Carrier Phase measurement		*/
        } /* else { dst->L[0]=0; }							/* Mark Carrier phase measeurement as invalid 	*/
        Padj*=CLIGHT; 
/*        if (auxData[dst->sat].iono>0) { Padj+=(float)auxData[dst->sat].iono/FREQL1/FREQL1;
        /*rintf("IONO=%d,%d,%.6fkm",auxData[dst->sat].iono,FREQL1,(float)auxData[dst->sat].iono/FREQL1/FREQL1); }*/
        dst->P[0]-=Padj;						/* Pseudo range [m] adjustment *Ladj*=FREQL1;  	*/

        /* NOTE CARRIER PHASE dst->L[0] [cycles] -> Mid 64 outputs L1 Cycles, M28 reports Meters			        */
        /*if (!dst->LLI[0]&&auxData[dst->sat].TimeTrack<30000) dst->L[0]=0;	/* SAT <30 Sekunden aktiv?			*/
        if (dst->L[0]) { 							/* LLI: bit1=slip, bit2=half-cycle-invalid 	*/
            /* if (!((auxData[dst->sat].trackStat&0x06)>>1)) dst->L[0]=0; 		/* MID  4: Carrier Phase valid?			*/
            /* if ( ((auxData[dst->sat].extStat&0x3c)>>2  )) { dst->LLI[0]=1; cs++;}	/* MID 64: Cycle Slip, Subframe sync, Multipath */
            /* if ( ((auxData[dst->sat].RecovStat&0x14)>>2)) dst->LLI[0]=1;	/* MID 64: Bad PrePos, wrong BitSync	 	*/
            
            /* NOTE invalid carrier phase measurements are flagged in RTKLIB by setting the carrier phase value to zero. */
            if (dst->LLI[0]) dst->L[0]=0;
            if (dst->sat==raw->nav.eph[dst->sat-1].sat) { 			/* If AVAILABLE, apply group delay correcetion (acc. IS-GPS-200K 20.3.3.3.2)	*/
                tgd=raw->nav.eph[dst->sat-1].tgd[0] * (1-sqrt(FREQL1)/(sqrt(FREQL1)-sqrt(FREQL2))); 
                } else { tgd=0; }
            Ladj=(0*tgd+rcBias+tTD/CLIGHT)*FREQL1; if (dst->L[0]) dst->L[0]-=Ladj;		 	/* Carrier Phase [Cycles] adjustment		*/ 
            dst->qualL[0]=7-(dst->L[0] ? 1 : 0 );				/* MID 2 PMode Phase Lock = 7			*/
        } 
/* NOTE        printf("\nSAT %2d P=%.0fkm pAdj=%.6fs L=%.fk LLI=%d Q=%d %.3f %.3f",dst->sat,dst->P[0]/1000,Padj/CLIGHT,dst->L[0]/1000,dst->LLI[0],dst->qualP[0],auxData[dst->sat].QIRatio,(QImax-QImin));*/
    }
    if (strstr(raw->opt,"-INVCP")) { raw->obs.data[n].L[0]*=-1; } 		/* Phase polarity flip option (-INVCP) */        
    raw->obs.n=n;  
    
    /* clear observation data buffer */
    for (i=0;i<MAXOBS;i++) {
        raw->obuf.data[i].time=time0;
        for (j=0;j<NFREQ+NEXOBS;j++) {
            raw->obuf.data[i].L[j]=raw->obuf.data[i].P[j]=0.0;
            raw->obuf.data[i].D[j]=0.0;
            raw->obuf.data[i].SNR[j]=raw->obuf.data[i].LLI[j]=0;
            raw->obuf.data[i].code[j]=CODE_NONE;
            raw->obuf.data[i].qualL[j]=raw->obuf.data[i].qualP[j]=0;
        }
    }
    if (n>1) printMBuf();
    return n>0?1:0;
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
     resolution. */
static int decode_sirfclock(raw_t *raw)
{
    double bias,tow;
    unsigned drift,wn;
    unsigned char *p;

    trace(4,"decode_sirfclock: len=%d\n",raw->len);

    p=raw->buff+4;
    if (raw->len!=28) {
        trace(2,"SiRF mid#7 length error: len=%d\n",raw->len);
        return -1;
    }

    wn=U2(p+1); tow=U4(p+3)/100;/* Seconds into the current week, accounting for clock bias, when the current measurement was
                                   made. This is the true GPS time of the solution. */

    drift=U4(p+8);		/* Clock Drift in CSR receivers is directly related to the frequency of the GPS clock, 
                                   derived from the GPS crystal. Clock drift can be used as a Doppler bias for Carrier
                                   Frequency calculation, reported in Message ID 28. 
                                   NOTE Clock drift is only reported to a truncated 1 Hz resolution! */
    bias=U4(p+12)/1e+9;		/* TBD: In einem 125er Array ablegen und rsd rechnen? */
    
    return decode_clock(raw,bias,drift,wn,tow); /* printf("\nMID 7 tow=%.3f, est=%.3f, bias=%f ",tow,(float)U4(p+16)/1000,bias); */
}

/* NOTE DECODE MID 41 Geodetic Navigation Data */
static int decode_sirfgeoclk(raw_t *raw)
{
    double bias,tow;
    unsigned drift,wn;
    unsigned char *p; 
    int i;
    trace(4,"decode_sirfgeoclk: len=%d\n",raw->len);

    p=raw->buff+4;
    if (raw->len!=99) {
        trace(2,"SiRF mid#41 length error: len=%d\n",raw->len);
        return -1;
    }

    wn=U2(p+5); tow=U4(p+7)/1000; 
    bias=U4(p+64)/CLIGHT/100;
    drift=S4(p+72)*FREQL1/CLIGHT/100; 
    altMSL=(float)S4(p+35)/100; 
    
/*    fprintf(stderr,"%s [INFO ] GPS N:%.4f° E:%.4f° H%.2fm HDOP:%.1fm Q:%d:%d\n",time_str(utc2gpst(timeget()),0),(float)S4(p+23)/1e+7,(float)S4(p+27)/1e+7,(float)S4(p+31)/100,(float)U1(p+89)/5,U1(p+4)&0x0f,U1(p+88)); */
    if (InfoCnt==2) {
        fprintf(stderr,"%4d/%02d/%02d %02d:%02d:%02.0f [INFO ] GPS N:%.4f° E:%.4f° H%.2fm HDOP:%.1fm Q:%d:%d ",U2(p+11),U1(p+13),U1(p+14),U1(p+15),U1(p+16),(float)U2(p+17)/1000,(float)S4(p+23)/1e+7,(float)S4(p+27)/1e+7,(float)S4(p+31)/100,(float)U1(p+89)/5,U1(p+4)&0x0f,U1(p+88));
        for (i=0;i<5;i++) printf("SF%d:%X ",i+1,SFcnt[i]);
        printf("\n");
    }
/*    fprintf(stderr,"%s [INFO ] UTC %4d-%02d-%02d %02d:%02d:%02.1f\n",time_str(utc2gpst(timeget()),0),U2(p+11),U1(p+13),U1(p+14),U1(p+15),U1(p+16),(float)U2(p+17)/1000); */
/*    printf("\nMID41 tow=%f, est=%f, bias=%f ",tow,tow,bias);*/
    return 0;

    return decode_clock(raw,bias,drift,wn,tow);

        printf("NValid:%d;",U2(p+1));
        printf("NType:");
        printBits(U1(p+3));
        printBits(U1(p+4));
        printf(";ExtWN:%d;",U2(p+5));
        printf("TOW:%d;",U4(p+7));
        printf("UTC:%4d-%2d-%2d %2d:%2d:%2.1f;",U2(p+11),U1(p+13),U1(p+14),U1(p+15),U1(p+16),(float)U2(p+17)/1000);
        printf("SatList:");
        printBits(U1(p+19));
        printBits(U1(p+20));      
        printBits(U1(p+21));      
        printBits(U1(p+22));      
        printf("Lat:%4d;",S4(p+23));
        printf("Lon:%4d;",S4(p+27));
        printf("AltEps:%4d;",S4(p+31));
        printf("AltMSL:%4d;",S4(p+35));
        printf("Map:%d;",U1(p+39));
        printf("SOG:%d;",U2(p+40));
        printf("COG:%d;",U2(p+42));
        printf("MagV:%d;",S2(p+44));
        printf("Climb:%d;",S2(p+46));
        printf("HeadR:%d;",S2(p+48));
        printf("EHPE:%d;",U4(p+50));
        printf("EVPE:%d;",U4(p+54));
        printf("ETE:%d;",U4(p+58));
        printf("ETVE:%d;",U2(p+62));
        printf("ClkB:%08X->%d;",U4(p+64),U4(p+64));
        printf("ClkBE:%08X->%d;",U4(p+68),U4(p+68));
        printf("ClkDr:%08X->%4d;",U4(p+72),S4(p+72));
        printf("ClkDrE:%08X->%d;",U4(p+76),U4(p+76));
        printf("Dist:%d;",U4(p+80));
        printf("DistE:%d;",U2(p+84));
        printf("HeadE:%d;",U2(p+86));
        printf("NrSVFix:%d;",U1(p+88));
        printf("HDOP:%.1f;",(float)U1(p+89)/5);
        printf("AddModeInf:");
        printBits(U1(p+90));
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
static int decode_sirfnlmeas(raw_t *raw)
{
    int i; unsigned ch,flags,pherrcnt,cm=0;
    unsigned char *p;
    unsigned gsw230=strstr(raw->opt,"-GSW230")!=NULL;
    obsd_t *obsd;
    
    trace(4,"decode_sirfnlmeas: len=%d\n",raw->len);
    if (raw->len!=64)	{ trace(2,"SiRF mid#28 length error: len=%d\n",raw->len); return -1; }
    p=raw->buff+4; ch=U1(p+1);
    if (ch>=MAXOBS)	{ trace(2,"SiRF mid#28 wrong channel: ch=%d\n",ch); return -1; }
    
    obsd=&raw->obuf.data[ch];
    if (ch>=raw->obuf.n) raw->obuf.n=ch+1;

    /* Fill Observation Data Structure */
    obsd->sat=satno(SYS_GPS,U1(p+6)); obsd->code[0]=CODE_L1C;
    obsd->time.sec=R8(p+7,gsw230); 
    auxData[obsd->sat].dtime=R8(p+7,gsw230);				/* GPS Software Time  printf("\nMID 28: GPS TIME:%.3f",obsd->time.sec); */

    /* NOTE The reported Time of Measurement, Pseudorange and Carrier Phase are all uncorrected values	*/
    obsd->P[0]=R8(p+15,gsw230);						/* Pseudorange, without corrections [m], smoothed by carrier phase!	*/
    obsd->D[0]=R4(p+23)*FREQL1/CLIGHT;					/* Carrier Frequency [m/s] -> Hz (Doppler Extract erst mit GPS Clock!)	*/
    obsd->L[0]=R8(p+27,gsw230)*FREQL1/CLIGHT;  				/* Carrier Phase [m], report as Cycles					*/
    /* printf("MID 28 sat=%d P=%08x%08x=%.0f D=%08x=%.0f L=%08x%08x=%.0f\n",obsd->sat,U4(p+15),U4(p+19),R8(p+15,gsw230),U4(p+23),R4(p+23)*FREQL1/CLIGHT,U4(p+27),U4(p+31),R8(p+27,gsw230)*FREQL1/CLIGHT);*/
    
    /* Calculate Signal to Noise ratio */
    for (i=120;i>0;i--) auxData[obsd->sat].cn[i+10]=auxData[obsd->sat].cn[i];
    obsd->SNR[0]=U1(p+38); auxData[obsd->sat].cn[1]=U1(p+38);
    for (i=39;i<=47;i++) {						/* Kleinstes Signalrauschverhältnis ermitteln und abspeichern		*/
        auxData[obsd->sat].cn[i-37]=U1(p+i);
        if (U1(p+i)<obsd->SNR[0]) obsd->SNR[0]=U1(p+i);
    }
    for (i=1;i<130;i++) cm+=auxData[obsd->sat].cn[i];
    obsd->SNR[0]=cm/130; obsd->SNR[0]*=4.0;

    /* Constistent code epoch alignment, phase errors > 60 degrees prec. second */
    flags=U1(p+37); pherrcnt=U1(p+54);					/* printf("\nsat=%d,PhaseQ=%d %d ",obsd->sat,pherrcnt,U1(p+54)); printBits(flags&0x02); */
    obsd->LLI[0]=flags&0x02&&pherrcnt<50?0:1; 				/* printf(" LLI=%d",obsd->LLI[0]); printf("\n"); */

    /* Fill Auxiliary Data Structure */
    auxData[obsd->sat].time.sec=obsd->time.sec;
    auxData[obsd->sat].CarrPhase28=obsd->L[0];    
    auxData[obsd->sat].CarrFreq28=obsd->D[0];
    auxData[obsd->sat].PseudoR=obsd->P[0];
    auxData[obsd->sat].TimeTrack=U2(p+35);				/* printf("TT=%d %04x %d",auxData[obsd->sat].TimeTrack,U2(p+35),U2(p+35));*/
    auxData[obsd->sat].SyncFlag=U1(p+37);
    auxData[obsd->sat].cno=obsd->SNR[0]; 
    auxData[obsd->sat].DeltaRange=U2(p+48);
    auxData[obsd->sat].MeanDRTime=U2(p+50);
    auxData[obsd->sat].ExtraTime=S2(p+52);
    auxData[obsd->sat].PhErrCnt=U1(p+54);
    
    return 0;
}


/* NOTE DECODE MID 30 Navigation Library SV State Data
   NOTE The data in MID 30 reports the computed satellite position and velocity at the specified GPS time!
   NOTE When using MID 30 SV position, adjust for difference between GPS TIME MID 30 and
        Time of Transmission (see note in MID 28), Iono dely is not included in pseudorange in MID 28      
        - Single precision floating point Conversion ... Datentyp in Manual falsch angegeben!	*/
static int decode_sirfsvstate(raw_t *raw)
{
    int sat,ch;
    double pr,dt,BiasCorr,trx,ts,toc;
    signed clkdrift;
    unsigned char *p;
    unsigned gsw230=strstr(raw->opt,"-GSW230")!=NULL;
    obsd_t *obsd;

/* NOTE GPS Software Time – Pseudorange (sec) = Time of Transmission = GPS Time. 
        Adjust SV position in MID 30 by (GPS Time MID 30 – Time of Transmission) * Vsat. */

    trace(4,"decode_sirfnlmeas: len=%d\n",raw->len); 
    if (raw->len!=91) {
        trace(2,"SiRF mid#30 length error: len=%d\n",raw->len);
        printf("SiRF mid#30 length error: len=%d\n",raw->len);
        return -1;
    } 

    p=raw->buff+4; sat=satno(SYS_GPS,U1(p+1)); /* printf("MID 30 sat=%d->obs[%d]?",sat,MAXOBS); */
    for(ch=0; ch<=MAXOBS; ch++) { if (raw->obs.data[ch].sat==sat) break; }	/* printf("(SAT=%d)",ch,sat); */
    /* NOTE Satellite Clock Error and Orbital Solution Error Estimation for Precise Navigation Applications
            https://www.researchgate.net/publication/272877529_Satellite_Clock_Error_and_Orbital_Solution_Error_Estimation_for_Precise_Navigation_Applications */
    if (ch<=MAXOBS) {
        obsd=&raw->obuf.data[ch];		/* NOTE RICHTIG? */
/*        printf("\nSAT=%d,P=%.0fkm",obsd->sat,obsd->P[0]/1000); */
        /* NOTE GPS Time = GPS Software Time – Pseudorange (sec) = Time of Transmission 	*/
        trx = obsd->time.sec;	/* printf("\nGPS Software Time (OBS) = %6.3fs",trx);*/
        pr = obsd->P[0];	/* printf("\nPseudo range (%.0fkm)  =      %.3fs",pr/1000,pr/CLIGHT); */
        ts = trx-pr/CLIGHT;	/* printf("\nTime of Transmission    = %6.3fs",ts); */
        /* NOTE Adjust SV position in MID 30 by (GPS Time MID 30 – Time of Transmission) * Vsat	*/
        toc = R8(p+2,gsw230); 	/* printf("\nGPS Time (MID30)        = %6.3fs",toc);*/
        dt = ts-toc;		/* printf("\nDelta                   =      %.3fs",dt);*/
                                /* printf("\nClock Bias		=     %.8fs",R8(p+58,gsw230));	/* sec */
        clkdrift = U4(p+66);    /* printf("\nClock Drift  (%08x) =     %us/s",clkdrift,U4(p+66));	/* s/s Datentyp = DEZIMAL, nicht Floating-Point!  */
                                /*printf("\nClkDr2 =%g",R4(p+66));
                                printf("\nvx=%f, vy=%f, vz=%f",R8(p+34,gsw230),R8(p+42,gsw230),R8(p+50,gsw230));
                                printf("\nPseudorange             =   %.3fkm",pr/1000);
                                printf("\nIono Delay   (%08x) =   %.3fkm",U4(p+79),(float)U4(p+79)/1000); /* Datentyp = DEZIMAL, nicht Floating-Point! */
        BiasCorr=(ClkBias-R8(p+58,gsw230))*CLIGHT; 
                                /* printf("\nBias Correction (%.1fms) = %.3fkm",BiasCorr/CLIGHT*1000,BiasCorr/1000); */
                                /* printf("\nesc = %f =",(float)R8(p+58,gsw230)+0*R4(p+66)*dt); */
/*                                printf("%fkm",(float)
                                (R8(p+58,gsw230)+0*R4(p+66)*dt)*CLIGHT/1000);
                                printf("\nR1 (%08x) = %d",U4(p+71),U4(p+71));
                                printf("\nR2 (%08x) = %d",U4(p+75),U4(p+75)); */

/* ++     obsd->time.sec=(float)ts-BiasAdj*R8(p+58,gsw230); */
/*        obsd->P[0]-=(float)0*clkdrift*CLIGHT/FREQL1;	/* Adjust Pseudo Range [m] with drift */
/* ++     obsd->P[0]-=(float)BiasAdj*R8(p+58,gsw230)*CLIGHT;	/* Adjust Pseudo Range with Bias 	*/
/*++        printf("\nSAT=%d, Bias=%.8fs, P=%.0fkm, Padj=%.0fkm, Ladj=%.0fCy ",obsd->sat,R8(p+58,gsw230),obsd->P[0]/1000,(float)BiasAdj*R8(p+58,gsw230)*CLIGHT/1000,BiasAdj*FREQL1*(float)R8(p+58,gsw230));

        /* Carrier Frequncy [Hz]  ebenfalls anpassen??? */
/* ++        obsd->L[0]-=(float)BiasAdj*R8(p+58,gsw230)*FREQL1; */
/*++        printf("P/L ratio =%.2fppm",(1-(obsd->P[0]*FREQL1/CLIGHT/obsd->L[0]))*1E+6);*/
    }

/*    printf("ClkBias:%08X%08X->%gsec -> ",U4(p+58),U4(p+62),R8(p+58,gsw230)); 
    printf("%f\n",R8(p+2,gsw230)-R8(p+58,gsw230));
*/
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
/*    printf("\n!!SAT#%dIono=%f,%d",sat,R4(p+79),S4(p+79)); 
/*    printf("\n!!ClkDrift: %08X    ->%gs/s",U4(p+66),R4(p+66));*/
/*    printf("\n!!ClkBias:%08X%08X->%gsec",U4(p+58),U4(p+62),R8(p+58,gsw230)); */
    
/* NOTE	When using MID 30 SV position, adjust for difference between GPS Time MID 30 and Time of Transmission
        (see the note in MID 28). Iono delay is not included in pseudorange in MID 28. 				*/
        return 0;
        printf("\nSVid:%2d;",U1(p+1));
        printf("GPSt:%1f;",R8(p+2,gsw230));
        printf("X:%1f;",R8(p+10,gsw230));
        printf("Y:%1f;",R8(p+18,gsw230));
        printf("Z:%1f;",R8(p+26,gsw230));
        printf("vx:%1f;",R8(p+34,gsw230));
        printf("vy:%1f;",R8(p+42,gsw230));
        printf("vz:%1f;",R8(p+50,gsw230));
        printf("ClkBias:%1f;",R8(p+58,gsw230));
        printf("ClkDrift:%f;",R4(p+66));
        printf("EFV:%d;",U1(p+70));
        printf("IonoDly:%f;",R4(p+79));

    return 0;	/* NOTE Gibt nichts zurück ... keine Aktion / Verwertung ... */
}
/* NOTE DECODE MID 64 Navigation Library (NL) Auxiliary Measurement Data	
        Added Self-Learing QI-Range	*/
static int decode_sirfauxdata(raw_t *raw)
{
    int sat;
    unsigned ch;
    unsigned char *p;
    float QI;
    printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
    p=raw->buff+4; if (U1(p+1)!=2) decode_sirfgen(raw);
    if (raw->len!=75) {
        trace(2,"SiRF mid#64 length error: len=%d\n",raw->len);
        printf("SiRF mid#64 length error: len=%d\n",raw->len);
        return -1;
    } 

    ch=U1(p+2); sat=satno(SYS_GPS,U1(p+2));
    if (ch>=MAXOBS) {
        trace(2,"SiRF mid#64 wrong channel: ch=%d\n",ch);
        printf("SiRF mid#64 wrong channel: ch=%d\n",ch);
        return -1;
    }
    auxData[sat].extStat=U1(p+4);
    auxData[sat].RecovStat=U1(p+62);		/*     printf("XXXXRecov=%02X,SWU=%d",U1(p+62),U4(p+63));*/
    auxData[sat].TimeTag64=U4(p+6);
    auxData[sat].CarrPhase64=U4(p+14);		/* S4? printf("\nZZZCP SAT=%2d RAW=%08X->U:%d -S:%d",sat,U4(p+14),S4(p+14));  printf("-> %d !!",U4(p+14)); */
                                                /*     printf("JJJ auxData=%f",auxData[sat].CarrPhase64); */
    auxData[sat].CarrFreq64=S4(p+18)*0.000476;
    auxData[sat].CarrAcc=S2(p+22); 
    auxData[sat].PRNoise=S2(p+42); 
    auxData[sat].dRngQ=S2(p+44);
    auxData[sat].PhLockQ=S2(p+46);		/*     printf("XXXX %04X->%d",U2(p+46),S2(p+46)); */
    auxData[sat].QIRatio=(float)U2(p+52)/U2(p+50);
    QI=(float)U2(p+52)/U2(p+50);
    if (QI < QImin) QImin=QI;
    if (QI > QImax) QImax=QI;
    printf("\nQRQR Qmin=%f Qmax=%f QR=%f",QImin,QImax,(float)U2(p+52)/U2(p+50));
   return 0;
}
static int decode_ephem(int sat, raw_t *raw)
{
    eph_t eph={0};
    int k,j,x1,x2,x3;

    x1=decode_frame(raw->subfrm[sat-1]   ,&eph,NULL,NULL,NULL,NULL);
    x2=decode_frame(raw->subfrm[sat-1]+30,&eph,NULL,NULL,NULL,NULL);
    x3=decode_frame(raw->subfrm[sat-1]+60,&eph,NULL,NULL,NULL,NULL);
    fprintf(stderr,"%s [INFO ] Ephemeris Decoding Status: %d%d%d ",time_str(utc2gpst(timeget()),0),x1,x2,x3);
    /* printf("IODE-Chk:%02X-%02X",U1(raw->subfrm[sat-1]+30+48/8),U1(raw->subfrm[sat-1]+60+216/8)); */
    if (!x3) printf("IODE mismatch: %02X/%02X",U1(raw->subfrm[sat-1]+30+48/8),U1(raw->subfrm[sat-1]+60+216/8));
    printf("\n");
    
    if (decode_frame(raw->subfrm[sat-1]   ,&eph,NULL,NULL,NULL,NULL)!=1||
            decode_frame(raw->subfrm[sat-1]+30,&eph,NULL,NULL,NULL,NULL)!=2||
            decode_frame(raw->subfrm[sat-1]+60,&eph,NULL,NULL,NULL,NULL)!=3) return 0;

    if (!strstr(raw->opt,"-EPHALL")) {
        if (eph.iode==raw->nav.eph[sat-1].iode) return 0; /* unchanged */
    }
    eph.sat=sat;
    raw->nav.eph[sat-1]=eph;
    raw->ephsat=sat;

    printMBuf();
    return 2;
}

    /* for (j=1;j<3;j++) {
        printf("\nDECODE EPH-SF%d:",j+1); for (k=0;k<30;k++) printf("%02X",U1(raw->subfrm[sat-1]+j*30+k)); 
        id2=getbitu(raw->subfrm[sat-1]+j*30,43,3); /* subframe id */
/*        printf("|%d|",id2);
    }
    printf("IODE-Chk:%02X-%02X",U1(raw->subfrm[sat-1]+30+48/8),U1(raw->subfrm[sat-1]+60+216/8)); */



/* NOTE DECODE MID 8 50BPS message (MID 8 & MID 56.5)
   NOTE decode_word is only used by sirf.c, not by ublox.c
        1000101100000000110100001010XX SiRF   -> padded trailing bits 
        XX1000101100000000110100001010 u-Blox -> padded front bits			
Chris Kuethe "Message 8 is generated as the data is received. It is not buffered
             on the chip. So when you enable message 8, you'll get one subframe
             every 6 seconds. Of the data received, the almanac and ephemeris are
             buffered and stored, so you can query them at will. Alas, the time
             parameters are not stored, which is really lame, as the UTC-GPS
             correction changes 1 second every few years. Maybe."		
SF#1:22C035 7BD1A5 EB4872 8E4C30 5F5680 567CC9 43B870 8BA625 C0CBDB 3BFFF5 
SF#1:22C035 7AAFC6 035400 8B61D6 3AD6D0 0A79F6 330EB9 08C465 FFFFE8 B5F06E */
static int decode_sirf50bps(raw_t *raw)
{
    int debug=0,mid,prn,sat,id,word,i,DataID,PageID,tlm,tow,SFID,carry,j,id2,week,toa,x4,x5;
    unsigned int words[10];
    unsigned char *p=raw->buff+4; 	/* strip start sequence and payload	*/
    unsigned char subfrm[30];
    raw->len=(raw->len-4-4);		/* strip checksum and end sequence 	*/

    trace(4,"decode_sirf50bps: len=%d\n",raw->len);
    mid=U1(p);prn=U1(p+2); /* if (mid!=81) {	/* NOTE MID #?				*/
    /*        printf("\nMID:%2d",mid); printf("->50BPS[%d]:SAT#%2d",raw->len,prn); 
        for (i=0;i<raw->len;i++) { if (i==0||i==3||i==7||i==11) printf("|"); printf("%02X",U1(p+i)); } 
    }*/

    if (raw->len!=43) {
        trace(2,"SiRF 50BSP length error: len=%d\n",raw->len);
        printf("SiRF 50BSP length error: len=%d\n",raw->len);
        return 0;
    }

    if (!(sat=satno(SYS_GPS,prn))) {
        trace(2,"SiRF 50BPS satellite number error: prn=%d\n",prn);
        printf("SiRF 50BPS satellite number error: prn=%d\n",prn);
        return 0; }
    
    /* EXIT on parity error		*/
    word=U4(p+3+0*4); if (!decode_word(word,subfrm)) { trace(2,"SiRF 50BPS parity error: prn=%d\n",prn); return 0; }
        
    /* tlm=(U4(p+7)&0xFFFFFC)>>9;			/* Bit1: 0->URE > 4.42x URA-Index 1E-5/h   1-> URE > 5.73xURA 1E-8/h	*/    
    tow=getbitu(p,7*8,17); SFID=(U1(p+9)&0x1c)>>2;	/* printf("\nTOW=%d HOW[3]=",tow); printBits(U1(p+9)); printf(" SFID=%d",SFID); */

    p+=3; for (i=0;i<10;i++,p+=4) words[i]=U4(p)>>8;	/* 24 bits without parity and padding bits	*/
    id=(words[1]>>2)&7; 				/* OK printf("id=%d",id);			*/
    if (id<1||5<id) { trace(2,"subfrm id error: sat=%2d id=%d len=%d\n",sat,id,raw->len); return -1; }
    SFcnt[id-1]++;
    /* printf("|%d|",id); */

    /* NOTE Save Subframe	*/
    for (i=0;i<10;i++) setbitu(raw->subfrm[sat-1]+(id-1)*30,i*24,24,words[i]);	/* Save Subframe back to raw			*/
    /* id2=getbitu(raw->subfrm[sat-1]+(id-1)*30,43,3); /* subframe id */
    /* printf("%d|",id2); */

    if (U1(p)==MID_SRF50BPS && VerData==1 && id < 4) return 0;	/* EXIT if Verified Ephermeris Data present	*/
    if (id==1) { printf("\nSF#%d:",id); for (i=0;i<10;i++) printf("%06X ",words[i]); }
    
    /* Suframe 1+2 werden gespeichert, Auswertung erfolgt, wenn Subframe #3 auftritt */
    switch (id) {
        case 3:	return decode_ephem(sat,raw); /* Ephemeris */
        case 4:	/* NOTE Almanac SV 25-32 + Ionosphere + UTC 	*/
            DataID=(words[3]&0xc00000)>>22; PageID=(words[3]&0x3f0000)>>16;
            if (debug) printf ("SF4 PRN=%d DID=%d PID=%d Valid=",prn,DataID,PageID);
            x4=decode_frame(raw->subfrm[sat-1]+90,NULL,raw->nav.alm,raw->nav.ion_gps,raw->nav.utc_gps,&raw->nav.leaps);
            if (debug) { (x4==4) ? putchar('Y') : putchar('N'); printf("\n"); }
            adj_utcweek(raw->time,raw->nav.utc_gps);            
            return 9;
        case 5:	/* NOTE Almanac SV 1-24 			*/
            DataID=(words[3]&0xc00000)>>22; PageID=(words[3]&0x3f0000)>>16;
            if (debug) printf ("SF5 PRN=%d DID=%d PID=%d Valid=",prn,DataID,PageID);
            toa=(words[2]&0xff00)>>8; week=words[2]&0xff;
            raw->nav.alm[sat-1].week=week;				/* Almanac Woche setzen */
            x5=decode_frame(raw->subfrm[sat-1]+120,NULL,raw->nav.alm,NULL,NULL,NULL);
            if (debug) { (x5==5) ? putchar('Y') : putchar('N'); printf("\n"); }
            return 0;
        default: return 0; printf("\nMID #%d: Subframe %d detected",mid,id); 
    }
    return 0;
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
            printf("MID=%d SID=%d ",U1(p+4),sid); for (i=0;i<raw->len;i++) printf("%02X",U1(p+i));
            printf("\n"); return decode_sirfgen(raw);
        }
    return 0;
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
    int i,id,DW,LW,HW,w,x1,x2,x3;
    unsigned int words[11];
  
    prn=U1(p);
    if (!(sat=satno(SYS_GPS,prn))) {
        trace(2,"SiRF satellite number error: prn=%d\n",prn);
        printf("SiRF satellite number error: prn=%d\n",prn);
        return -1;
    }

    trace(4,"decode_sirfeph: sat=%2d\n",sat);

    for (j=0;j<3;j++) memcpy(raw->subfrm[sat-1]+j*30,p+1+j*30,30);

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

    printf("MID 15 ANALYSE:A0A2|002B|MI|SV|1111111111111111111111111112222222222222222222222222222233333333333333333333333333|CkSm|B0B3");

    printf("\nMID 15 ANALYSE:"); for (i=0;i<raw->len;i++) { printf("%02X",U1(p+i)); if (i==1||i==3||i==4||i==5||i==95||i==97) printf("|"); }
    printf("\nMID 15 [l=%d] SAT=%2d",raw->len,sat);
    
    /* NOTE Save Subframe */
    for (j=0;j<3;j++) {
        for (w=1,i=0;i<6;i++) { if (i==0) {w=0; words[w]=0; printf("\n"); }
            printf("j=%d i=%d p=%2d ",j,i,6+i*3+j*15);
            /* DW=U4(p+6+i*3+j*15)>>8; LW=(DW&0xfff000)>>12; HW=(DW&0xfff); */
            DW=U4(p+6+i*3+j*15)>>8; LW=(DW&0xfff000)>>12; HW=(DW&0xfff); 
            printf("W=%06X ",DW); printBits((DW&0xff0000)>>16); printf(""); printBits((DW&0xff00)>>8); printf(""); printBits((DW&0xff));
            printf(" w[%2d]=%04X w[%2d]=%04X\n",w,LW,w+1,HW); 
            words[w++]=LW; words[w++]=HW;
        }
        printf("\n"); /*  id=1; for (i=0;i<11;i++) printf("w[%2d]=%04X\n",i,words[i]); */
        id=(words[1]>>2)&7; 				/* OK printf("id=%d",id);			*/
        if (id<1||5<id) id=j+1;
        for (i=0;i<10;i++) setbitu(raw->subfrm[sat-1]+(id-1)*30,i*24,24,words[i]);	/* Save Subframe back to raw			*/
        printf("SV=%2d Subframe[%d|%d]=",sat,id,j+1);
        for (i=0;i<30;i++) printf("%02X",U1(raw->subfrm[sat-1]+(id-1)*30+i));
        printf("\n");
    }


    /* 7 X UNIT16 1111 1111 1111 1111 1111 1111 1111 */
    /* 9 words with 3 bytes per subframe = 27 bytes = 14 x 16-bit words */

    p+=2; for (i=0;i<10;i++,p+=4) words[i]=U4(p)>>8;	/* 24 bits without parity and padding bits	*/
    id=(words[1]>>2)&7; 				/* OK printf("id=%d",id);			*/
    if (id<1||5<id) { trace(2,"subfrm id error: sat=%2d id=%d len=%d\n",sat,id,raw->len); return -1; }
    SFcnt[id-1]++;
    printf("MID15:");
    printf("|%d|",id); 

    /* NOTE Save Subframe	*/
    /* for (i=0;i<10;i++) setbitu(raw->subfrm[sat-1]+(id-1)*30,i*24,24,words[i]);	/* Save Subframe back to raw			*/
}

/* NOTE DECODE MID 14 Almanac Data */
static int decode_sirfalm(raw_t *raw)
{
    int prn,sat,week,id,status,x5; /* ,ckSum=0; */
    unsigned char *D=raw->buff+5;
    unsigned char subfrm[25];

    id=(U1(D)&0xC0)>>6;		/* DataID */
    prn=U1(D)&0x3F;	   	/* SVid   */
    week=(U2(D+1)&0xFFC0)>>6;  	/* Week   */
    status=U2(D+1)&0x003F;	/* Status */
    
    if (!(sat=satno(SYS_GPS,prn))) {
        trace(2,"SiRF satellite number error: prn=%d\n",prn);
        return -1;
    }
    trace(4,"decode_sirfalm: sat=%2d\n",sat);
    return 0;

    printf("\nDECODE_ALM_0x0E[%d]:id=%2d:sat=%2d:week=%d:status=%d:",raw->len-9,id,sat,week,status); printBits(status);

    /* NOTE Check Data Alignment!	*/
    memcpy(subfrm+1,D+3,24);
    subfrm[0]=4+16; 			/* subframe id#5 setzen! */
    memcpy(raw->subfrm[sat-1]+120+5,subfrm,25);
    raw->nav.alm[sat-1].week=week;	/* Almanac Woche setzen */

    x5=decode_frame(raw->subfrm[sat-1]+120,NULL,raw->nav.alm,NULL,NULL,NULL); /* decode subframe #5 */
    printf(" x5=%d\n",x5);
/*    print_alm(sat,raw->nav.alm); */
    return 0;
}
/* NOTE DECODE MID 93 Temperature Value Output	*/
static int decode_sirftcxo(raw_t *raw)
{
    unsigned char *p; 
    int sid;

    trace(4,"decode_sirftcxo: len=%d\n",raw->len);

    p=raw->buff+4;
    sid=U1(p+1);
    if (sid != 18) return 0;	/* Exit on SubId other than #18 */
    if (InfoCnt<50) {InfoCnt++; return 0;}
    InfoCnt=0;
    if (raw->len!=34) {
        trace(2,"SiRF mid#93 length error: len=%d\n",raw->len);
        printf("SiRF mid#93 length error: len=%d\n",raw->len);
        return -1;
    }
    ClkBiasTCXO=U4(p+18)/1.0e9;	/*    printf("MID 93: tow=%d\n",U4(p+2));*/
    fprintf(stderr,"%s [INFO ] Temperature = %.1f° ",time_str(utc2gpst(timeget()),0),(float)(140*U1(p+22)/255)-40);
    fprintf(stderr,"Clock Drift = %d±%dHz ",U4(p+10),U4(p+14)); 
    fprintf(stderr,"Bias = %fs\n",U4(p+18)/1.0e9);
    return 0;
    
    printf("TOW:%6u;",U4(p+2));
    printf("wk:%4hu;",U2(p+6));
    printf("TStatus:");
    printBits(U1(p+8));
    printf(";ClkOff:%d;",U1(p+9));
    printf("ClkDrift:%d;",U4(p+10));
    printf("ClkDriftU:%d;",U4(p+14));
    printf("ClkBias:%f;",U4(p+18)/1.0e9);
    printf("Temperature:%.1f;",(float)(140*U1(p+22)/255)-40);
    return 0;
}

/* DECODE SiRF raw messages -------------------------------------------- */
static int decode_sirf(raw_t *raw)
{
    unsigned char *p=raw->buff;
    int rv,mid=U1(p+4),i=0,sid;

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
        case MID_SRFCLOCK:	return decode_sirfclock(raw);
        case MID_SRFGEOCLK:	return decode_sirfgeoclk(raw);
        case MID_SRF50BPS:	return decode_sirf50bps(raw);
        case MID_SRFNLMEAS:	return decode_sirfnlmeas(raw);
        case MID_SRFEPH:	return decode_sirfeph(raw);
        case MID_SRFALM:	return decode_sirfalm(raw);
        case MID_SRF50BPSHQ:	return decode_sirf50bpshq(raw);
        case MID_SRFSVSTATE:	return decode_sirfsvstate(raw);
        case MID_SRFTCXO:	return decode_sirftcxo(raw);
        case MID_SRFMTDO:	return decode_sirfmtdo(raw);
        case MID_SRFAUXDATA:	return decode_sirfauxdata(raw);
        case 6:			/* MID  6 Software Version String 				*/
            fprintf(stderr,"%s [INFO ] Firmware = %s\n",time_str(utc2gpst(timeget()),0),p+7);
            return 0;
        case 11: return 0;	/* MID 11 Command Acknowledgment A0A2|0003|0B|8400|008F|B0B3 	*/
        case 50:		/* MID 50 SBAS Parameters					*/
            if (U1(p+5)==0) return 0;
            fprintf(stderr,"%s [INFO ] ",time_str(utc2gpst(timeget()),0));
            /*[INFO ]A0A2000D32 000012000000000000000000 0044B0B3 
                             +4 +5+6+7                         
            for (i=0;i<raw->len;i++) printf("%02X",U1(p+i)); */
            printf("SBAS PRN:%3d ", U1(p+5));
            printf("Integretiy:%3d ", U1(p+6));
            printf("Timeout:%3d ", U1(p+7));
            /* printf("FlagBits:"); printBits(U1(p+8)); */
            printf("Timeout:");
            ((U1(p+8) & (1 << 1)) ? putchar('D') : putchar('U'));
            printf(" Health:");
            ((U1(p+8) & (1 << 2)) ? putchar('Y') : putchar('N'));
            printf(" Correction:");
            ((U1(p+8) & (1 << 3)) ? putchar('Y') : putchar('N'));
            printf(" SBAS PRN:");
            ((U1(p+8) & (1 << 3)) ? putchar('D') : putchar('U'));
            printf("\n");
            return 0;
        case 75:		/* MID 75 ACK/NACK/ERROR Notification A0A2|0007|4B|01|9400FA0000|01DA|B0B3 */
            sid=U1(p+5);
            fprintf(stderr,"%s [NACK ] ",time_str(utc2gpst(timeget()),0));
            /* for (i=5;i<raw->len-4;i++) printf("%02X ",U1(p+i)); /* 51 [NACK ] 01 49 04 FA 00 00 */
            printf("Check command script in terms of MID %d!",U1(p+6));
            printf("\n");
            return 0;
        default: 	    	return decode_sirfgen(raw);
            return 0;
    }
    return 0;
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
 * return : status (-1: error message, 0: no message, 1: input observation data,
 *                  2: input ephemeris,
 *                  9: input ion/utc parameter)
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

