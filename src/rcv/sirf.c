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
*-------------------------------------------------------------------------------*/
#include "rtklib.h"

#define SIRFSYNC1	0xA0    /* SiRF binary message start sync code 1		*/
#define SIRFSYNC2	0xA2    /* SiRF binary message start sync code 2 		*/
#define SIRFSYNCEND1	0xB0    /* SiRF binary message end sync code 1 			*/
#define SIRFSYNCEND2	0xB3    /* SiRF binary message end sync code 2 			*/

#define MID_SRFCLOCK	0x07    /* SiRF binary clock status 				*/
#define MID_SRF50BPS	0x08    /* SiRF binary 50BPS data 				*/ 
#define MID_SRFNLMEAS	0x1c    /* SiRF binary navlib measurement data 			*/
#define MID_SRFEPH	0x0f	/* SiRF Ephemeris Data (Response to Poll) 		*/
#define MID_SRFALM	0x0e	/* SiRF Almanac Data (Response to Poll) 		*/
#define MID_SRF50BPSHQ	0x38    /* SiRF binary 50BPS data 				*/ 
#define MID_SRFSVSTATE  0x1e	/* SiRF Navigation Library SV State Data		*/
#define MID_SRFAUXDATA	0x40	/* SiRF Navigation Library Auxiliary Measurement Data	*/

#define SIRFMAXRAWLEN 2047      /* max length of SiRF binary raw message 		*/

/* double L[NFREQ+NEXOBS]; /* observation data carrier-phase (cycle) */
/* double P[NFREQ+NEXOBS]; /* observation data pseudorange (m) */
/* float  D[NFREQ+NEXOBS]; /* observation data doppler frequency (Hz) */
/* NOTE * invalid carrier phase measurements are flagged in RTKLIB by setting the carrier phase value to zero. */

typedef struct {        /* MID 4 + MID28 + MID30 + MID64 data 					*/
    int sat;            /* satellite number 						*/
    int trackStat;	/* Measured Tracker Data Out - Status				*/
    int TimeTag28;	/* TimeTag28 							*/
    gtime_t time;	/* GPS Software Time % clock bias = GPS time of measurement 	*/
    double PseudoR;	/* Pseudorange [m] 						*/
    float CarrFreq28;	/* Carrier Frequency [m/s] 					*/
    double CarrPhase28; /* Carrier Phase [m] 						*/
    int TimeTrack;	/* Time in Track [m/s] 						*/
    int SyncFlag;	/* Sync Flaggs							*/
    int cno;		/* reserved for Std dev.					*/
    int DeltaRange;	/* Delta Range Interval						*/
    int MeanDRTime;	/* Mean Delta Range Time					*/
    int ExtraTime;	/* Extrapolation Time						*/
    int PhErrCnt;	/* Phase Error Count						*/
    double px,py,pz;	/* position X,Y,Z 						*/
    double vx,vy,vz;	/* velocity X,Y,Z 						*/
    double clkbias;	/* Clock Bias 							*/
    int clkdrift;	/* Clock drift							*/
    int iono;		/* Ionospheric Delay 						*/
    int extStat;	/* Extended Status: SF Sync verified, Possible Cycle Slip, SF Sync lost, Multipath, Weak Frame */
    int RecovStat;	/* Weak Bit Sync, False Lock, wrong BitSync, wrong FrameSync, Bad PrePos  */
    int TimeTag64;	/* Measurement Time Tag */
    double CarrPhase64;	/* Carrier Phase */
    float CarrFreq64;	/* Carrier Freq */
    signed short CarrAcc;	/* Carrier Acceleration (Doppler Rate) */
    signed short PRNoise;	/* Pseudorange Noise (one sigma, normalized and left-shifted 16 bits */
    signed short dRngQ;		/* Delta Range Quality						*/
    signed short PhLockQ;	/* Phase Lock Quality, normalized and left-shifted 8 bits */
    
} auxData_t;

/* Global Variabels ------------------------------------------------------------*/
int VerData = 0;
double ClkBiasTCXO = 96250/10e+9;	/* calculated clock bias of TCXO 				*/

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
    }else {
    q = (unsigned char *)&value+3;
    for (i=0;i<4;i++) *q--=*p++;
    q = (unsigned char *)&value+7;
    for (i=0;i<4;i++) *q--=*p++;
    return value;
    }
}

static void printBits(unsigned char num)
{
   int bit;
   for(bit=8*sizeof(num)-1; bit>=0; bit--) {
        (num & (1 << bit)) ? putchar('1') : putchar('0');
   }
}

/* static char* BitMask(unsigned char num)
{
   int bit;
   char* BitMask;
   BitMask[8] = "";
   for(bit=8*sizeof(num)-1; bit>=0; bit--) {
        (num & (1 << bit)) ? strcat(BitMask,'1') : strcat(BitMask,'0');
   }
   return BitMask;
}*/

/* checksum ------------------------------------------------------------------*/
static int chksum(const unsigned char *buff, int len)
{
    int i;
    unsigned short sum=0;

    if (len<8) return 0;
    for (i=4;i<len-4;i++) sum=0x7fff&(sum+buff[i]);
    return (sum>>8)==buff[len-4]&&(sum&0xFF)==buff[len-3];
}


/* decode id#04 Measured Tracker Data Out ---------------------------------- */
static int decode_sirfmtdo(raw_t *raw)
{
    unsigned int i,j,ch,sat,state;
    unsigned char *p;
    unsigned gsw230=strstr(raw->opt,"-GSW230")!=NULL;

/*    trace(4,"decode_sirfnlmeas: len=%d\n",raw->len);
    if (raw->len!=64) {
        trace(2,"SiRF mid#28 length error: len=%d\n",raw->len);
        return -1;
    } */

    p=raw->buff+4; ch=U1(p+1);
    if (ch>=MAXOBS) {
        trace(2,"SiRF mid#28 wrong channel: ch=%d\n",ch);
        return -1;
    }
    sat=satno(SYS_GPS,U1(p+6)); 

        /* printf("Chans:%d;",U1(p+7));
        printf("\n"); */
        for(j=0; j<=11; j++) {
/*            printf("CH#%2d:",j+1);*/
            sat=satno(SYS_GPS,U1(p+9+j*15));
            if (sat) {
/*                printf("SV_id:%2d;",sat); */
                state=U1(p+12+j*15);
/*                printBits(U1(p+12+j*15)); */
                auxData[sat].trackStat=state;
/*                printf(":CP=%d ",(auxData[sat].trackStat&0x02)>>1); */

            }
             
            /* printf(";Valid_Phase:");
            ((U1(message+12+j*15) & (1 << 1)) ? putchar('Y') : putchar('N'));
            printf(";Subframe_Complete:");
            ((U1(message+12+j*15) & (1 << 3)) ? putchar('Y') : putchar('N'));
            printf(";Costas_Lock:");
            ((U1(message+12+j*15) & (1 << 4)) ? putchar('Y') : putchar('N'));
            printf(";Code_Lock:");
            ((U1(message+12+j*15) & (1 << 5)) ? putchar('Y') : putchar('N'));
            printf("\n"); */
        }

    return 0;
}    


/* decode id#07 navigation data (user) ---------------------------------------
MID #7 contains the clock bias that must be considered. 
1) Adjust the GPS Software time by subtracting clock bias,
2) adjust pseudorange by subtracting clock bias times the speed of light, 
3) and adjust carrier phase by subtracting clock bias times speed of light/GPS L1 frequency. 

For a nominal clock drift value of 96.25 kHz (equal to a GPS Clock frequency of 24.5535 MHz),
the correction value is 18315.766 m/s.

NOTE Appendix A: Reported clock bias and clock bias computed using the above
     formula will likely agree only to within a few nanoseconds because the
     actual measurement interval may be slightly more or less than an exact
     second, and the clock drift is only reported to a (truncated) 1 Hz 
     resolution. */
static int decode_sirfclock(raw_t *raw)
{
    unsigned drift;
    double bias;
    int i,j,n=0;
    unsigned char *p;
    unsigned wn;
    double tow;
    obsd_t *src,*dst;
    gtime_t tr,time0={0};

    trace(4,"decode_sirfclock: len=%d\n",raw->len);

    p=raw->buff+4;
    if (raw->len!=28) {
        trace(2,"SiRF mid#7 length error: len=%d\n",raw->len);
        return -1;
    }

    wn=U2(p+1); tow=U4(p+3)*0.01; raw->time=gpst2time(wn,tow);

    /* GPS Clock Frequency 
       NOTE MID 93.18 provides Clock Drift Uncertainty */
    /* printf("\nGPS Clock:%f kHz, RepBias:%8.3f µs, ",(FREQL1+drift)*16/1540/1e+3,U4(p+12)/1e+3);
    printf("CalcBias = %8.3f µs, ",1e6*drift/FREQL1);
    printf("DeltaTime = %.3f s, ",tow-U4(p+16)/1000); */

    drift=U4(p+8);		/* Clock Drift in CSR receivers is directly related to the frequency of the GPS clock, 
                                   derived from the GPS crystal. Clock drift can be used as a Doppler bias for Carrier
                                   Frequency calculation, reported in Message ID 28. 
                                   NOTE Clock drift is only reported to a truncated 1 Hz resolution! */
    bias=U4(p+12)/1e+9;	/* TBD: In einem 125er Array ablegen und rsd rechnen? */
    /*bias=drift/FREQL1; 		/* ClkBiasTCXO; */
    /*printf("\nRAW:"); for (i=0;i<raw->len;i++) printf("%02X",U1(p-4+i));	/* Print RAW Data Bytes	*/

    /* printf("\nBIAS %08X->%f",U4(p+12),bias); */
    /*,FREQL1,FREQL1*bias);*/

    /* NOTE Korrektur von Carrier Frequency, Phase, Pseudorange, Zeitbasis, etc. */
    for (i=0;i<raw->obuf.n&&i<MAXOBS;i++) {
        src=&raw->obuf.data[i];
        if (!satsys(src->sat,NULL)) continue;
        if (fabs(tow+bias-src->time.sec)>0.1) continue;
        dst=&raw->obs.data[n++]; *dst=*src;		/* Kopie des Orginals erstellen */
        dst->time=gpst2time(wn,src->time.sec-bias);	/* Subtract Clock Bias to get time of receipt 	*/
        tr=dst->time;					/* NOTE TBD */

        /* printf("\nClkStatus//Data//AuxData\n"); */
        /* printf("\nClock Drift(s)  =   %15d // %d // %d",U4(p+8),auxData[dst->sat].clkdrift,auxData[dst->sat].CarrAcc); 
           printf("\nClock bias(es)  =   %15d // %f",U4(p+12),auxData[dst->sat].clkbias); */

        /* Carrier frequency adjustment (acc. OSP Manual - bei U-Blox keine Korrektur)
           Corrected Carrier Frequency (m/s) = Reported Carrier Frequency (m/s) – Clock Drift (Hz)*C / 1575420000 Hz. 		
                                             = 16756.8m/s - 18315.766m/s ...? */
        /* printf("\nCarrier Frequency = %15.3f // %15.3f // %15.3f",dst->D[0],auxData[dst->sat].CarrFreq28,auxData[dst->sat].CarrFreq64); */
        /* dst->D[0]-=drift;  /*FREQL1/CLIGHT;		/* Adjust reported Carrier Frequency [m/s] by Clock Drift [Hz]			*/
        /* printf(" // %15.3f",dst->D[0]); */
        
        /* Carrier Phase Adjustmest [m] -> [Cycles]  -> Mid 64 Output in L1 Cycle, M28 in Metern
           1) Subtract clock bias times speed off light and divide by GPS L1 frequency 
           2) Convert from [m] to Cycles by dividing by wavelength			    */
        /* NICHT ZU GEBRAUCHEN! dst->L[0]=auxData[dst->sat].CarrPhase64; */
        if (dst->L[0]) {
              /* printf("\nCarrier Phase     = %15.3f // %15.3f // ",dst->L[0],auxData[dst->sat].CarrPhase28); */
/*            printf("%15.3f",auxData[dst->sat].CarrPhase64); */
            dst->L[0]-=FREQL1*bias;
/*            printf("\nCP %d -",auxData[dst->sat].CarrPhase64);
            dst->L[0]=auxData[dst->sat].CarrPhase64;
            printf("\nCP %f !",dst->L[0]); */
            if (((auxData[dst->sat].trackStat&0x02)>>1)>0) dst->L[0]=0;	/* MID 4: Carrier Phase valid? */
            /* printf("%15.3f // %15.3f // D=%.1f // DQ=%6d // PQ=%5d // NQ=%5d,%5d",auxData[dst->sat].CarrPhase64,dst->L[0],100*FREQL1*bias/dst->L[0],auxData[dst->sat].dRngQ,auxData[dst->sat].PhLockQ,auxData[dst->sat].PRNoise,auxData[dst->sat].cno);  */
            /* dst->SNR[0]=160; /*auxData[dst->sat].PhLockQ/3; */
        }
        
        /* Adjust Pseudorange measurement in [m] by clock bias */
        /* printf("\nPseudorange=      = %15.3f // %15.3f // ",dst->P[0],auxData[dst->sat].PseudoR); */
        printf("\nPseudorange=%15.3f",dst->P[0]);
        dst->P[0]-=CLIGHT*bias; /*printf(" // %15.3f",dst->P[0]);*/
        printf(" // %15.3f // %6d",dst->P[0],auxData[dst->sat].iono);
        dst->P[0]-=auxData[dst->sat].iono;
        printf(" // %15.3f",dst->P[0]);

                
        /* LLI: bit1=slip,bit2=half-cycle-invalid */
        /* printf("\nStati (Ext)="); printBits(auxData[dst->sat].extStat); printf("|"); printBits(auxData[dst->sat].RecovStat);printf("|");
        printf("Ext:%d",(auxData[dst->sat].extStat&0x3c)>>2); */
        if ((auxData[dst->sat].extStat&0x3c)>>2) dst->LLI[0]=1;		/* Cycle Slip, Subframe sync, Multipath */
        if ((auxData[dst->sat].RecovStat&0x14)>>2) dst->LLI[0]=1;	/* Bad PrePos, wrong BitSync		*/
        if (dst->LLI[0]) dst->L[0]=0;

        if (!dst->LLI[0]) {
            /* printf("\nSAT#%2d:CarrAcc=%6d;NoisePR=%6d;DeltaRangeQ=%6d;PhaseLockQ=%6d",dst->sat,auxData[dst->sat].CarrAcc,auxData[dst->sat].PRNoise,auxData[dst->sat].dRngQ,auxData[dst->sat].PhLockQ); */
            /* DeltaRange -> immer 1000/500 printf("\nDeltaRange=%6d,%6d;",auxData[dst->sat].DeltaRange,auxData[dst->sat].MeanDRTime); */
            /*MeanDeltaRangeT=%d;ExtrapolationTime=%d",dst->sat,auxData[dst->sat].DeltaRange,auxData[dst->sat].MeanDRTime,auxData[dst->sat].ExtraTime);*/
        }    

    }
    raw->obs.n=n;

    /* clear observation data buffer */
    for (i=0;i<MAXOBS;i++) {
        raw->obuf.data[i].time=time0;
        for (j=0;j<NFREQ+NEXOBS;j++) {
            raw->obuf.data[i].L[j]=raw->obuf.data[i].P[j]=0.0;
            raw->obuf.data[i].D[j]=0.0;
            raw->obuf.data[i].SNR[j]=raw->obuf.data[i].LLI[j]=0;
            raw->obuf.data[i].code[j]=CODE_NONE;
        }
    }


    return n>0?1:0;
}
/* decode id#28 measurement block --------------------------------------------
NOTE The reported Time of Measurement, Pseudorange and Carrier Phase are all uncorrected values.
NOTE GPS Software Time – Clock Bias = Time of Receipt = GPS Time. 
     GPS Software Time – Pseudorange (sec) = Time of Transmission = GPS Time. 
     Adjust SV position in MID 30 by (GPS Time MID 30 – Time of Transmission) * Vsat.      */
static int decode_sirfnlmeas(raw_t *raw)
{
    int i; unsigned ch,flags,pherrcnt;
    unsigned char *p;
    unsigned gsw230=strstr(raw->opt,"-GSW230")!=NULL;
    obsd_t *obsd;
    char *q; int week; double tow,toff,tt,tn,tadj=0.0; gtime_t time;	/* U-Blox */

    trace(4,"decode_sirfnlmeas: len=%d\n",raw->len);
    if (raw->len!=64) {
        trace(2,"SiRF mid#28 length error: len=%d\n",raw->len);
        return -1;
    }

    p=raw->buff+4; ch=U1(p+1);
    if (ch>=MAXOBS) {
        trace(2,"SiRF mid#28 wrong channel: ch=%d\n",ch);
        return -1;
    }
    
    obsd=&raw->obuf.data[ch];
    if (ch>=raw->obuf.n) raw->obuf.n=ch+1;

    /* Fill Observation Data Structure */
    obsd->sat=satno(SYS_GPS,U1(p+6)); obsd->code[0]=CODE_L1C;
    obsd->time.sec=R8(p+7,gsw230); 					/* GPS Software Time 							*/
    obsd->P[0]=R8(p+15,gsw230);						/* Pseudorange, without corrections [m]					*/
    obsd->D[0]=R4(p+23)*FREQL1/CLIGHT;					/* Carrier Frequency [m/s] -> Hz (Doppler Extract erst mit GPS Clock!)	*/
    obsd->L[0]=R8(p+27,gsw230)*FREQL1/CLIGHT;  /*((U1(p+37)&0x02)>>1);	/* Carrier Phase [m], report as Cycles					*/
    obsd->SNR[0]=U1(p+38); for (i=39;i<=47;i++) {			/* Kleinstes Signalrauschverhältnis ermitteln und abspeichern		*/
        if (U1(p+i)<obsd->SNR[0]) obsd->SNR[0]=U1(p+i);
    }
    obsd->SNR[0]*=4.0;
    flags=U1(p+37); pherrcnt=U1(p+54);					/* Constistent code epoch alignment, phase errors > 60 degrees prec. second */
    obsd->LLI[0]=flags&0x02&&pherrcnt<50?0:1;

    /* Fill Auxiliary Data Structure */
    auxData[obsd->sat].time.sec=obsd->time.sec;
    auxData[obsd->sat].CarrPhase28=obsd->L[0];    
    auxData[obsd->sat].CarrFreq28=obsd->D[0];
/*    printf("\nXXX RAW=%04X%04X->%f",U4(p+27),U4(p+31),R8(p+27,gsw230)*FREQL1/CLIGHT);
    printf("YYY obsd=%f,auxData=%f",obsd->L[0],auxData[obsd->sat].CarrPhase28); */
/*    printf("\nXXX28CarrF28=%08X->%f",U4(p+23),R4(p+23)); */
    auxData[obsd->sat].PseudoR=obsd->P[0];
    auxData[obsd->sat].TimeTrack=U2(p+35);
    auxData[obsd->sat].SyncFlag=U1(p+37);
    auxData[obsd->sat].cno=obsd->SNR[0]; 
    auxData[obsd->sat].DeltaRange=U2(p+48);
    auxData[obsd->sat].MeanDRTime=U2(p+50);
    auxData[obsd->sat].ExtraTime=S2(p+52);
    auxData[obsd->sat].PhErrCnt=U1(p+54);
    
    return 0;

    /*   tstat=U1(p+30);    * trkStat *
    /* 	if (!(tstat&1)) P=0.0;	*
    /* if (!(tstat&2)||L==-0.5||cpstd>CPSTD_VALID) L=0.0; * invalid phase */
    /* raw->obs.data[j].qualL[f-1]=cpstd<=7?cpstd:0;
       raw->obs.data[j].qualP[f-1]=prstd; */

    /* NOTE * offset by time tag adjustment - nur anpassen, wenn Phase gültig!
    if (toff!=0.0&&L!=0.0) {
        P-=toff*CLIGHT;
        L-=toff*sig_freq(sys,f,frqid-7);
    } */
    
    /* Std Dev of receiver -> obs.qualX */
    /* prstd=U1(p+27)&15; /* pseudorange std-dev */
    /* cpstd=U1(p+28)&15; /* cpStdev (m) */
    /* prstd=1<<(prstd>=5?prstd-5:0); /* prstd=2^(x-5) */
    /* prstd=prstd<=9?prstd:9;  /* limit to 9 to fit RINEX format */
}

/* decode id#30 Navigation Library SV State -------------------------------------------
NOTE When using MID 30 SV position, adjust for difference between GPS TIME MID 30 and
     Time of Transmission (see note in MID 28), Iono dely is not included in
     pseudorange in MID 28								*/
static int decode_sirfsvstate(raw_t *raw)
{
    int i,sat;
    unsigned ch,flags,pherrcnt;
    unsigned char *p;
    unsigned gsw230=strstr(raw->opt,"-GSW230")!=NULL;

/*    trace(4,"decode_sirfnlmeas: len=%d\n",raw->len);
    if (raw->len!=64) {
        trace(2,"SiRF mid#30 length error: len=%d\n",raw->len);
        return -1;
    } */
    p=raw->buff+4; ch=U1(p+1); sat=satno(SYS_GPS,U1(p+1));
    if (ch>=MAXOBS) {
        trace(2,"SiRF mid#30 wrong channel: ch=%d\n",ch);
        return -1;
    }

    auxData[sat].time.sec=R8(p+2,gsw230);
    auxData[sat].px=R8(p+10,gsw230);
    auxData[sat].py=R8(p+18,gsw230);
    auxData[sat].pz=R8(p+26,gsw230);
    auxData[sat].vx=R8(p+34,gsw230);
    auxData[sat].vy=R8(p+42,gsw230);
    auxData[sat].vz=R8(p+50,gsw230);
    auxData[sat].clkbias=R8(p+58,gsw230);
    auxData[sat].clkdrift=S4(p+66);	/* R4, zum Beispiel passt aber S4 */
    auxData[sat].iono=R4(p+79);		/* R4, zum Beispiel passt aber S4 */
/*    printf("\n!!Iono=%f,%d",R4(p+79),S4(p+79));
    printf("\n!!CLK:%08X->%lf,%f,%d",U4(p+66),R4(p+66)*1e+12,R4(p+66),R4(p+66));
    printf("\n!!CLKD:%08X%08X->%lf,%f,%d",U4(p+58),U4(p+62),R8(p+58,gsw230),R8(p+58,gsw230),R8(p+58,gsw230)); */
    return 0;
}
/* decode id#64 Navigation Library Auxiliary Measurement Data------------------------ */
static int decode_sirfauxdata(raw_t *raw)
{
    int i,sat;
    unsigned ch,flags,pherrcnt;
    unsigned char *p;
    unsigned gsw230=strstr(raw->opt,"-GSW230")!=NULL;

/*    trace(4,"decode_sirfnlmeas: len=%d\n",raw->len);
    if (raw->len!=64) {
        trace(2,"SiRF mid#30 length error: len=%d\n",raw->len);
        return -1;
    } */
    p=raw->buff+4; if (U1(p+1)!=2) return 0;	/* Exit on other subchannels */
    ch=U1(p+2); sat=satno(SYS_GPS,U1(p+2));
    if (ch>=MAXOBS) {
        trace(2,"SiRF mid#64 wrong channel: ch=%d\n",ch);
        return -1;
    }
    auxData[sat].extStat=U1(p+4);
    auxData[sat].RecovStat=U1(p+62);
/*    printf("XXXXRecov=%02X,SWU=%d",U1(p+62),U4(p+63));*/
    auxData[sat].TimeTag64=U4(p+6);
    auxData[sat].CarrPhase64=S4(p+14);	/* S4?*/
/*    printf("\nZZZ SAT=%2d RAW=%08X->%d",sat,U4(p+14),S4(p+14));
    printf("-> %d !!",U4(p+14));
    printf("JJJ auxData=%f",auxData[sat].CarrPhase64); */
    auxData[sat].CarrFreq64=S4(p+18)*0.000476;
/*    printf("XXXXX=%08X->%d->%lf",U4(p+18),S4(p+18),S4(p+18)*0.000476);*/
    auxData[sat].CarrAcc=S2(p+22); 
    auxData[sat].PRNoise=S2(p+42);
    auxData[sat].dRngQ=S2(p+44);
    auxData[sat].PhLockQ=S2(p+46);
/*    printf("XXXX %04X->%d",U2(p+46),S2(p+46)); */

/* Carrier frequency may be interpreted as the measured Doppler on the received signal.
   The value is reported in metres per second but can be converted to hertz using the
   Doppler equation:
   Doppler frequency / Carrier frequency = Velocity / Speed of light, 
   where Doppler frequency is in Hz; Carrier frequency = 1,575,420,000 Hz;
   Velocity is in m/s; Speed of light = 299,792,458 m/s.
   Note that the computed Doppler frequency contains a bias equal to the current clock
   drift as reported in Message ID 7. This bias, nominally 96.250 kHz, is equivalent to over 18 km/s. */

   return 0;
}
/* save subframe -------------------------------------------------------------*/
static int save_subfrm(int sat, raw_t *raw)
{
    unsigned int i,id,word;
    unsigned char *p=raw->buff+7;
    unsigned char subfrm[30];
    
    for (i=0;i<10;i++) {
        word=U4(p+i*4);
        if (!decode_word(word,subfrm+i*3)) {
            trace(4,"save_subfrm: parity error sat=%2d i=%d word=%08x\n",sat,i,word);
/*            printf("save_subfrm: parity fail sat=%2d i=%d word=%08x\n",sat,i,word); */
/*            printf("FAIL\n"); ... kommt zu OFT VOR?!?! */
            return -1; /* 0 */
        } 
    }
/*    printf("OK"); */

    id=getbitu(subfrm,43,3);
/*    printf(":SF=%d:",id); */
    if (id<1||5<id) {   
        printf("id=%d->FAIL\n",id);
        return -1; /* 0 */
    } 

    trace(4,"save_subfrm: sat=%2d id=%d\n",sat,id);
/*    printf("par=ok:sat=%2d:id=[",sat); 
    switch (id) {
        case 3:  printf("ephem"); break;
        case 4:  printf("alm_1"); break;
        case 5:  printf("alm_2"); break;
        default: printf(" !%d! ",id); break;
    }
    printf("]"); */

    memcpy(raw->subfrm[sat-1]+(id-1)*30,subfrm,30);

/*    printf("\nDECODE_IS-GPS-200:sat=%2d:id=%2d:   ",sat,id); 
    for (i=0;i<30;i++) {
        printf("%02X",subfrm[i]);
    }
    printf("\n"); */

    return id;
}
/* report ephemeris 
   Struktur von eph -> rtklib.h */    
static void print_eph(eph_t eph)
{
/*    printf("\nDEC:");    
    printf("sat=%d:iode=%d:iodc=%d:sva=%d:svh=%d:week=%d:code=%d:flag=%d:",eph.sat,eph.iode,eph.iodc,eph.sva,eph.svh,eph.week,eph.code,eph.flag);
    printf("toe=%.2f:toc=%.2f:ttr=%.2f:\n",time2gpst(eph.toe, &eph.week),time2gpst(eph.toc, &eph.week),time2gpst(eph.ttr, &eph.week));
    printf("A=%f:e=%f:i0=%f:OMG0=%f:omg=%f:M0=%f:deln=%f:OMGd=%f:idot=%f:\n",eph.A,eph.e,eph.i0,eph.OMG0,eph.omg,eph.M0,eph.deln,eph.OMGd,eph.idot);
    printf("crc=%f:crs=%f:cuc=%f:cus=%f:cic=%f:cis=%f:toes=%f:fit=%f:f0=%f:f1=%f:f2=%f:\n",eph.crc,eph.crs,eph.cuc,eph.cus,eph.cic,eph.cis,eph.toes,eph.fit,eph.f0,eph.f1,eph.f2);
    printf("tgd[0]=%f:tgd[1]=%f:tgd[2]=%f:tgd[3]=%f\n",eph.tgd[0],eph.tgd[1],eph.tgd[2],eph.tgd[3]); */
    /*    sqrtA     =R8(p,gsw230);        p+=8;
    eph.e     =R8(p,gsw230);        p+=8;
    eph.M0    =R8(p,gsw230)*SC2RAD; p+=8;
    eph.OMG0  =R8(p,gsw230)*SC2RAD; p+=8;
    eph.i0    =R8(p,gsw230)*SC2RAD; p+=8;
    eph.omg   =R8(p,gsw230)*SC2RAD; p+=8;
    eph.deln  =R4(p)*SC2RAD; p+=4;
    eph.OMGd  =R4(p)*SC2RAD; p+=4;
    eph.idot  =R4(p)*SC2RAD; p+=4;
    eph.crc   =R4(p);        p+=4;
    eph.crs   =R4(p);        p+=4;
    eph.cuc   =R4(p);        p+=4;
    eph.cus   =R4(p);        p+=4;
    eph.cic   =R4(p);        p+=4;
    eph.cis   =R4(p);        p+=4;
    eph.A     =sqrtA*sqrtA;*/
}
static void print_alm(int sat, alm_t *alm)
{
    printf("\nDEC_ALM:");
    printf("sat=%d,alm.sat=%d:svh=%d:svconf=%d:week=%d:",sat,alm[sat-1].sat,alm[sat-1].svh,alm[sat-1].svconf,alm[sat-1].week);
    printf("toa=%.2f:",time2gpst(alm->toa, &alm->week));
    printf("A=%f:e=%f:i0=%f:OMG=%f:omg=%f:M0=%f:OMGd=%f:",alm->A,alm->e,alm->i0,alm->OMG0,alm->omg,alm->M0,alm->OMGd);
    printf("toas=%f:f0=%f:f1=%f",alm->toas,alm->f0,alm->f1); 
}
/* MID 15 - decode ephemeris (polled data) 
It is essential to consult with GPS-ICD documentation to become more familiar with 
conversions. For more information, see https://www.gps.gov/technical/icwg/IS-GPS-200K.pdf
ICD200 X(24bits) 50-bps, 24-bit data word (See GPS ICD 200) Subframe 1,2,3 Words 2-10
SiRF   X(16bits) SiRF Data structure per subframe, D[0] -> D[14], 2 byte words            */
static int decode_ephem(int sat, raw_t *raw)
{
    eph_t eph={0};

    if (decode_frame(raw->subfrm[sat-1]   ,&eph,NULL,NULL,NULL,NULL)!=1||
            decode_frame(raw->subfrm[sat-1]+30,&eph,NULL,NULL,NULL,NULL)!=2||
            decode_frame(raw->subfrm[sat-1]+60,&eph,NULL,NULL,NULL,NULL)!=3) return 0;

    if (!strstr(raw->opt,"-EPHALL")) {
        if (eph.iode==raw->nav.eph[sat-1].iode) return 0; /* unchanged */
    }
    eph.sat=sat;
    raw->nav.eph[sat-1]=eph;
    raw->ephsat=sat;

    return 2;
}

/* decode almanac and ion/utc ------------------------------------------------*/
static int decode_alm1(int sat, raw_t *raw)
{
    /* int week; */
    trace(4,"decode_alm1 : sat=%2d\n",sat);
    /* week=(U2(raw->buff+5)&0xFFC0)>>6;   			* Week *
    printf("sat=%d,week=%d",sat,week);*/
    raw->nav.alm[sat-1].week=(U2(raw->buff+5)&0xFFC0)>>6;	/* Almanac Woche setzen */
    decode_frame(raw->subfrm[sat-1]+90,NULL,raw->nav.alm,raw->nav.ion_gps,
            raw->nav.utc_gps,&raw->nav.leaps);
    /* print_alm(sat,raw->nav.alm);*/

    return 0;
}
/* decode almanac ------------------------------------------------------------*/
static int decode_alm2(int sat, raw_t *raw)
{
    int week;

    trace(4,"decode_alm2 : sat=%2d\n",sat);
    week=(U2(raw->buff+5)&0xFFC0)>>6;   			/* Week */
    raw->nav.alm[sat-1].week=week;				/* Almanac Woche setzen */
    decode_frame(raw->subfrm[sat-1]+120,NULL,raw->nav.alm,NULL,NULL,NULL);
    /* print_alm(sat,raw->nav.alm); */

    return  0;
}

/* decode SiRF 50BPS message ------------ ------------------------------------*/
static int decode_sirf50bps(raw_t *raw)
{
    int prn,sat,id,word;
    unsigned char *p=raw->buff+4;
    unsigned char subfrm[30];

    trace(4,"decode_sirf50bps: len=%d\n",raw->len);

    if (raw->len<51) {
        trace(2,"SiRF 50BSP length error: len=%d\n",raw->len);
        printf("SiRF 50BSP length error: len=%d\n",raw->len);
        return 0;
    }
    
    prn=U1(p+2);
    if (!(sat=satno(SYS_GPS,prn))) {
        trace(2,"SiRF 50BPS satellite number error: prn=%d\n",prn);
        printf("SiRF 50BPS satellite number error: prn=%d\n",prn);
        return 0;
    }
    
    /* Check Verified Ephermeris Data */
    word=U4(p+7); if (!decode_word(word,subfrm)) return 0;	/* EXIT on parity error				*/
    id=(U1(subfrm+2)&0x1C)>>2; 					/* printf("\nMID#%d,SF:%d:",U1(p),id);		*/
 /*    VerData=0;*/
    if (U1(p)==MID_SRF50BPS && VerData==1 && id != 5) return 0;	/* EXIT if Verified Ephermeris Data present	*/
    
    /* Save Subframe */
    id=save_subfrm(sat,raw); 					/* printf("|Chk:%d|ID:%d",(U1(subfrm+2)&0x1C)>>2,id); */

    /* Suframe 1+2 werden gespeichert, Auswertung erfolgt, wenn Suframe #3 auftritt */
    if (id==3) return decode_ephem(sat,raw); /* Ephemeris */
    if (id==4) return decode_alm1 (sat,raw); /* Almanac SV 25-32 + Ionosphere + UTC */
    if (id==5) return decode_alm2 (sat,raw); /* Almanac SV 1-24 */
    return 0;
}

/*    for (i=0;i<30;i++) subfrm[i]=0;					* Set subfrm to ZERO 	*
    printf("\nRAW:"); for (i=0;i<4;i++) printf("%02X",U1(p+3+1*4+i));	* Print RAW Data Bytes	*
    word=U4(p+3+1*4); printf("-> W2:%08X",word);    			* Print word value 	*
    for (i=0;i<4;i++) {							* Print Bit Pattern	*
        printf("|"); printBits(U1(p+3+1*4+i)); 
    }
    if (!decode_word(word,subfrm)) {					* Decode Subframe	*
        trace(4,"save_subfrm: parity error sat=%2d i=%d word=%08x\n",sat,i,word);
        printf(" -> parity error");
        return -1;
    }
    printf("\nGPS-Decode[#%2d]:  ",U1(p)); for (i=0;i<4;i++) printf("%02X",U1(subfrm+i));	* Print Subframe	*
    for (i=0;i<4;i++) {							* Print Bit Pattern	*
        printf("|"); printBits(U1(subfrm+i)); 
    }
*/

/* decode navigation data subframe 2 -----------------------------------------
-> Import von rcvraw.c */

/* decode MID 15 Ephemeris Data
   Struktur von eph -> rtklib.h */    
static int decode_sirfeph(raw_t *raw)
{
    eph_t eph={0};
    int prn,sat,j;
    unsigned char *p=raw->buff+5;
    
    prn=U1(p);
    if (!(sat=satno(SYS_GPS,prn))) {
        trace(2,"SiRF satellite number error: prn=%d\n",prn);
        printf("SiRF satellite number error: prn=%d\n",prn);
        return -1;
    }

    trace(4,"decode_sirfeph: sat=%2d\n",sat);

    for (j=0;j<3;j++) memcpy(raw->subfrm[sat-1]+j*30,p+1+j*30,30);

    if (decode_frame(raw->subfrm[sat-1]   ,&eph,NULL,NULL,NULL,NULL)!=1||
            decode_frame(raw->subfrm[sat-1]+30,&eph,NULL,NULL,NULL,NULL)!=2||
            decode_frame(raw->subfrm[sat-1]+60,&eph,NULL,NULL,NULL,NULL)!=3) return 0;

    if (!strstr(raw->opt,"-EPHALL")) {
        if (eph.iode==raw->nav.eph[sat-1].iode) return 0; /* unchanged */
    }
    
    eph.sat=sat;
    raw->nav.eph[sat-1]=eph;
    raw->ephsat=sat;
    print_eph(eph);

    return 2;
}

/* decode MID 14 Almanac Data */
static int decode_sirfalm(raw_t *raw)
{
    int prn,sat,week,id,status; /* ,ckSum=0; */
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

/*    printf("\nDECODE_ALM_0x0E[%d]:id=%2d:sat=%2d:week=%d:status=%d:",raw->len-9-OffSet,id,sat,week,status); */
    memcpy(subfrm+1,D+3,24);
    subfrm[0]=4+16; 			/* subframe id#5 setzen! */
    memcpy(raw->subfrm[sat-1]+120+5,subfrm,25);
    raw->nav.alm[sat-1].week=week;	/* Almanac Woche setzen */

    decode_frame(raw->subfrm[sat-1]+120,NULL,raw->nav.alm,NULL,NULL,NULL); /* decode subframe #5 */
/*    print_alm(sat,raw->nav.alm); */
    return 0;
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
MID 64 0x32 D Navigation Library Messages -> TXCO Frequenz! HEAVY
MID 71 0x47 D Hardware Configuration Request ?? 1 Byte ...
MID 91 0x5B D Hardware Control Output VCTCXO on/off, AGC Gain Output -> MID 166 zum Ausschalten
MID 92 0x5C - CW Controller Output -> Interferenzen ...
MID 93 0x50 D TXCO Learning Output Response  ! Frequenz ! Wichtig !
MID225 0xE1 - Sub ID 6 = Statistics Channel (-> gefunden in OSP Info, Rev. 8!)
... Abschalten M166? NO geht nur mit 2, 4, 28, 30, 41, 255 */
static int decode_sirfgen(raw_t *raw)
{
    int i,prn,sat,id,j,w,sid,vis;
    int marray[24] = {MID_SRFCLOCK, MID_SRF50BPS, MID_SRFNLMEAS, MID_SRFALM, MID_SRFEPH, 71, 2, 4, 9, 13, 30, 31, 41, 50, 51, 56, 164, 91, 92, 93, 11, 18, 225 };
    /* 130,141,164 193 */
    unsigned gsw230=strstr(raw->opt,"-GSW230")!=NULL;
    
    unsigned char *message=raw->buff+4;
    int mid=U1(message);

    /* MID im Array enthalten? */
    for(i=0;i<24; i++) if (mid == marray[i]) return 0; 
    if (mid != 164) {
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
/*        printf("\nMID=64.%d:",sid); */
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
/*                printf("SVid:%2d;",U1(message+2));
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
                printf("SWTU:%4d;",U4(message+63)); */
                
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


/* Carrier frequency may be interpreted as the measured Doppler on the received signal.
   The value is reported in metres per second but can be converted to hertz using the
   Doppler equation:
   Doppler frequency / Carrier frequency = Velocity / Speed of light, 
   where Doppler frequency is in Hz; Carrier frequency = 1,575,420,000 Hz;
   Velocity is in m/s; Speed of light = 299,792,458 m/s.
   Note that the computed Doppler frequency contains a bias equal to the current clock
   drift as reported in Message ID 7. This bias, nominally 96.250 kHz, is equivalent to over 18 km/s. */
                

                break;
            case 3:	/* SID=3 Aiding Initialization 	*/
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
        for (i=1;i<raw->len-8;i++) printf("%02X ", U1(message+i) ); 
        printf("\n");
    }
    return 0;
}
/* decode SiRF _verified_ 50BPS message -------------------------------------------*/
static int decode_sirf50bpshq(raw_t *raw)
{
    unsigned char *p=raw->buff+4;
    if (U1(p+1) != 5)     return decode_sirfgen(raw);	/* Exit to generic decode */
    if (VerData == 0) {
        printf("\nVerified 50bps Data detected.");
        VerData=1;
    }
    trace(4,"decode_sirf50bpshq: len=%d\n",raw->len);
    
    memmove(raw->buff+4+1, raw->buff+4+2, raw->len-6);
    raw->len=(raw->len-1);
    return decode_sirf50bps(raw);
}

/* decode SiRF raw message --------------------------------------------
DECODE_SIRF:mid=14[  38]:A2001E0E2006C1601A98900980FD4D00A10CA475376F9936597721F50C009A093C
RAW:sat=32                      :2006C1601A98900980FD4D00A10CA475376F9936597721F5
*/
static int decode_sirf(raw_t *raw)
{
    unsigned char *p=raw->buff;
    int mid=U1(p+4);

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
    switch (mid) {
        case MID_SRFCLOCK:	return decode_sirfclock(raw);
        case MID_SRF50BPS:	return decode_sirf50bps(raw);
        case MID_SRFNLMEAS:	return decode_sirfnlmeas(raw);
        case MID_SRFEPH:	return decode_sirfeph(raw);
        case MID_SRFALM:	return decode_sirfalm(raw);
        case MID_SRF50BPSHQ:	return decode_sirf50bpshq(raw);
        case MID_SRFSVSTATE:	return decode_sirfsvstate(raw);
        case MID_SRFAUXDATA:	return decode_sirfauxdata(raw);
        case 4:			return decode_sirfmtdo(raw);
        case 6:			
            /* printf("SiRF Firmware = %s\n",p+7); 	/* Software Version String */
            fprintf(stderr,"%s [%s]\n",time_str(utc2gpst(timeget()),0),p+7);
            break;
        default: 	    	return decode_sirfgen(raw);
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

