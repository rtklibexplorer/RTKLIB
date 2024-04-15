/*------------------------------------------------------------------------------
* rtkpos.c : precise positioning
*
*          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
* history : 2007/01/12 1.0  new
*           2007/03/13 1.1  add slip detection by LLI flag
*           2007/04/18 1.2  add antenna pcv correction
*                           change rtkpos argin
*           2008/07/18 1.3  refactored
*           2009/01/02 1.4  modify rtk positioning api
*           2009/03/09 1.5  support glonass, gallileo and qzs
*           2009/08/27 1.6  fix bug on numerical exception
*           2009/09/03 1.7  add check of valid satellite number
*                           add check time sync for moving-base
*           2009/11/23 1.8  add api rtkopenstat(),rtkclosestat()
*                           add receiver h/w bias estimation
*                           add solution status output
*           2010/04/04 1.9  support ppp-kinematic and ppp-static modes
*                           support earth tide correction
*                           changed api:
*                               rtkpos()
*           2010/09/07 1.10 add elevation mask to hold ambiguity
*           2012/02/01 1.11 add extended receiver error model
*                           add glonass interchannel bias correction
*                           add slip detection by L1-L5 gf jump
*                           output snr of rover receiver in residuals
*           2013/03/10 1.12 add otl and pole tides corrections
*           2014/05/26 1.13 support beidou and galileo
*                           add output of gal-gps and bds-gps time offset
*           2014/05/28 1.14 fix bug on memory exception with many sys and freq
*           2014/08/26 1.15 add function to swap sol-stat file with keywords
*           2014/10/21 1.16 fix bug on beidou amb-res with pos2-bdsarmode=0
*           2014/11/08 1.17 fix bug on ar-degradation by unhealthy satellites
*           2015/03/23 1.18 residuals referenced to reference satellite
*           2015/05/20 1.19 no output solution status file with Q=0
*           2015/07/22 1.20 fix bug on base station position setting
*           2016/07/30 1.21 suppress single solution if !prcopt.outsingle
*                           fix bug on slip detection of backward filter
*           2016/08/20 1.22 fix bug on ddres() function
*           2018/10/10 1.13 support api change of satexclude()
*           2018/12/15 1.14 disable ambiguity resolution for gps-qzss
*           2019/08/19 1.15 fix bug on return value of resamb_LAMBDA()
*           2020/11/30 1.16 support of NavIC/IRNSS in API rtkpos()
*                           add detecting cycle slips by L1-Lx GF phase jump
*                           delete GLONASS IFB correction in ddres()
*                           use integer types in stdint.h
*-----------------------------------------------------------------------------*/
#include <stdarg.h>
#include "rtklib.h"

/* algorithm configuration -------------------------------------------------- */
#define STD_PREC_VAR_THRESH 0  /* pos variance threshold to skip standard precision */
                              /* solution: 0   = run every epoch, */
                              /*           0.5 = skip except for first*/

/* constants/macros ----------------------------------------------------------*/

#define SQR(x)      ((x)*(x))
#define SQRT(x)     ((x)<=0.0||(x)!=(x)?0.0:sqrt(x))
#define MIN(x,y)    ((x)<=(y)?(x):(y))
#define MAX(x,y)    ((x)>=(y)?(x):(y))
#define ROUND(x)    (int)floor((x)+0.5)

#define VAR_POS     SQR(30.0) /* initial variance of receiver pos (m^2) */
#define VAR_POS_FIX SQR(1e-4) /* initial variance of fixed receiver pos (m^2) */
#define VAR_VEL     SQR(10.0) /* initial variance of receiver vel ((m/s)^2) */
#define VAR_ACC     SQR(10.0) /* initial variance of receiver acc ((m/ss)^2) */
#define VAR_GRA     SQR(0.001) /* initial variance of gradient (m^2) */
#define INIT_ZWD    0.15     /* initial zwd (m) */

#define GAP_RESION  120      /* gap to reset ionosphere parameters (epochs) */

#define TTOL_MOVEB  (1.0+2*DTTOL)
                             /* time sync tolerance for moving-baseline (s) */

/* number of parameters (pos,ionos,tropos,hw-bias,phase-bias,real,estimated) */
#define NF(opt)     ((opt)->ionoopt==IONOOPT_IFLC?1:(opt)->nf)
#define NP(opt)     ((opt)->dynamics==0?3:9)
#define NI(opt)     ((opt)->ionoopt!=IONOOPT_EST?0:MAXSAT)
#define NT(opt)     ((opt)->tropopt<TROPOPT_EST?0:((opt)->tropopt<TROPOPT_ESTG?2:6))
#define NL(opt)     ((opt)->glomodear!=GLO_ARMODE_AUTOCAL?0:NFREQGLO)
#define NB(opt)     ((opt)->mode<=PMODE_DGPS?0:MAXSAT*NF(opt))
#define NR(opt)     (NP(opt)+NI(opt)+NT(opt)+NL(opt))
#define NX(opt)     (NR(opt)+NB(opt))

/* state variable index */
#define II(s,opt)   (NP(opt)+(s)-1)                 /* ionos (s:satellite no) */
#define IT(r,opt)   (NP(opt)+NI(opt)+NT(opt)/2*(r)) /* tropos (r:0=rov,1:ref) */
#define IL(f,opt)   (NP(opt)+NI(opt)+NT(opt)+(f))   /* receiver h/w bias */
#define IB(s,f,opt) (NR(opt)+MAXSAT*(f)+(s)-1) /* phase bias (s:satno,f:freq) */

/* poly coeffs used to adjust AR ratio by # of sats, derived by fitting to  example from:
   https://www.tudelft.nl/citg/over-faculteit/afdelingen/geoscience-remote-sensing/research/lambda/lambda */
static double ar_poly_coeffs[3][5] = {
    {-1.94058448e-01, -7.79023476e+00, 1.24231120e+02, -4.03126050e+02,  3.50413202e+02},
    {6.42237302e-01, -8.39813962e+00,  2.92107285e+01, -2.37577308e+01, -1.14307128e+00},
    {-2.22600390e-02,  3.23169103e-01, -1.39837429e+00, 2.19282996e+00, -5.34583971e-02}};

/* global variables ----------------------------------------------------------*/
static int statlevel=0;          /* rtk status output level (0:off) */
static FILE *fp_stat=NULL;       /* rtk status file pointer */
static char file_stat[1024]="";  /* rtk status file original path */
static gtime_t time_stat={0};    /* rtk status file time */

/* open solution status file ---------------------------------------------------
* open solution status file and set output level
* args   : char     *file   I   rtk status file
*          int      level   I   rtk status level (0: off)
* return : status (1:ok,0:error)
* notes  : file can constain time keywords (%Y,%y,%m...) defined in reppath().
*          The time to replace keywords is based on UTC of CPU time.
* output : solution status file record format
*
*   $POS,week,tow,stat,posx,posy,posz,posxf,posyf,poszf
*          week/tow : gps week no/time of week (s)
*          stat     : solution status
*          posx/posy/posz    : position x/y/z ecef (m) float
*          posxf/posyf/poszf : position x/y/z ecef (m) fixed
*
*   $VELACC,week,tow,stat,vele,veln,velu,acce,accn,accu,velef,velnf,veluf,accef,accnf,accuf
*          week/tow : gps week no/time of week (s)
*          stat     : solution status
*          vele/veln/velu    : velocity e/n/u (m/s) float
*          acce/accn/accu    : acceleration e/n/u (m/s^2) float
*          velef/velnf/veluf : velocity e/n/u (m/s) fixed
*          accef/accnf/accuf : acceleration e/n/u (m/s^2) fixed
*
*   $CLK,week,tow,stat,clk1,clk2,clk3,clk4,clk5,clk6
*          week/tow : gps week no/time of week (s)
*          stat     : solution status
*          clk1     : receiver clock bias GPS (ns)
*          clk2     : receiver clock bias GLO-GPS (ns)
*          clk3     : receiver clock bias GAL-GPS (ns)
*          clk4     : receiver clock bias BDS-GPS (ns)
*          clk5     : receiver clock bias IRN-GPS (ns)
*          clk6     : receiver clock bias QZS-GPS (ns)
*
*   $ION,week,tow,stat,sat,az,el,ion,ion-fixed
*          week/tow : gps week no/time of week (s)
*          stat     : solution status
*          sat      : satellite id
*          az/el    : azimuth/elevation angle(deg)
*          ion      : vertical ionospheric delay L1 (m) float
*          ion-fixed: vertical ionospheric delay L1 (m) fixed
*
*   $TROP,week,tow,stat,rcv,ztd,ztdf
*          week/tow : gps week no/time of week (s)
*          stat     : solution status
*          rcv      : receiver (1:rover,2:base station)
*          ztd      : zenith total delay (m) float
*          ztdf     : zenith total delay (m) fixed
*
*   $HWBIAS,week,tow,stat,frq,bias,biasf
*          week/tow : gps week no/time of week (s)
*          stat     : solution status
*          frq      : frequency (1:L1,2:L2,...)
*          bias     : h/w bias coefficient (m/MHz) float
*          biasf    : h/w bias coefficient (m/MHz) fixed
*
*   $SAT,week,tow,sat,frq,az,el,resp,resc,vsat,snr,fix,slip,lock,outc,slipc,rejc,icbias,bias,bias_var,lambda
*          week/tow : gps week no/time of week (s)
*          sat/frq  : satellite id/frequency (1:L1,2:L2,...)
*          az/el    : azimuth/elevation angle (deg)
*          resp     : pseudorange residual (m)
*          resc     : carrier-phase residual (m)
*          vsat     : valid data flag (0:invalid,1:valid)
*          snr      : signal strength (dbHz)
*          fix      : ambiguity flag  (0:no data,1:float,2:fixed,3:hold,4:ppp)
*          slip     : cycle-slip flag (bit1:slip,bit2:parity unknown)
*          lock     : carrier-lock count
*          outc     : data outage count
*          slipc    : cycle-slip count
*          rejc     : data reject (outlier) count
*          icbias   : interchannel bias (GLONASS)
*          bias     : phase bias
*          bias_var : variance of phase bias
*          lambda   : wavelength
*
*-----------------------------------------------------------------------------*/
extern int rtkopenstat(const char *file, int level)
{
    gtime_t time=utc2gpst(timeget());
    char path[1024];

    trace(3,"rtkopenstat: file=%s level=%d\n",file,level);

    if (level<=0) return 0;

    reppath(file,path,time,"","");

    if (!(fp_stat=fopen(path,"w"))) {
        trace(1,"rtkopenstat: file open error path=%s\n",path);
        return 0;
    }
    strcpy(file_stat,file);
    time_stat=time;
    statlevel=level;
    return 1;
}
/* close solution status file --------------------------------------------------
* close solution status file
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
extern void rtkclosestat(void)
{
    trace(3,"rtkclosestat:\n");

    if (fp_stat) fclose(fp_stat);
    fp_stat=NULL;
    file_stat[0]='\0';
    statlevel=0;
}
/* write solution status to buffer -------------------------------------------*/
extern int rtkoutstat(rtk_t *rtk, char *buff)
{
    ssat_t *ssat;
    double tow,pos[3],vel[3],acc[3],vela[3]={0},acca[3]={0},xa[3];
    int i,j,week,est,nfreq,nf=NF(&rtk->opt);
    char id[32],*p=buff;

    if (rtk->sol.stat==SOLQ_NONE) {
        return 0;
    }
    /* write ppp solution status to buffer */
    if (rtk->opt.mode>=PMODE_PPP_KINEMA) {
        return pppoutstat(rtk,buff);
    }
    est=rtk->opt.mode>=PMODE_DGPS;
    nfreq=est?nf:1;
    tow=time2gpst(rtk->sol.time,&week);

    /* receiver position */
    if (est) {
        for (i=0;i<3;i++) xa[i]=i<rtk->na?rtk->xa[i]:0.0;
        p+=sprintf(p,"$POS,%d,%.3f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",week,tow,
                   rtk->sol.stat,rtk->x[0],rtk->x[1],rtk->x[2],xa[0],xa[1],
                   xa[2]);
    }
    else {
        p+=sprintf(p,"$POS,%d,%.3f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",week,tow,
                   rtk->sol.stat,rtk->sol.rr[0],rtk->sol.rr[1],rtk->sol.rr[2],
                   0.0,0.0,0.0);
    }
    /* receiver velocity and acceleration */
    if (est&&rtk->opt.dynamics) {
        ecef2pos(rtk->sol.rr,pos);
        ecef2enu(pos,rtk->x+3,vel);
        ecef2enu(pos,rtk->x+6,acc);
        if (rtk->na>=6) ecef2enu(pos,rtk->xa+3,vela);
        if (rtk->na>=9) ecef2enu(pos,rtk->xa+6,acca);
        p+=sprintf(p,"$VELACC,%d,%.3f,%d,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f\n",
                   week,tow,rtk->sol.stat,vel[0],vel[1],vel[2],acc[0],acc[1],
                   acc[2],vela[0],vela[1],vela[2],acca[0],acca[1],acca[2]);
    }
    else {
        ecef2pos(rtk->sol.rr,pos);
        ecef2enu(pos,rtk->sol.rr+3,vel);
        p+=sprintf(p,"$VELACC,%d,%.3f,%d,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f\n",
                   week,tow,rtk->sol.stat,vel[0],vel[1],vel[2],
                   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
    }
    /* receiver clocks */
    p+=sprintf(p,"$CLK,%d,%.3f,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
               week,tow,rtk->sol.stat,1,rtk->sol.dtr[0]*1E9,rtk->sol.dtr[1]*1E9,
               rtk->sol.dtr[2]*1E9,rtk->sol.dtr[3]*1E9,
               rtk->sol.dtr[4]*1E9,rtk->sol.dtr[5]*1E9);

    /* ionospheric parameters */
    if (est&&rtk->opt.ionoopt==IONOOPT_EST) {
        for (i=0;i<MAXSAT;i++) {
            ssat=rtk->ssat+i;
            if (!ssat->vs) continue;
            satno2id(i+1,id);
            j=II(i+1,&rtk->opt);
            xa[0]=j<rtk->na?rtk->xa[j]:0.0;
            p+=sprintf(p,"$ION,%d,%.3f,%d,%s,%.1f,%.1f,%.4f,%.4f\n",week,tow,
                       rtk->sol.stat,id,ssat->azel[0]*R2D,ssat->azel[1]*R2D,
                       rtk->x[j],xa[0]);
        }
    }
    /* tropospheric parameters */
    if (est&&(rtk->opt.tropopt>=TROPOPT_EST)) {
        for (i=0;i<2;i++) {
            j=IT(i,&rtk->opt);
            xa[0]=j<rtk->na?rtk->xa[j]:0.0;
            p+=sprintf(p,"$TROP,%d,%.3f,%d,%d,%.4f,%.4f\n",week,tow,
                       rtk->sol.stat,i+1,rtk->x[j],xa[0]);
        }
    }
    /* receiver h/w bias */
    if (est&&rtk->opt.glomodear==GLO_ARMODE_AUTOCAL) {
        for (i=0;i<nfreq;i++) {
            j=IL(i,&rtk->opt);
            xa[0]=j<rtk->na?rtk->xa[j]:0.0;
            p+=sprintf(p,"$HWBIAS,%d,%.3f,%d,%d,%.4f,%.4f\n",week,tow,
                       rtk->sol.stat,i+1,rtk->x[j],xa[0]);
        }
    }
    return (int)(p-buff);
}
/* swap solution status file -------------------------------------------------*/
static void swapsolstat(void)
{
    gtime_t time=utc2gpst(timeget());
    char path[1024];

    if ((int)(time2gpst(time     ,NULL)/INT_SWAP_STAT)==
        (int)(time2gpst(time_stat,NULL)/INT_SWAP_STAT)) {
        return;
    }
    time_stat=time;

    if (!reppath(file_stat,path,time,"","")) {
        return;
    }
    if (fp_stat) fclose(fp_stat);

    if (!(fp_stat=fopen(path,"w"))) {
        trace(2,"swapsolstat: file open error path=%s\n",path);
        return;
    }
    trace(3,"swapsolstat: path=%s\n",path);
}
/* output solution status ----------------------------------------------------*/
static void outsolstat(rtk_t *rtk,const nav_t *nav)
{
    ssat_t *ssat;
    double tow;
    char buff[MAXSOLMSG+1],id[32];
    int i,j,k,n,week,nfreq,nf=NF(&rtk->opt);

    if (statlevel<=0||!fp_stat||!rtk->sol.stat) return;

    trace(3,"outsolstat:\n");

    /* swap solution status file */
    swapsolstat();

    /* write solution status */
    n=rtkoutstat(rtk,buff); buff[n]='\0';

    fputs(buff,fp_stat);

    if (rtk->sol.stat==SOLQ_NONE||statlevel<=1) return;

    tow=time2gpst(rtk->sol.time,&week);
    nfreq=rtk->opt.mode>=PMODE_DGPS?nf:1;

    /* write residuals and status */
    for (i=0;i<MAXSAT;i++) {
        ssat=rtk->ssat+i;
        if (!ssat->vs) continue;
        satno2id(i+1,id);
        for (j=0;j<nfreq;j++) {
            k=IB(i+1,j,&rtk->opt);
            fprintf(fp_stat,"$SAT,%d,%.3f,%s,%d,%.1f,%.1f,%.4f,%.4f,%d,%.0f,%d,%d,%d,%d,%d,%d,%.2f,%.6f,%.5f\n",
                    week,tow,id,j+1,ssat->azel[0]*R2D,ssat->azel[1]*R2D,
                    ssat->resp[j],ssat->resc[j],ssat->vsat[j],ssat->snr_rover[j]*SNR_UNIT,
                    ssat->fix[j],ssat->slip[j]&3,ssat->lock[j],ssat->outc[j],
                    ssat->slipc[j],ssat->rejc[j],k<rtk->nx?rtk->x[k]:0,
                    k<rtk->nx?rtk->P[k+k*rtk->nx]:0,ssat->icbias[j]);
        }
    }
}
/* save error message --------------------------------------------------------*/
static void errmsg(rtk_t *rtk, const char *format, ...)
{
    char buff[256],tstr[32];
    int n;
    va_list ap;
    time2str(rtk->sol.time,tstr,2);
    n=sprintf(buff,"%s: ",tstr+11);
    va_start(ap,format);
    n+=vsprintf(buff+n,format,ap);
    va_end(ap);
    n=n<MAXERRMSG-rtk->neb?n:MAXERRMSG-rtk->neb;
    memcpy(rtk->errbuf+rtk->neb,buff,n);
    rtk->neb+=n;
    trace(2,"%s",buff);
}
/* single-differenced observable ---------------------------------------------*/
static double sdobs(const obsd_t *obs, int i, int j, int k)
{
    double pi=(k<NFREQ)?obs[i].L[k]:obs[i].P[k-NFREQ];
    double pj=(k<NFREQ)?obs[j].L[k]:obs[j].P[k-NFREQ];
    return pi==0.0||pj==0.0?0.0:pi-pj;
}
/* single-differenced geometry-free linear combination of phase --------------*/
static double gfobs(const obsd_t *obs, int i, int j, int k, const nav_t *nav)
{
    double freq1,freq2,L1,L2;

    freq1=sat2freq(obs[i].sat,obs[i].code[0],nav);
    freq2=sat2freq(obs[i].sat,obs[i].code[k],nav);
    L1=sdobs(obs,i,j,0);
    L2=sdobs(obs,i,j,k);
    if (freq1==0.0||freq2==0.0||L1==0.0||L2==0.0) return 0.0;
    return L1*CLIGHT/freq1-L2*CLIGHT/freq2;
}
/* single-differenced measurement error variance -----------------------------*/
static double varerr(int sat, int sys, double el, double snr_rover, double snr_base,
                     double bl, double dt, int f, const prcopt_t *opt, const obsd_t *obs)
{
    double a,b,c,d,e;
    double snr_max=opt->err[5];
    double fact;
    double sinel=sin(el),var;
    int nf=NF(opt),frq,code;

    frq=f%nf;code=f<nf?0:1;
    /* increase variance for pseudoranges */
    if (code) fact=opt->eratio[frq];
    /* else adjust variance between freqs */
    else fact=opt->eratio[frq]/opt->eratio[0];

    /* adjust variance for constellation */
    switch (sys) {
        case SYS_GPS: fact*=EFACT_GPS;break;
        case SYS_GLO: fact*=EFACT_GLO;break;
        case SYS_GAL: fact*=EFACT_GAL;break;
        case SYS_SBS: fact*=EFACT_SBS;break;
        case SYS_QZS: fact*=EFACT_QZS;break;
        case SYS_CMP: fact*=EFACT_CMP;break;
        case SYS_IRN: fact*=EFACT_IRN;break;
        default:      fact*=EFACT_GPS;break;
    }
    /* adjust variance for config parameters */
    a=fact*opt->err[1];  /* base term */
    b=fact*opt->err[2];  /* el term */
    c=opt->err[3]*bl/1E4; /* baseline term */
    d=CLIGHT*opt->sclkstab*dt; /* clock term */
    /* calculate variance */
    var=2.0*(a*a+b*b/sinel/sinel+c*c)+d*d;
    if (opt->err[6]>0) {  /* add SNR term */
        e=fact*opt->err[6];
        var+=e*e*(pow(10,0.1*MAX(snr_max-snr_rover,0))+
                  pow(10,0.1*MAX(snr_max-snr_base, 0)));
    }
    if (opt->err[7]>0.0) {   /* add rcvr stdevs term */
        if (code) var+=SQR(opt->err[7]*0.01*(1<<(obs->Pstd[frq]+5))); /* 0.01*2^(n+5) */
        else var+=SQR(opt->err[7]*obs->Lstd[frq]*0.004*0.2); /* 0.004 cycles -> m) */
    }

    var*=(opt->ionoopt==IONOOPT_IFLC)?SQR(3.0):1.0;
    return var;
}
/* baseline length -----------------------------------------------------------*/
static double baseline(const double *ru, const double *rb, double *dr)
{
    int i;
    for (i=0;i<3;i++) dr[i]=ru[i]-rb[i];
    return norm(dr,3);
}
/* initialize state and covariance -------------------------------------------*/
static inline void initx(rtk_t *rtk, double xi, double var, int i)
{
    int j;
    rtk->x[i]=xi;
    for (j=0;j<rtk->nx;j++) rtk->P[i+j*rtk->nx]=0.0;
    for (j=0;j<rtk->nx;j++) rtk->P[j+i*rtk->nx]=0.0;
    rtk->P[i+i*rtk->nx]=var;
}
/* select common satellites between rover and reference station --------------*/
static int selsat(const obsd_t *obs, double *azel, int nu, int nr,
                  const prcopt_t *opt, int *sat, int *iu, int *ir)
{
    int i,j,k=0;

    trace(3,"selsat  : nu=%d nr=%d\n",nu,nr);

    for (i=0,j=nu;i<nu&&j<nu+nr;i++,j++) {
        if      (obs[i].sat<obs[j].sat) j--;
        else if (obs[i].sat>obs[j].sat) i--;
        else if (azel[1+j*2]>=opt->elmin) { /* elevation at base station */
            sat[k]=obs[i].sat; iu[k]=i; ir[k++]=j;
            trace(4,"(%2d) sat=%3d iu=%2d ir=%2d\n",k-1,obs[i].sat,i,j);
        }
    }
    return k;
}
/* temporal update of position/velocity/acceleration -------------------------*/
static void udpos(rtk_t *rtk, double tt)
{
    double *F,*P,*FP,*x,*xp,pos[3],Q[9]={0},Qv[9],var=0.0;
    int i,j,*ix,nx;

    trace(3,"udpos   : tt=%.3f\n",tt);

    /* fixed mode */
    if (rtk->opt.mode==PMODE_FIXED) {
        for (i=0;i<3;i++) initx(rtk,rtk->opt.ru[i],VAR_POS_FIX,i);
        return;
    }
    /* initialize position for first epoch */
    if (norm(rtk->x,3)<=0.0) {
        trace(3,"rr_init=");tracemat(3,rtk->sol.rr,1,6,15,6);
        for (i=0;i<3;i++) initx(rtk,rtk->sol.rr[i],VAR_POS,i);
        if (rtk->opt.dynamics) {
            for (i=3;i<6;i++) initx(rtk,rtk->sol.rr[i],VAR_VEL,i);
            for (i=6;i<9;i++) initx(rtk,1E-6,VAR_ACC,i);
        }
    }
    /* static mode */
    if (rtk->opt.mode==PMODE_STATIC||rtk->opt.mode==PMODE_STATIC_START) return;

    /* kinmatic mode without dynamics */
    if (!rtk->opt.dynamics) {
        for (i=0;i<3;i++) initx(rtk,rtk->sol.rr[i],VAR_POS,i);
        return;
    }
    /* check variance of estimated position */
    for (i=0;i<3;i++) var+=rtk->P[i+i*rtk->nx];
    var/=3.0;

    if (var>VAR_POS) {
        /* reset position with large variance */
        for (i=0;i<3;i++) initx(rtk,rtk->sol.rr[i],VAR_POS,i);
        for (i=3;i<6;i++) initx(rtk,rtk->sol.rr[i],VAR_VEL,i);
        for (i=6;i<9;i++) initx(rtk,1E-6,VAR_ACC,i);
        trace(2,"reset rtk position due to large variance: var=%.3f\n",var);
        return;
    }
    /* generate valid state index */
    ix=imat(rtk->nx,1);
    for (i=nx=0;i<rtk->nx;i++) {
         /*    TODO:  The b34 code causes issues so use b33 code for now */
        if (i<9||(rtk->x[i]!=0.0&&rtk->P[i+i*rtk->nx]>0.0)) ix[nx++]=i;
    }
    /* state transition of position/velocity/acceleration */
    F=eye(nx); P=mat(nx,nx); FP=mat(nx,nx); x=mat(nx,1); xp=mat(nx,1);

    for (i=0;i<6;i++) {
        F[i+(i+3)*nx]=tt;
    }
    /* include accel terms if filter is converged */
    if (var<rtk->opt.thresar[1]) {
        for (i=0;i<3;i++) {
            F[i+(i+6)*nx]=(tt>=0?1:-1)*SQR(tt)/2.0;
        }
    }
    else trace(3,"pos var too high for accel term: %.4f\n", var);
    for (i=0;i<nx;i++) {
        x[i]=rtk->x[ix[i]];
        for (j=0;j<nx;j++) {
            P[i+j*nx]=rtk->P[ix[i]+ix[j]*rtk->nx];
        }
    }
    /* x=F*x, P=F*P*F' */
    matmul("NN",nx,1,nx,F,x,xp);
    matmul("NN",nx,nx,nx,F,P,FP);
    matmul("NT",nx,nx,nx,FP,F,P);
    
    for (i=0;i<nx;i++) {
        rtk->x[ix[i]]=xp[i];
        for (j=0;j<nx;j++) {
            rtk->P[ix[i]+ix[j]*rtk->nx]=P[i+j*nx];
        }
    }
    /* process noise added to only acceleration  P=P+Q */
    Q[0]=Q[4]=SQR(rtk->opt.prn[3])*fabs(tt);
    Q[8]=SQR(rtk->opt.prn[4])*fabs(tt);
    ecef2pos(rtk->x,pos);
    covecef(pos,Q,Qv);
    for (i=0;i<3;i++) for (j=0;j<3;j++) {
        rtk->P[i+6+(j+6)*rtk->nx]+=Qv[i+j*3];
    }
    free(ix); free(F); free(P); free(FP); free(x); free(xp);
}
/* temporal update of ionospheric parameters ---------------------------------*/
static void udion(rtk_t *rtk, double tt, double bl, const int *sat, int ns)
{
    double el,fact;
    int i,j;

    trace(3,"udion   : tt=%.3f bl=%.0f ns=%d\n",tt,bl,ns);

    /* reset ionospheric delays for sats with long outages */
    for (i=1;i<=MAXSAT;i++) {
        j=II(i,&rtk->opt);
        if (rtk->x[j]!=0.0&&
            rtk->ssat[i-1].outc[0]>GAP_RESION&&rtk->ssat[i-1].outc[1]>GAP_RESION)
            rtk->x[j]=0.0;
    }
    for (i=0;i<ns;i++) {
        j=II(sat[i],&rtk->opt);

        if (rtk->x[j]==0.0) {
            /* initialize ionospheric delay state */
            initx(rtk,1E-6,SQR(rtk->opt.std[1]*bl/1E4),j);
        }
        else {
            /* elevation dependent factor of process noise */
            el=rtk->ssat[sat[i]-1].azel[1];
            fact=cos(el);
            rtk->P[j+j*rtk->nx]+=SQR(rtk->opt.prn[1]*bl/1E4*fact)*fabs(tt);
        }
    }
}
/* temporal update of tropospheric parameters --------------------------------*/
static void udtrop(rtk_t *rtk, double tt, double bl)
{
    int i,j,k;

    trace(3,"udtrop  : tt=%.3f\n",tt);

    for (i=0;i<2;i++) {
        j=IT(i,&rtk->opt);

        if (rtk->x[j]==0.0) {
            initx(rtk,INIT_ZWD,SQR(rtk->opt.std[2]),j); /* initial zwd */

            if (rtk->opt.tropopt>=TROPOPT_ESTG) {
                for (k=0;k<2;k++) initx(rtk,1E-6,VAR_GRA,++j);
            }
        }
        else {
            rtk->P[j+j*rtk->nx]+=SQR(rtk->opt.prn[2])*fabs(tt);

            if (rtk->opt.tropopt>=TROPOPT_ESTG) {
                for (k=0;k<2;k++) {
                    rtk->P[++j*(1+rtk->nx)]+=SQR(rtk->opt.prn[2]*0.3)*fabs(tt);
                }
            }
        }
    }
}
/* temporal update of receiver h/w biases ------------------------------------*/
static void udrcvbias(rtk_t *rtk, double tt)
{
    int i,j;

    trace(3,"udrcvbias: tt=%.3f\n",tt);

    for (i=0;i<NFREQGLO;i++) {
        j=IL(i,&rtk->opt);

        if (rtk->x[j]==0.0) {
            /* add small offset to avoid initializing with zero */
            initx(rtk,rtk->opt.thresar[2]+1e-6,rtk->opt.thresar[3],j);
        }
        /* hold to fixed solution */
        else if (rtk->nfix>=rtk->opt.minfix) {
            initx(rtk,rtk->xa[j],rtk->Pa[j+j*rtk->na],j);
        }
        else {
            rtk->P[j+j*rtk->nx]+=SQR(rtk->opt.thresar[4])*fabs(tt);
        }
    }
}
/* detect cycle slip by LLI --------------------------------------------------*/
static void detslp_ll(rtk_t *rtk, const obsd_t *obs, int i, int rcv)
{
    uint32_t slip,LLI;
    int f,sat=obs[i].sat;

    trace(4,"detslp_ll: i=%d rcv=%d\n",i,rcv);

    for (f=0;f<rtk->opt.nf;f++) {

        if ((obs[i].L[f]==0.0&&obs[i].LLI[f]==0)||
            fabs(timediff(obs[i].time,rtk->ssat[sat-1].pt[rcv-1][f]))<DTTOL) {
            continue;
        }
        /* restore previous LLI */
        if (rcv==1) LLI=getbitu(&rtk->ssat[sat-1].slip[f],0,2); /* rover */
        else        LLI=getbitu(&rtk->ssat[sat-1].slip[f],2,2); /* base  */

        /* detect slip by cycle slip flag in LLI */
        if (rtk->tt>=0.0) { /* forward */
            if (obs[i].LLI[f]&1) {
                errmsg(rtk,"slip detected forward (sat=%2d rcv=%d F=%d LLI=%x)\n",
                       sat,rcv,f+1,obs[i].LLI[f]);
            }
            slip=obs[i].LLI[f];
        }
        else { /* backward */
            if (LLI&1) {
                errmsg(rtk,"slip detected backward (sat=%2d rcv=%d F=%d LLI=%x)\n",
                       sat,rcv,f+1,LLI);
            }
            slip=LLI;
        }
        /* detect slip by parity unknown flag transition in LLI */
        if (((LLI&2)&&!(obs[i].LLI[f]&2))||(!(LLI&2)&&(obs[i].LLI[f]&2))) {
            errmsg(rtk,"slip detected half-cyc (sat=%2d rcv=%d F=%d LLI=%x->%x)\n",
                   sat,rcv,f+1,LLI,obs[i].LLI[f]);
            slip|=1;
        }
        /* save current LLI */
        if (rcv==1) setbitu(&rtk->ssat[sat-1].slip[f],0,2,obs[i].LLI[f]);
        else        setbitu(&rtk->ssat[sat-1].slip[f],2,2,obs[i].LLI[f]);

        /* save slip and half-cycle valid flag */
        rtk->ssat[sat-1].slip[f]|=(uint8_t)slip;
        rtk->ssat[sat-1].half[f]=(obs[i].LLI[f]&2)?0:1;
    }
}
/* detect cycle slip by geometry free phase jump -----------------------------*/
static void detslp_gf(rtk_t *rtk, const obsd_t *obs, int i, int j,
                           const nav_t *nav)
{
    int k,sat=obs[i].sat;
    double gf0,gf1;

    trace(4,"detslp_gf: i=%d j=%d\n",i,j);

    /* skip check if slip already detected or check disabled*/
    if (rtk->opt.thresslip==0) return;
    for (k=0;k<rtk->opt.nf;k++)
        if (rtk->ssat[sat-1].slip[k]&1) return;

    for (k=1;k<rtk->opt.nf;k++) {
        /* calc SD geomotry free LC of phase between freq0 and freqk */
        if ((gf1=gfobs(obs,i,j,k,nav))==0.0) continue;

        gf0=rtk->ssat[sat-1].gf[k-1];    /* retrieve previous gf */
        rtk->ssat[sat-1].gf[k-1]=gf1;    /* save current gf for next epoch */

        if (gf0!=0.0&&fabs(gf1-gf0)>rtk->opt.thresslip) {
            rtk->ssat[sat-1].slip[0]|=1;
            rtk->ssat[sat-1].slip[k]|=1;
            errmsg(rtk,"slip detected GF jump (sat=%2d L1-L%d dGF=%.3f)\n",
                sat,k+1,gf0-gf1);
        }
    }
}
/* detect cycle slip by doppler and phase difference -------------------------*/
static void detslp_dop(rtk_t *rtk, const obsd_t *obs, const int *ix, int ns,
                       int rcv, const nav_t *nav)
{
    int i,ii,f,sat,ndop=0;
    double dph,dpt,mean_dop=0;
    double dopdif[MAXSAT][NFREQ], tt[MAXSAT][NFREQ];

    trace(4,"detslp_dop: rcv=%d\n", rcv);
    if (rtk->opt.thresdop<=0) return;  /* skip test if doppler thresh <= 0 */

    /* calculate doppler differences for all sats and freqs */
    for (i=0;i<ns;i++) {
        ii = ix[i];
        sat=obs[ii].sat;

        for (f=0;f<rtk->opt.nf;f++) {
            dopdif[i][f]=0;tt[i][f]=0.00;
            if (obs[ii].L[f]==0.0||obs[ii].D[f]==0.0||rtk->ssat[sat-1].ph[rcv-1][f]==0.0) continue;
            if (fabs(tt[i][f]=timediff(obs[ii].time,rtk->ssat[sat-1].pt[rcv-1][f]))<DTTOL) continue;

            /* calc phase difference and doppler x time (cycle) */
            dph=(obs[ii].L[f]-rtk->ssat[sat-1].ph[rcv-1][f])/tt[i][f];
            dpt=-obs[ii].D[f];
            dopdif[i][f]=dph-dpt;

            /* if not outlier, use this to calculate mean */
            if (fabs(dopdif[i][f])<3*rtk->opt.thresdop) {
                mean_dop+=dopdif[i][f];
                ndop++;
            }
        }
    }
    /* calc mean doppler diff, most likely due to clock error */
    if (ndop==0) return;  /* unable to calc mean doppler, usually very large clock err */
    mean_dop=mean_dop/ndop;

    /* set slip if doppler difference with mean removed exceeds threshold */
    for (i=0;i<ns;i++) {
        sat=obs[ix[i]].sat;

        for (f=0;f<rtk->opt.nf;f++) {
            if (dopdif[i][f]==0.00) continue;
            if (fabs(dopdif[i][f]-mean_dop)>rtk->opt.thresdop) {
                rtk->ssat[sat-1].slip[f]|=1;
                errmsg(rtk,"slip detected doppler (sat=%2d rcv=%d dL%d=%.3f off=%.3f tt=%.2f)\n",
                   sat,rcv,f+1,dopdif[i][f]-mean_dop,mean_dop,tt[i][f]);
            }
        }
    }
}
/* temporal update of phase biases -------------------------------------------*/
static void udbias(rtk_t *rtk, double tt, const obsd_t *obs, const int *sat,
                   const int *iu, const int *ir, int ns, const nav_t *nav)
{
    double cp,pr,cp1,cp2,pr1,pr2,*bias,offset,freqi,freq1,freq2,C1,C2;
    int i,j,k,slip,rejc,reset,nf=NF(&rtk->opt),f2;

    trace(3,"udbias  : tt=%.3f ns=%d\n",tt,ns);

    /* clear cycle slips */
    for (i=0;i<ns;i++) {
        for (k=0;k<rtk->opt.nf;k++) rtk->ssat[sat[i]-1].slip[k]&=0xFC;
    }

    /* detect cycle slip by doppler and phase difference */
    detslp_dop(rtk,obs,iu,ns,1,nav);
    detslp_dop(rtk,obs,ir,ns,2,nav);

    for (i=0;i<ns;i++) {
        /* detect cycle slip by LLI */
        detslp_ll(rtk,obs,iu[i],1);
        detslp_ll(rtk,obs,ir[i],2);

        /* detect cycle slip by geometry-free phase jump */
        detslp_gf(rtk,obs,iu[i],ir[i],nav);

        /* update half-cycle valid flag */
        for (k=0;k<nf;k++) {
            rtk->ssat[sat[i]-1].half[k]=
                !((obs[iu[i]].LLI[k]&2)||(obs[ir[i]].LLI[k]&2));
        }
    }
    for (k=0;k<nf;k++) {
        /* reset phase-bias if instantaneous AR or expire obs outage counter */
        for (i=1;i<=MAXSAT;i++) {

            reset=++rtk->ssat[i-1].outc[k]>(uint32_t)rtk->opt.maxout;

            if (rtk->opt.modear==ARMODE_INST&&rtk->x[IB(i,k,&rtk->opt)]!=0.0) {
                initx(rtk,0.0,0.0,IB(i,k,&rtk->opt));
            }
            else if (reset&&rtk->x[IB(i,k,&rtk->opt)]!=0.0) {
                initx(rtk,0.0,0.0,IB(i,k,&rtk->opt));
                trace(3,"udbias : obs outage counter overflow (sat=%3d L%d n=%d)\n",
                      i,k+1,rtk->ssat[i-1].outc[k]);
                rtk->ssat[i-1].outc[k]=0;
            }
            if (rtk->opt.modear!=ARMODE_INST&&reset) {
                rtk->ssat[i-1].lock[k]=-rtk->opt.minlock;
            }
        }
        /* update phase bias noise and check for cycle slips */
        for (i=0;i<ns;i++) {
            j=IB(sat[i],k,&rtk->opt);
            rtk->P[j+j*rtk->nx]+=rtk->opt.prn[0]*rtk->opt.prn[0]*fabs(tt);
            slip=rtk->ssat[sat[i]-1].slip[k];
            rejc=rtk->ssat[sat[i]-1].rejc[k];
            if (rtk->opt.ionoopt==IONOOPT_IFLC) {
                f2=seliflc(rtk->opt.nf,rtk->ssat[sat[i]-1].sys);
                slip|=rtk->ssat[sat[i]-1].slip[f2];
            }
            if (rtk->opt.modear==ARMODE_INST||(!(slip&1)&&rejc<2)) continue;
            /* reset phase-bias state if detecting cycle slip or outlier */
            rtk->x[j]=0.0;
            rtk->ssat[sat[i]-1].rejc[k]=0;
            rtk->ssat[sat[i]-1].lock[k]=-rtk->opt.minlock;
            /* retain icbiases for GLONASS sats */
            if (rtk->ssat[sat[i]-1].sys!=SYS_GLO) rtk->ssat[sat[i]-1].icbias[k]=0;
        }
        bias=zeros(ns,1);

        /* estimate approximate phase-bias by delta phase - delta code */
        for (i=j=0,offset=0.0;i<ns;i++) {
            if (rtk->opt.ionoopt!=IONOOPT_IFLC) {
                /* phase diff between rover and base in cycles */
                cp=sdobs(obs,iu[i],ir[i],k); /* cycle */
                /* pseudorange diff between rover and base in meters */
                pr=sdobs(obs,iu[i],ir[i],k+NFREQ);
                freqi=sat2freq(sat[i],obs[iu[i]].code[k],nav);
                if (cp==0.0||pr==0.0||freqi==0.0) continue;
                /* estimate bias in cycles */
                bias[i]=cp-pr*freqi/CLIGHT;
            }
            else {  /* use ionosphere free calc with 2 freqs */
                f2=seliflc(rtk->opt.nf,rtk->ssat[sat[i]-1].sys);
                cp1=sdobs(obs,iu[i],ir[i],0);
                cp2=sdobs(obs,iu[i],ir[i],f2);
                pr1=sdobs(obs,iu[i],ir[i],NFREQ);
                pr2=sdobs(obs,iu[i],ir[i],NFREQ+f2);
                freq1=sat2freq(sat[i],obs[iu[i]].code[0],nav);
                freq2=sat2freq(sat[i],obs[iu[i]].code[f2],nav);
                if (cp1==0.0||cp2==0.0||pr1==0.0||pr2==0.0||freq1<=0.0||freq2<=0.0) continue;

                C1= SQR(freq1)/(SQR(freq1)-SQR(freq2));
                C2=-SQR(freq2)/(SQR(freq1)-SQR(freq2));
                /* estimate bias in meters */
                bias[i]=(C1*cp1*CLIGHT/freq1+C2*cp2*CLIGHT/freq2)-(C1*pr1+C2*pr2);
            }
            if (rtk->x[IB(sat[i],k,&rtk->opt)]!=0.0) {
                offset+=bias[i]-rtk->x[IB(sat[i],k,&rtk->opt)];
                j++;
            }
        }
        /* correct phase-bias offset to ensure phase-code coherency */
        if (j>0) {
            for (i=1;i<=MAXSAT;i++) {
                if (rtk->x[IB(i,k,&rtk->opt)]!=0.0) rtk->x[IB(i,k,&rtk->opt)]+=offset/j;
            }
        }
        /* set initial states of phase-bias */
        for (i=0;i<ns;i++) {
            if (bias[i]==0.0||rtk->x[IB(sat[i],k,&rtk->opt)]!=0.0) continue;
            initx(rtk,bias[i],SQR(rtk->opt.std[0]),IB(sat[i],k,&rtk->opt));
            trace(3,"     sat=%3d, F=%d: init phase=%.3f\n",sat[i],k+1,bias[i]);
            rtk->ssat[sat[i]-1].lock[k]=-rtk->opt.minlock;
        }
        free(bias);
    }
}
/* Temporal update of states --------------------------------------------------*/
static void udstate(rtk_t *rtk, const obsd_t *obs, const int *sat,
                    const int *iu, const int *ir, int ns, const nav_t *nav)
{
    trace(3,"udstate : ns=%d\n",ns);
    
    double tt=rtk->tt;

    /* Temporal update of position/velocity/acceleration */
    udpos(rtk,tt);

    /* Temporal update of ionospheric parameters */
    if (rtk->opt.ionoopt==IONOOPT_EST || rtk->opt.tropopt>=TROPOPT_EST) {
        double dr[3], bl=baseline(rtk->x,rtk->rb,dr);
        if (rtk->opt.ionoopt==IONOOPT_EST) {
            udion(rtk,tt,bl,sat,ns);
        }
        /* Temporal update of tropospheric parameters */
        if (rtk->opt.tropopt>=TROPOPT_EST) {
            udtrop(rtk,tt,bl);
        }
    }
    /* Temporal update of receiver h/w bias */
    if (rtk->opt.glomodear==GLO_ARMODE_AUTOCAL&&(rtk->opt.navsys&SYS_GLO)) {
        udrcvbias(rtk,tt);
    }
    /* Temporal update of phase-bias */
    if (rtk->opt.mode>PMODE_DGPS) {
        udbias(rtk,tt,obs,sat,iu,ir,ns,nav);
    }
}
/* UD (undifferenced) phase/code residual for satellite ----------------------*/
static void zdres_sat(int base, double r, const obsd_t *obs, const nav_t *nav,
                      const double *azel, const double *dant,
                      const prcopt_t *opt, double *y, double *freq)
{
    double freq1,freq2,C1,C2,dant_if;
    int i,nf=NF(opt),f2;

    if (opt->ionoopt==IONOOPT_IFLC) { /* iono-free linear combination */
        freq1=sat2freq(obs->sat,obs->code[0],nav);
        f2=seliflc(opt->nf,satsys(obs->sat,NULL));
        freq2=sat2freq(obs->sat,obs->code[f2],nav);

        if (freq1==0.0||freq2==0.0) return;

        if (testsnr(base,0,azel[1],obs->SNR[0]*SNR_UNIT,&opt->snrmask)||
            testsnr(base,f2,azel[1],obs->SNR[f2]*SNR_UNIT,&opt->snrmask)) return;

        C1= SQR(freq1)/(SQR(freq1)-SQR(freq2));
        C2=-SQR(freq2)/(SQR(freq1)-SQR(freq2));
        dant_if=C1*dant[0]+C2*dant[f2];

        if (obs->L[0]!=0.0&&obs->L[f2]!=0.0) {
            y[0]=C1*obs->L[0]*CLIGHT/freq1+C2*obs->L[f2]*CLIGHT/freq2-r-dant_if;
        }
        if (obs->P[0]!=0.0&&obs->P[f2]!=0.0) {
            y[nf]=C1*obs->P[0]+C2*obs->P[f2]-r-dant_if;
        }
        freq[0]=1.0;
    }
    else {
        for (i=0;i<nf;i++) {
            if ((freq[i]=sat2freq(obs->sat,obs->code[i],nav))==0.0) continue;

            /* check SNR mask */
            if (testsnr(base,i,azel[1],obs->SNR[i]*SNR_UNIT,&opt->snrmask)) {
                continue;
            }
            /* residuals = observable - estimated range */
            if (obs->L[i]!=0.0) y[i   ]=obs->L[i]*CLIGHT/freq[i]-r-dant[i];
            if (obs->P[i]!=0.0) y[i+nf]=obs->P[i]               -r-dant[i];
            trace(4,"zdres_sat: %d: L=%.6f P=%.6f r=%.6f f=%.0f\n",obs->sat,obs->L[i],
                obs->P[i],r,freq[i]);
        }
    }
}
/* undifferenced phase/code residuals ----------------------------------------
    calculate zero diff residuals [observed pseudorange - range]
        output is in y[0:nu-1], only shared input with base is nav
 args:  I   base:  1=base,0=rover
        I   obs  = sat observations
        I   n    = # of sats
        I   rs [(0:2)+i*6]= sat position {x,y,z} (m)
        I   dts[(0:1)+i*2]= sat clock {bias,drift} (s|s/s)
        I   var  = variance of ephemeris
        I   svh  = sat health flags
        I   nav  = sat nav data
        I   rr   = rcvr pos (x,y,z)
        I   opt  = options
        O   y[(0:1)+i*2] = zero diff residuals {phase,code} (m)
        O   e    = line of sight unit vectors to sats
        O   azel = [az, el] to sats                                           */
static int zdres(int base, const obsd_t *obs, int n, const double *rs,
                 const double *dts, const double *var, const int *svh,
                 const nav_t *nav, const double *rr, const prcopt_t *opt,
                 double *y, double *e, double *azel, double *freq)
{
    double r,rr_[3],pos[3],dant[NFREQ]={0},disp[3];
    double mapfh,zhd,zazel[]={0.0,90.0*D2R};
    int i,nf=NF(opt);

    trace(3,"zdres   : n=%d rr=%.2f %.2f %.2f\n",n,rr[0], rr[1], rr[2]);

    /* init residuals to zero */
    for (i=0;i<n*nf*2;i++) y[i]=0.0;

    if (norm(rr,3)<=0.0) return 0; /* no receiver position */

    /* rr_ = local copy of rcvr pos */
    for (i=0;i<3;i++) rr_[i]=rr[i];

    /* adjust rcvr pos for earth tide correction */
    if (opt->tidecorr) {
        tidedisp(gpst2utc(obs[0].time),rr_,opt->tidecorr,&nav->erp,
                 opt->odisp[base],disp);
        for (i=0;i<3;i++) rr_[i]+=disp[i];
    }
    /* translate rcvr pos from ecef to geodetic */
    ecef2pos(rr_,pos);

    /* loop through satellites */
    for (i=0;i<n;i++) {
        /* compute geometric-range and azimuth/elevation angle */
        if ((r=geodist(rs+i*6,rr_,e+i*3))<=0.0) continue;
        if (satazel(pos,e+i*3,azel+i*2)<opt->elmin) continue;

        /* excluded satellite? */
        if (satexclude(obs[i].sat,var[i],svh[i],opt)) continue;

        /* adjust range for satellite clock-bias */
        r+=-CLIGHT*dts[i*2];

        /* adjust range for troposphere delay model (hydrostatic) */
        zhd=tropmodel(obs[0].time,pos,zazel,0.0);
        mapfh=tropmapf(obs[i].time,pos,azel+i*2,NULL);
        r+=mapfh*zhd;

        /* calc receiver antenna phase center correction */
        antmodel(opt->pcvr+base,opt->antdel[base],azel+i*2,opt->posopt[1],
                 dant);

        /* calc undifferenced phase/code residual for satellite */
        trace(4,"sat=%d r=%.6f c*dts=%.6f zhd=%.6f map=%.6f\n",obs[i].sat,r,CLIGHT*dts[i*2],zhd,mapfh);
        zdres_sat(base,r,obs+i,nav,azel+i*2,dant,opt,y+i*nf*2,freq+i*nf);
    }
    trace(4,"rr_=%.3f %.3f %.3f\n",rr_[0],rr_[1],rr_[2]);
    trace(4,"pos=%.9f %.9f %.3f\n",pos[0]*R2D,pos[1]*R2D,pos[2]);
    for (i=0;i<n;i++) {
        if ((obs[i].L[0]==0&&obs[i].L[1]==0&&obs[i].L[2]==0)||base==0) continue;
        trace(3,"sat=%2d rs=%13.3f %13.3f %13.3f dts=%13.10f az=%6.1f el=%5.1f\n",
              obs[i].sat,rs[i*6],rs[1+i*6],rs[2+i*6],dts[i*2],azel[i*2]*R2D,
              azel[1+i*2]*R2D);
    }
    trace(3,"y=\n"); tracemat(3,y,nf*2,n,13,3);

    return 1;
}
/* test valid observation data -----------------------------------------------*/
static int validobs(int i, int j, int f, int nf, double *y)
{
    /* check for valid residuals */
    return y[f+i*nf*2]!=0.0&&y[f+j*nf*2]!=0.0;
}
/* double-differenced measurement error covariance ---------------------------
*
*   nb[n]:  # of sat pairs in group
*   n:      # of groups (2 for each system, phase and code)
*   Ri[nv]: variances of first sats in double diff pairs
*   Rj[nv]: variances of 2nd sats in double diff pairs
*   nv:     total # of sat pairs
*   R[nv][nv]:  double diff measurement err covariance matrix       */
static void ddcov(const int *nb, int n, const double *Ri, const double *Rj,
                  int nv, double *R)
{
    int i,j,k=0,b;

    trace(4,"ddcov   : n=%d\n",n);

    for (i=0;i<nv*nv;i++) R[i]=0.0;
    for (b=0;b<n;k+=nb[b++]) {  /* loop through each system */

        for (i=0;i<nb[b];i++) for (j=0;j<nb[b];j++) {
            R[k+i+(k+j)*nv]=Ri[k+i]+(i==j?Rj[k+i]:0.0);
        }
    }
    trace(5,"R=\n"); tracemat(5,R,nv,nv,8,6);
}
/* baseline length constraint ------------------------------------------------*/
static int constbl(rtk_t *rtk, const double *x, const double *P, double *v,
                   double *H, double *Ri, double *Rj, int index)
{
    const double thres=0.1; /* threshold for nonlinearity (v.2.3.0) */
    double xb[3],b[3],bb,var=0.0;
    int i;

    trace(4,"constbl : \n");

    /* time-adjusted baseline vector and length */
    for (i=0;i<3;i++) {
        xb[i]=rtk->rb[i];
        b[i]=x[i]-xb[i];
    }
    bb=norm(b,3);

    /* approximate variance of solution */
    if (P) {
        for (i=0;i<3;i++) var+=P[i+i*rtk->nx];
        var/=3.0;
    }
    /* check nonlinearity */
    if (var>SQR(thres*bb)) {
        trace(3,"constbl : pos variance large (bb=%.3f var=%.3f)\n",bb,var);
        /* return 0; */ /* threshold too strict for all use cases, report error but continue on */

    }
    /* constraint to baseline length */
    v[index]=rtk->opt.baseline[0]-bb;
    if (H) {
        for (i=0;i<3;i++) H[i+index*rtk->nx]=b[i]/bb;
    }
    Ri[index]=0.0;
    Rj[index]=SQR(rtk->opt.baseline[1]);

    trace(3,"constbl : baseline len   v=%13.3f R=%8.6f\n",v[index],Rj[index]);

    return 1;
}
/* precise tropospheric model -------------------------------------------------*/
static double prectrop(gtime_t time, const double *pos, int r,
                       const double *azel, const prcopt_t *opt, const double *x,
                       double *dtdx)
{
    double m_w=0.0,cotz,grad_n,grad_e;
    int i=IT(r,opt);

    /* wet mapping function */
    tropmapf(time,pos,azel,&m_w);

    if (opt->tropopt>=TROPOPT_ESTG&&azel[1]>0.0) {

        /* m_w=m_0+m_0*cot(el)*(Gn*cos(az)+Ge*sin(az)): ref [6] */
        cotz=1.0/tan(azel[1]);
        grad_n=m_w*cotz*cos(azel[0]);
        grad_e=m_w*cotz*sin(azel[0]);
        m_w+=grad_n*x[i+1]+grad_e*x[i+2];
        dtdx[1]=grad_n*x[i];
        dtdx[2]=grad_e*x[i];
    }
    else dtdx[1]=dtdx[2]=0.0;
    dtdx[0]=m_w;
    return m_w*x[i];
}
/* test satellite system (m=0:GPS/SBS,1:GLO,2:GAL,3:BDS,4:QZS,5:IRN) ---------*/
static int test_sys(int sys, int m)
{
    switch (sys) {
        case SYS_GPS: return m==0;
        case SYS_SBS: return m==0;
        case SYS_GLO: return m==1;
        case SYS_GAL: return m==2;
        case SYS_CMP: return m==3;
        case SYS_QZS: return m==4;
        case SYS_IRN: return m==5;
    }
    return 0;
}
/* double-differenced residuals and partial derivatives  -----------------------------------
        O rtk->ssat[i].resp[j] = residual pseudorange error
        O rtk->ssat[i].resc[j] = residual carrier phase error
        I rtk->rb= base location
        I dt = time diff between base and rover observations
        I x = rover pos & vel and sat phase biases (float solution)
        I P = error covariance matrix of float states
        I sat = list of common sats
        I y = zero diff residuals (code and phase, base and rover)
        I e = line of sight unit vectors to sats
        I azel = [az, el] to sats
        I iu,ir = user and ref indices to sats
        I ns = # of sats
        O v = double diff innovations (measurement-model) (phase and code)
        O H = linearized translation from innovations to states (az/el to sats)
        O R = measurement error covariances
        O vflg = bit encoded list of sats used for each double diff  */
static int ddres(rtk_t *rtk, const obsd_t *obs, double dt, const double *x,
                 const double *P, const int *sat, double *y, double *e,
                 double *azel, double *freq, const int *iu, const int *ir,
                 int ns, double *v, double *H, double *R, int *vflg)
{
    prcopt_t *opt=&rtk->opt;
    double bl,dr[3],posu[3],posr[3],didxi=0.0,didxj=0.0,*im,icb,threshadj;
    double *tropr,*tropu,*dtdxr,*dtdxu,*Ri,*Rj,freqi,freqj,*Hi=NULL,df;
    int i,j,k,m,f,nv=0,nb[NFREQ*NSYS*2+2]={0},b=0,sysi,sysj,nf=NF(opt);
    int ii,jj,frq,code;

    trace(3,"ddres   : dt=%.4f ns=%d\n",dt,ns);

    /* bl=distance from base to rover, dr=x,y,z components */
    bl=baseline(x,rtk->rb,dr);
    /* translate ecef pos to geodetic pos */
    ecef2pos(x,posu); ecef2pos(rtk->rb,posr);

    Ri=mat(ns*nf*2+2,1); Rj=mat(ns*nf*2+2,1); im=mat(ns,1);
    tropu=mat(ns,1); tropr=mat(ns,1); dtdxu=mat(ns,3); dtdxr=mat(ns,3);

    /* zero out residual phase and code biases for all satellites */
    for (i=0;i<MAXSAT;i++) for (j=0;j<NFREQ;j++) {
        rtk->ssat[i].resp[j]=rtk->ssat[i].resc[j]=0.0;
    }
    /* compute factors of ionospheric and tropospheric delay
           - only used if kalman filter contains states for ION and TROP delays
           usually insignificant for short baselines (<10km)*/
    for (i=0;i<ns;i++) {
        if (opt->ionoopt==IONOOPT_EST) {
            im[i]=(ionmapf(posu,azel+iu[i]*2)+ionmapf(posr,azel+ir[i]*2))/2.0;
        }
        if (opt->tropopt>=TROPOPT_EST) {
            tropu[i]=prectrop(rtk->sol.time,posu,0,azel+iu[i]*2,opt,x,dtdxu+i*3);
            tropr[i]=prectrop(rtk->sol.time,posr,1,azel+ir[i]*2,opt,x,dtdxr+i*3);
        }
    }
    /* step through sat systems: m=0:gps/sbs,1:glo,2:gal,3:bds 4:qzs 5:irn*/
    for (m=0;m<6;m++) {

        /* step through phases/codes */
        for (f=opt->mode>PMODE_DGPS?0:nf;f<nf*2;f++) {
            frq=f%nf;code=f<nf?0:1;

            /* find reference satellite with highest elevation, set to i */
            for (i=-1,j=0;j<ns;j++) {
                sysi=rtk->ssat[sat[j]-1].sys;
                if (!test_sys(sysi,m) || sysi==SYS_SBS) continue;
                if (!validobs(iu[j],ir[j],f,nf,y)) continue;
                /* skip sat with slip unless no other valid sat */
                if (i>=0&&rtk->ssat[sat[j]-1].slip[frq]&LLI_SLIP) continue;
                if (i<0||azel[1+iu[j]*2]>=azel[1+iu[i]*2]) i=j;
            }
            if (i<0) continue;

            /* calculate double differences of residuals (code/phase) for each sat */
            for (j=0;j<ns;j++) {
                if (i==j) continue;  /* skip ref sat */
                sysi=rtk->ssat[sat[i]-1].sys;
                sysj=rtk->ssat[sat[j]-1].sys;
                freqi=freq[frq+iu[i]*nf];
                freqj=freq[frq+iu[j]*nf];
                if (freqi<=0.0||freqj<=0.0) continue;
                if (!test_sys(sysj,m)) continue;
                if (!validobs(iu[j],ir[j],f,nf,y)) continue;

                if (H) {
                    Hi=H+nv*rtk->nx;
                    for (k=0;k<rtk->nx;k++) Hi[k]=0.0;
                }

                /* double-differenced measurements from 2 receivers and 2 sats in meters */
                v[nv]=(y[f+iu[i]*nf*2]-y[f+ir[i]*nf*2])-
                      (y[f+iu[j]*nf*2]-y[f+ir[j]*nf*2]);

                /* partial derivatives by rover position, combine unit vectors from two sats */
                if (H) {
                    for (k=0;k<3;k++) {
                        Hi[k]=-e[k+iu[i]*3]+e[k+iu[j]*3];  /* translation of innovation to position states */
                    }
                }
                if (opt->ionoopt==IONOOPT_EST) {
                    /* adjust double-differenced measurements by double-differenced ionospheric delay term */
                    didxi=(code?-1.0:1.0)*im[i]*SQR(FREQL1/freqi);
                    didxj=(code?-1.0:1.0)*im[j]*SQR(FREQL1/freqj);
                    v[nv]-=didxi*x[II(sat[i],opt)]-didxj*x[II(sat[j],opt)];
                    if (H) {
                        Hi[II(sat[i],opt)]= didxi;
                        Hi[II(sat[j],opt)]=-didxj;
                    }
                }
                if (opt->tropopt>=TROPOPT_EST) {
                    /* adjust double-differenced measurements by double-differenced tropospheric delay term */
                    v[nv]-=(tropu[i]-tropu[j])-(tropr[i]-tropr[j]);
                    for (k=0;k<(opt->tropopt<TROPOPT_ESTG?1:3);k++) {
                        if (!H) continue;
                        Hi[IT(0,opt)+k]= (dtdxu[k+i*3]-dtdxu[k+j*3]);
                        Hi[IT(1,opt)+k]=-(dtdxr[k+i*3]-dtdxr[k+j*3]);
                    }
                }
                ii=IB(sat[i],frq,opt);
                jj=IB(sat[j],frq,opt);
                if (!code) {
                    /* adjust phase residual by double-differenced phase-bias term,
                          IB=look up index by sat&freq */
                    if (opt->ionoopt!=IONOOPT_IFLC) {
                        /* phase-bias states are single-differenced so need to difference them */
                        v[nv]-=CLIGHT/freqi*x[ii]-CLIGHT/freqj*x[jj];
                        if (H) {
                        Hi[ii]= CLIGHT/freqi;
                        Hi[jj]=-CLIGHT/freqj;
                        }
                    }
                    else {
                        v[nv]-=x[ii]-x[jj];
                        if (H) {
                            Hi[ii]= 1.0;
                            Hi[jj]=-1.0;
                        }
                    }
                }


                /* adjust double-difference for glonass sats */
                if (sysi==SYS_GLO&&sysj==SYS_GLO) {
                    if (rtk->opt.glomodear==GLO_ARMODE_AUTOCAL && frq<NFREQGLO) {
                        /* auto-cal method */
                        df=(freqi-freqj)/(f==0?DFRQ1_GLO:DFRQ2_GLO);
                        v[nv]-=df*x[IL(frq,opt)];
                        if (H) Hi[IL(frq,opt)]=df;
                    }
                    else if (rtk->opt.glomodear==GLO_ARMODE_FIXHOLD && frq<NFREQGLO) {
                        /* fix-and-hold method */
                        icb=rtk->ssat[sat[i]-1].icbias[frq]*CLIGHT/freqi - rtk->ssat[sat[j]-1].icbias[frq]*CLIGHT/freqj;
                        v[nv]-=icb;
                    }
                }

                /* adjust double-difference for sbas sats */
                if (sysj==SYS_SBS&&sysi==SYS_GPS) {
                    if (rtk->opt.glomodear==GLO_ARMODE_FIXHOLD && frq<NFREQ) {
                        /* fix-and-hold method */
                        icb=rtk->ssat[sat[i]-1].icbias[frq]*CLIGHT/freqi - rtk->ssat[sat[j]-1].icbias[frq]*CLIGHT/freqj;
                        v[nv]-=icb;
                    }
                }

                /* save residuals */
                if (code) rtk->ssat[sat[j]-1].resp[frq]=v[nv];  /* pseudorange */
                else      rtk->ssat[sat[j]-1].resc[frq]=v[nv];  /* carrier phase */

                /* open up outlier threshold if one of the phase biases was just initialized */
                threshadj=(P[ii+rtk->nx*ii]==SQR(rtk->opt.std[0]))||
                    (P[jj+rtk->nx*jj]==SQR(rtk->opt.std[0]))?10:1;
                /* if residual too large, flag as outlier */
                if (fabs(v[nv])>opt->maxinno[code]*threshadj) {
                    rtk->ssat[sat[j]-1].vsat[frq]=0;
                    rtk->ssat[sat[j]-1].rejc[frq]++;
                    errmsg(rtk,"outlier rejected (sat=%3d-%3d %s%d v=%.3f)\n",
                            sat[i],sat[j],code?"P":"L",frq+1,v[nv]);
                    continue;
                }

                /* single-differenced measurement error variances (m) */
                Ri[nv] = varerr(sat[i], sysi, azel[1+iu[i]*2],
                                SNR_UNIT*rtk->ssat[sat[i]-1].snr_rover[frq],
                                SNR_UNIT*rtk->ssat[sat[i]-1].snr_base[frq],
                                bl,dt,f,opt,&obs[iu[i]]);
                Rj[nv] = varerr(sat[j], sysj, azel[1+iu[j]*2],
                                SNR_UNIT*rtk->ssat[sat[j]-1].snr_rover[frq],
                                SNR_UNIT*rtk->ssat[sat[j]-1].snr_base[frq],
                                bl,dt,f,opt,&obs[iu[j]]);
                /* increase variance if half cycle flags set */
                if (!code&&(obs[iu[i]].LLI[frq]&LLI_HALFC)) Ri[nv]+=0.01;
                if (!code&&(obs[iu[j]].LLI[frq]&LLI_HALFC)) Rj[nv]+=0.01;

                /* set valid data flags */
                if (opt->mode>PMODE_DGPS) {
                    if (!code) rtk->ssat[sat[i]-1].vsat[frq]=rtk->ssat[sat[j]-1].vsat[frq]=1;
                }
                else {
                    rtk->ssat[sat[i]-1].vsat[frq]=rtk->ssat[sat[j]-1].vsat[frq]=1;
                }

                if (rtk->opt.glomodear==GLO_ARMODE_AUTOCAL)
                    icb=x[IL(frq,opt)];
                else
                    icb=rtk->ssat[sat[i]-1].icbias[frq]*CLIGHT/freqi -
                        rtk->ssat[sat[j]-1].icbias[frq]*CLIGHT/freqj;
                jj=IB(sat[j],frq,&rtk->opt);
                trace(3,"sat=%3d-%3d %s%d v=%13.3f R=%9.6f %9.6f icb=%9.3f lock=%5d x=%9.3f P=%.3f\n",
                        sat[i],sat[j],code?"P":"L",frq+1,v[nv],Ri[nv],Rj[nv],icb,
                        rtk->ssat[sat[j]-1].lock[frq],x[jj],P[jj+jj*rtk->nx]);

                vflg[nv++]=(sat[i]<<16)|(sat[j]<<8)|((code?1:0)<<4)|(frq);
                nb[b]++;
            }
            b++;
        }
    }  /* end of system loop */

    /* baseline length constraint, for fixed distance between base and rover */
    if (rtk->opt.baseline[0]>0.0&&constbl(rtk,x,P,v,H,Ri,Rj,nv)) {
        vflg[nv++]=3<<4;
        nb[b++]++;
    }
    if (H) {trace(5,"H=\n"); tracemat(5,H,rtk->nx,nv,7,4);}

    /* double-differenced measurement error covariance */
    ddcov(nb,b,Ri,Rj,nv,R);

    free(Ri); free(Rj); free(im);
    free(tropu); free(tropr); free(dtdxu); free(dtdxr);

    return nv;
}
/* time-interpolation of residuals (for post-processing solutions) -----------
        time = rover time stamp
        obs = pointer to first base observation for this epoch
        y = pointer to base obs errors */
static double intpres(gtime_t time, const obsd_t *obs, int n, const nav_t *nav,
                      rtk_t *rtk, double *y)
{
    static obsd_t obsb[MAXOBS];
    static double yb[MAXOBS*NFREQ*2],rs[MAXOBS*6],dts[MAXOBS*2],var[MAXOBS];
    static double e[MAXOBS*3],azel[MAXOBS*2],freq[MAXOBS*NFREQ];
    static int nb=0,svh[MAXOBS*2];
    prcopt_t *opt=&rtk->opt;
    double tt,ttb,*p,*q;
    int i,j,k,nf=NF(opt);

    tt=timediff(time,obs[0].time); /* time delta between rover obs and current base obs */
    trace(3,"intpres : n=%d tt=%.1f, epoch=%d\n",n,tt,rtk->epoch);
    /* use current base obs if first epoch or delta time between rover obs and
       current base obs very small */
    if (nb==0||rtk->epoch==0||fabs(tt)<DTTOL) {
        nb=n; for (i=0;i<n;i++) obsb[i]=obs[i];  /* current base obs -> previous base obs */
        return tt;
    }
    /* use current base obs if delta time between rover obs and previous base obs too large
       or same as between current base and rover */
    ttb=timediff(time,obsb[0].time); /* time delta between rover obs and previous base obs */

    if (fabs(ttb)>opt->maxtdiff*2.0||ttb==tt) return tt;

    /* calculate sat positions for previous base obs */
    satposs(time,obsb,nb,nav,opt->sateph,rs,dts,var,svh);

    /* calculate [measured pseudorange - range] for previous base obs */
    if (!zdres(1,obsb,nb,rs,dts,var,svh,nav,rtk->rb,opt,yb,e,azel,freq)) {
        return tt;
    }
    /* interpolate previous and current base obs */
    for (i=0;i<n;i++) {
        /* align previous sat to current sat */
        for (j=0;j<nb;j++) if (obsb[j].sat==obs[i].sat) break;
        if (j>=nb) continue;
        /* p=ptr to current obs error, q=ptr to prev obs error,
           tt = delta time between rover and current base obs,
           ttb = delta time between rover and previous base obs */
        for (k=0,p=y+i*nf*2,q=yb+j*nf*2;k<nf*2;k++,p++,q++) {
            if (*p==0.0||*q==0.0||(obs[i].LLI[k%nf]&LLI_SLIP)||(obsb[j].LLI[k%nf]&LLI_SLIP))
               *p=0.0;
            else
                /* calculate interpolated values */
               *p=(ttb*(*p)-tt*(*q))/(ttb-tt);
        }
    }
    return fabs(ttb)<fabs(tt)?ttb:tt;
}
/* index for single to double-difference transformation matrix (D') --------------------*/
static int ddidx(rtk_t *rtk, int *ix, int gps, int glo, int sbs)
{
    int i,j,k,m,f,n,nb=0,na=rtk->na,nf=NF(&rtk->opt),nofix;
    double fix[MAXSAT],ref[MAXSAT];

    trace(3,"ddidx: gps=%d/%d glo=%d/%d sbs=%d\n",gps,rtk->opt.gpsmodear,glo,rtk->opt.glomodear,sbs);

    /* clear fix flag for all sats (1=float, 2=fix) */
    for (i=0;i<MAXSAT;i++) for (j=0;j<NFREQ;j++) {
        rtk->ssat[i].fix[j]=0;
    }
    for (m=0;m<6;m++) { /* m=0:GPS/SBS,1:GLO,2:GAL,3:BDS,4:QZS,5:IRN */

        /* skip if ambiguity resolution turned off for this sys */
        nofix=(m==0&&gps==0)||(m==1&&glo==0)||(m==3&&rtk->opt.bdsmodear==0);

        /* step through freqs */
        for (f=0,k=na;f<nf;f++,k+=MAXSAT) {

            /* look for first valid sat (i=state index, i-k=sat index) */
            for (i=k;i<k+MAXSAT;i++) {
                /* skip if sat not active */
                if (rtk->x[i]==0.0||!test_sys(rtk->ssat[i-k].sys,m)||
                    !rtk->ssat[i-k].vsat[f]) {
                    continue;
                }
                /* set sat to use for fixing ambiguity if meets criteria */
                if (rtk->ssat[i-k].lock[f]>=0&&!(rtk->ssat[i-k].slip[f]&2)&&
                    rtk->ssat[i-k].azel[1]>=rtk->opt.elmaskar&&!nofix) {
                    rtk->ssat[i-k].fix[f]=2; /* fix */
                    break;/* break out of loop if find good sat */
                }
                /* else don't use this sat for fixing ambiguity */
                else rtk->ssat[i-k].fix[f]=1;
            }
            if (i>=k+MAXSAT||rtk->ssat[i-k].fix[f]!=2) continue;  /* no good sat found */
            /* step through all sats (j=state index, j-k=sat index, i-k=first good sat) */
            for (n=0,j=k;j<k+MAXSAT;j++) {
                if (i==j||rtk->x[j]==0.0||!test_sys(rtk->ssat[j-k].sys,m)||
                    !rtk->ssat[j-k].vsat[f]) {
                    continue;
                }
                if (sbs==0 && satsys(j-k+1,NULL)==SYS_SBS) continue;
                if (rtk->ssat[j-k].lock[f]>=0&&!(rtk->ssat[j-k].slip[f]&2)&&
                    rtk->ssat[j-k].vsat[f]&&
                    rtk->ssat[j-k].azel[1]>=rtk->opt.elmaskar&&!nofix) {
                    /* set D coeffs to subtract sat j from sat i */
                    ix[nb*2  ]=i; /* state index of ref bias */
                    ix[nb*2+1]=j; /* state index of target bias */
                    /* inc # of sats used for fix */
                    ref[nb]=i-k+1;
                    fix[nb++]=j-k+1;
                    rtk->ssat[j-k].fix[f]=2; /* fix */
                    n++; /* count # of sat pairs for this freq/constellation */
                }
                /* else don't use this sat for fixing ambiguity */
                else rtk->ssat[j-k].fix[f]=1;
            }
            /* don't use ref sat if no sat pairs */
            if (n==0) rtk->ssat[i-k].fix[f]=1;
        }
    }

    if (nb>0) {
        trace(3,"refSats=");tracemat(3,ref,1,nb,7,0);
        trace(3,"fixSats=");tracemat(3,fix,1,nb,7,0);
    }
    return nb;
}
/* translate double diff fixed phase-bias values to single diff fix phase-bias values */
static void restamb(rtk_t *rtk, const double *bias, int nb, double *xa)
{
    int i,n,m,f,index[MAXSAT]={0},nv=0,nf=NF(&rtk->opt);

    trace(3,"restamb :\n");

    for (i=0;i<rtk->nx;i++) xa[i]=rtk->x [i];  /* init all fixed states to float state values */
    for (i=0;i<rtk->na;i++) xa[i]=rtk->xa[i];  /* overwrite non phase-bias states with fixed values */

    for (m=0;m<6;m++) for (f=0;f<nf;f++) {

        for (n=i=0;i<MAXSAT;i++) {
            if (!test_sys(rtk->ssat[i].sys,m)||rtk->ssat[i].fix[f]!=2) {
                continue;
            }
            index[n++]=IB(i+1,f,&rtk->opt);
        }
        if (n<2) continue;

        xa[index[0]]=rtk->x[index[0]];

        for (i=1;i<n;i++) {
            xa[index[i]]=xa[index[0]]-bias[nv++];
        }
    }
}
/* hold integer ambiguity ----------------------------------------------------*/
static void holdamb(rtk_t *rtk, const double *xa)
{
    double *v,*H,*R;
    int i,j,n,m,f,info,index[MAXSAT],nb=rtk->nx-rtk->na,nv=0,nf=NF(&rtk->opt);
    double dd;
    
    trace(3,"holdamb :\n");

    v=mat(nb,1); H=zeros(nb,rtk->nx);

    for (m=0;m<6;m++) for (f=0;f<nf;f++) {

        for (n=i=0;i<MAXSAT;i++) {
            if (!test_sys(rtk->ssat[i].sys,m)||rtk->ssat[i].fix[f]!=2||
                rtk->ssat[i].azel[1]<rtk->opt.elmaskhold) {
                continue;
            }
            index[n++]=IB(i+1,f,&rtk->opt);
            rtk->ssat[i].fix[f]=3; /* hold */
        }
        /* use ambiguity resolution results to generate a set of pseudo-innovations
                to feed to kalman filter based on error between fixed and float solutions */
        for (i=1;i<n;i++) {
            /* phase-biases are single diff, so subtract errors to get
                 double diff: v(nv)=err(i)-err(0) */
            v[nv]=(xa[index[0]]-xa[index[i]])-(rtk->x[index[0]]-rtk->x[index[i]]);

            H[index[0]+nv*rtk->nx]= 1.0;
            H[index[i]+nv*rtk->nx]=-1.0;
            nv++;
        }
    }
    /* return if less than min sats for hold (skip if fix&hold for GLONASS only) */
    if (rtk->opt.modear==ARMODE_FIXHOLD&&nv<rtk->opt.minholdsats) {
        trace(3,"holdamb: not enough sats to hold ambiguity\n");
        free(v); free(H);
        return;
    }

    rtk->holdamb=1;  /* set flag to indicate hold has occurred */
    R=zeros(nv,nv);
    for (i=0;i<nv;i++) R[i+i*nv]=rtk->opt.varholdamb;

    /* update states with constraints */
    if ((info=filter(rtk->x,rtk->P,H,v,R,rtk->nx,nv))) {
        errmsg(rtk,"filter error (info=%d)\n",info);
    }
    free(R);free(v); free(H);

    /* skip glonass/sbs icbias update if not enabled  */
    if (rtk->opt.glomodear!=GLO_ARMODE_FIXHOLD) return;

    /* Move fractional part of bias from phase-bias into ic bias for GLONASS sats (both in cycles) */
    for (f=0;f<nf;f++) {
        i=-1;
        for (j=nv=0;j<MAXSAT;j++) {
            /* check if valid GLONASS sat */
            if (test_sys(rtk->ssat[j].sys,1)&&rtk->ssat[j].vsat[f]&&rtk->ssat[j].lock[f]>=0) {
                if (i<0) {
                    i=j;  /* use first valid sat for reference sat */
                    index[nv++]=j;
                }
                else {  /* adjust the rest */
                    /* find phase-bias difference */
                    dd=rtk->x[IB(j+1,f,&rtk->opt)]-rtk->x[IB(i+1,f,&rtk->opt)];
                    dd=rtk->opt.gainholdamb*(dd-ROUND(dd));  /* throwout integer part of answer and multiply by filter gain */
                    rtk->x[IB(j+1,f,&rtk->opt)]-=dd;  /* remove fractional part from phase bias */
                    rtk->ssat[j].icbias[f]+=dd;       /* and move to IC bias */
                    index[nv++]=j;
                }
            }
        }
    }
    /* Move fractional part of bias from phase-bias into ic bias for SBAS sats (both in cycles) */
    for (f=0;f<nf;f++) {
        i=-1;
        for (j=nv=0;j<MAXSAT;j++) {
            /* check if valid GPS/SBS sat */
            if (test_sys(rtk->ssat[j].sys,0)&&rtk->ssat[j].vsat[f]&&rtk->ssat[j].lock[f]>=0) {
                if (i<0) {
                    i=j;  /* use first valid GPS sat for reference sat */
                    index[nv++]=j;
                }
                else {  /* adjust the SBS sats */
                    if (rtk->ssat[j].sys!=SYS_SBS) continue;
                    /* find phase-bias difference */
                    dd=rtk->x[IB(j+1,f,&rtk->opt)]-rtk->x[IB(i+1,f,&rtk->opt)];
                    dd=rtk->opt.gainholdamb*(dd-ROUND(dd));  /* throwout integer part of answer and multiply by filter gain */
                    rtk->x[IB(j+1,f,&rtk->opt)]-=dd;  /* remove fractional part from phase bias diff */
                    rtk->ssat[j].icbias[f]+=dd;       /* and move to IC bias */
                    index[nv++]=j;
                }
            }
        }
    }
}
/* resolve integer ambiguity by LAMBDA ---------------------------------------*/
static int resamb_LAMBDA(rtk_t *rtk, double *bias, double *xa,int gps,int glo,int sbs)
{
    prcopt_t *opt=&rtk->opt;
    int i,j,nb,nb1,info,nx=rtk->nx,na=rtk->na;
    double *DP,*y,*b,*db,*Qb,*Qab,*QQ,s[2];
    int *ix;
    double coeff[3];
    double QQb[MAXSAT];

    trace(3,"resamb_LAMBDA : nx=%d\n",nx);

    rtk->sol.ratio=0.0;
    rtk->nb_ar=0;
    /* Create index of single to double-difference transformation matrix (D')
          used to translate phase biases to double difference */
    ix=imat(nx,2);
    if ((nb=ddidx(rtk,ix,gps,glo,sbs))<(rtk->opt.minfixsats-1)) {  /* nb is sat pairs */
        errmsg(rtk,"not enough valid double-differences\n");
        free(ix);
        return -1; /* flag abort */
    }
    rtk->nb_ar=nb;
    /* nx=# of float states, na=# of fixed states, nb=# of double-diff phase biases */
    y=mat(nb,1); DP=mat(nb,nx-na); b=mat(nb,2); db=mat(nb,1); Qb=mat(nb,nb);
    Qab=mat(na,nb); QQ=mat(na,nb);

    /* phase-bias covariance (Qb) and real-parameters to bias covariance (Qab) */
    /* y=D*xc, Qb=D*Qc*D', Qab=Qac*D' */
    for (i=0;i<nb;i++) {
        y[i]=rtk->x[ix[i*2]]-rtk->x[ix[i*2+1]];
    }
    for (j=0;j<nx-na;j++) for (i=0;i<nb;i++) {
        DP[i+j*nb]=rtk->P[ix[i*2]+(na+j)*nx]-rtk->P[ix[i*2+1]+(na+j)*nx];
    }
    for (j=0;j<nb;j++) for (i=0;i<nb;i++) {
        Qb[i+j*nb]=DP[i+(ix[j*2]-na)*nb]-DP[i+(ix[j*2+1]-na)*nb];
    }
    for (j=0;j<nb;j++) for (i=0;i<na;i++) {
        Qab[i+j*na]=rtk->P[i+ix[j*2]*nx]-rtk->P[i+ix[j*2+1]*nx];
    }
    for (i=0;i<nb;i++) QQb[i]=1000*Qb[i+i*nb];

    trace(3,"N(0)=     "); tracemat(3,y,1,nb,7,2);
    trace(3,"Qb*1000=  "); tracemat(3,QQb,1,nb,7,4);

    /* lambda/mlambda integer least-square estimation */
    /* return best integer solutions */
    /* b are best integer solutions, s are residuals */
    if (!(info=lambda(nb,2,y,Qb,b,s))) {
        trace(3,"N(1)=     "); tracemat(3,b   ,1,nb,7,2);
        trace(3,"N(2)=     "); tracemat(3,b+nb,1,nb,7,2);

        rtk->sol.ratio=s[0]>0?(float)(s[1]/s[0]):0.0f;
        if (rtk->sol.ratio>999.9) rtk->sol.ratio=999.9f;

        /* adjust AR ratio based on # of sats, unless minAR==maxAR */
        if (opt->thresar[5]!=opt->thresar[6]) {
            nb1=nb<50?nb:50; /* poly only fitted for upto 50 sat pairs */
            /* generate poly coeffs based on nominal AR ratio */
            for ((i=0);i<3;i++) {
                 coeff[i] = ar_poly_coeffs[i][0];
                 for ((j=1);j<5;j++)
                    coeff[i] = coeff[i]*opt->thresar[0]+ar_poly_coeffs[i][j];
            }
            /* generate adjusted AR ratio based on # of sat pairs */
            rtk->sol.thres = coeff[0];
            for (i=1;i<3;i++) {
                rtk->sol.thres = rtk->sol.thres*1/(nb1+1)+coeff[i];
            }
            rtk->sol.thres = MIN(MAX(rtk->sol.thres,opt->thresar[5]),opt->thresar[6]);
        } else
            rtk->sol.thres=(float)opt->thresar[0];
        /* validation by popular ratio-test of residuals*/
        if (s[0]<=0.0||s[1]/s[0]>=rtk->sol.thres) {

            /* init non phase-bias states and covariances with float solution values */
            /* transform float to fixed solution (xa=x-Qab*Qb\(b0-b)) */
            for (i=0;i<na;i++) {
                rtk->xa[i]=rtk->x[i];
                for (j=0;j<na;j++) rtk->Pa[i+j*na]=rtk->P[i+j*nx];
            }
            /* y = differences between float and fixed dd phase-biases
               bias = fixed dd phase-biases   */
            for (i=0;i<nb;i++) {
                bias[i]=b[i];
                y[i]-=b[i];
            }
            /* adjust non phase-bias states and covariances using fixed solution values */
            if (!matinv(Qb,nb)) {  /* returns 0 if inverse successful */
                /* rtk->xa = rtk->x-Qab*Qb^-1*(b0-b) */
                matmul("NN",nb,1,nb,Qb ,y,db); /* db = Qb^-1*(b0-b) */
                matmulm("NN",na,1,nb,Qab,db,rtk->xa); /* rtk->xa = rtk->x-Qab*db */
                
                /* rtk->Pa=rtk->P-Qab*Qb^-1*Qab') */
                /* covariance of fixed solution (Qa=Qa-Qab*Qb^-1*Qab') */
                matmul("NN",na,nb,nb,Qab,Qb ,QQ);  /* QQ = Qab*Qb^-1 */
                matmulm("NT",na,na,nb,QQ ,Qab,rtk->Pa); /* rtk->Pa = rtk->P-QQ*Qab' */
                
                trace(3,"resamb : validation ok (nb=%d ratio=%.2f thresh=%.2f s=%.2f/%.2f)\n",
                      nb,s[0]==0.0?0.0:s[1]/s[0],rtk->sol.thres,s[0],s[1]);

                /* translate double diff fixed phase-bias values to single diff
                fix phase-bias values, result in xa */
                restamb(rtk,bias,nb,xa);
            }
            else nb=0;
        }
        else { /* validation failed */
            errmsg(rtk,"ambiguity validation failed (nb=%d ratio=%.2f thresh=%.2f s=%.2f/%.2f)\n",
                   nb,s[1]/s[0],rtk->sol.thres,s[0],s[1]);
            nb=0;
        }
    }
    else {
        errmsg(rtk,"lambda error (info=%d)\n",info);
        nb=0;
    }
    free(ix);
    free(y); free(DP); free(b); free(db); free(Qb); free(Qab); free(QQ);

    return nb; /* number of ambiguities */
}

/* resolve integer ambiguity by LAMBDA using partial fix techniques and multiple attempts -----------------------*/
static int manage_amb_LAMBDA(rtk_t *rtk, double *bias, double *xa, const int *sat, int nf, int ns)
{
    int i,f,lockc[NFREQ],ar=0,excflag=0,arsats[MAXOBS]={0};
    int gps1=-1,glo1=-1,sbas1=-1,gps2,glo2,sbas2,nb,rerun,dly;
    float ratio1,posvar=0;

    /* calc position variance, will skip AR if too high to avoid false fix */
    for (i=0;i<3;i++) posvar+=rtk->P[i+i*rtk->nx];
    posvar/=3.0; /* maintain compatibility with previous code */

    trace(3,"posvar=%.6f\n",posvar);
    trace(3,"prevRatios= %.3f %.3f\n",rtk->sol.prev_ratio1,rtk->sol.prev_ratio2);
    trace(3,"num ambiguities used last AR: %d\n",rtk->nb_ar);

    /* skip AR if don't meet criteria */
    if (rtk->opt.mode<=PMODE_DGPS||rtk->opt.modear==ARMODE_OFF||
        rtk->opt.thresar[0]<1.0||posvar>rtk->opt.thresar[1]) {
        trace(3,"Skip AR\n");
        rtk->sol.ratio=0.0;
        rtk->sol.prev_ratio1=rtk->sol.prev_ratio2=0.0;
        rtk->nb_ar=0;
        return 0;
    }
    /* if no fix on previous sample and enough sats, exclude next sat in list */
    if (rtk->sol.prev_ratio2<rtk->sol.thres&&rtk->nb_ar>=rtk->opt.mindropsats) {
        /* find and count sats used last time for AR */
        for (f=0;f<nf;f++) for (i=0;i<ns;i++)
            if (rtk->ssat[sat[i]-1].vsat[f] && rtk->ssat[sat[i]-1].lock[f]>=0 && rtk->ssat[sat[i]-1].azel[1]>=rtk->opt.elmin) {
                arsats[ar++]=i;
            }
        if (rtk->excsat<ar) {
            i=sat[arsats[rtk->excsat]];
            for (f=0;f<nf;f++) {
                lockc[f]=rtk->ssat[i-1].lock[f];  /* save lock count */
                /* remove sat from AR long enough to enable hold if stays fixed */
                rtk->ssat[i-1].lock[f]=-rtk->nb_ar;
            }
            trace(3,"AR: exclude sat %d\n",i);
            excflag=1;
        } else rtk->excsat=0; /* exclude none and reset to beginning of list */
    }

    /* for inital ambiguity resolution attempt, include all enabled sats */
    gps1=1;    /* always enable gps for initial pass */
    glo1=(rtk->opt.navsys&SYS_GLO)?(((rtk->opt.glomodear==GLO_ARMODE_FIXHOLD)&&!rtk->holdamb)?0:1):0;
    sbas1=(rtk->opt.navsys&SYS_GLO)?glo1:((rtk->opt.navsys&SYS_SBS)?1:0);
    /* first attempt to resolve ambiguities */
    nb=resamb_LAMBDA(rtk,bias,xa,gps1,glo1,sbas1);
    ratio1=rtk->sol.ratio;
    /* reject bad satellites if AR filtering enabled */
    if (rtk->opt.arfilter) {
        rerun=0;
        /* if results are much poorer than previous epoch or dropped below ar ratio thresh, remove new sats */
        if (nb>=0 && rtk->sol.prev_ratio2>=rtk->sol.thres && ((rtk->sol.ratio<rtk->sol.thres) ||
            (rtk->sol.ratio<rtk->opt.thresar[0]*1.1 && rtk->sol.ratio<rtk->sol.prev_ratio1/2.0))) {
            trace(3,"low ratio: check for new sat\n");
            dly=2;
            for (i=0;i<ns;i++) for (f=0;f<nf;f++) {
                if (rtk->ssat[sat[i]-1].fix[f]!=2) continue;
                /* check for new sats */
                if (rtk->ssat[sat[i]-1].lock[f]==0) {
                    trace(3,"remove sat %d:%d lock=%d\n",sat[i],f,rtk->ssat[sat[i]-1].lock[f]);
                    rtk->ssat[sat[i]-1].lock[f]=-rtk->opt.minlock-dly;  /* delay use of this sat with stagger */
                    dly+=2;  /* stagger next try of new sats */
                    rerun=1;
                }
            }
        }
        /* rerun if filter removed any sats */
        if (rerun) {
            trace(3,"rerun AR with new sats removed\n");
            /* try again with new sats removed */
            nb=resamb_LAMBDA(rtk,bias,xa,gps1,glo1,sbas1);
        }
    }
    rtk->sol.prev_ratio1=ratio1;


    /* if fix-and-hold gloarmode enabled, re-run AR with final gps/glo settings if differ from above */
    if ((rtk->opt.navsys&SYS_GLO) && rtk->opt.glomodear==GLO_ARMODE_FIXHOLD && rtk->sol.ratio<rtk->sol.thres) {
        glo2=sbas2=0;
        /* turn off gpsmode if not enabled and got good fix (used for debug and eval only) */
        gps2=rtk->opt.gpsmodear==0&&rtk->sol.ratio>=rtk->sol.thres?0:1;

        /* if modes changed since initial AR run or haven't run yet,re-run with new modes */
        if (glo1!=glo2||gps1!=gps2)
            nb=resamb_LAMBDA(rtk,bias,xa,gps2,glo2,sbas2);
    }
    /* restore excluded sat if still no fix or significant increase in ar ratio */
    if (excflag && (rtk->sol.ratio<rtk->sol.thres) && (rtk->sol.ratio<(1.5*rtk->sol.prev_ratio2))) {
        i=sat[arsats[rtk->excsat++]];
        for (f=0;f<nf;f++) rtk->ssat[i-1].lock[f]=lockc[f];
        trace(3,"AR: restore sat %d\n",i);
    }

    rtk->sol.prev_ratio1=ratio1>0?ratio1:rtk->sol.ratio;
    rtk->sol.prev_ratio2=rtk->sol.ratio;

    return nb;
}

/* validation of solution ----------------------------------------------------*/
static int valpos(rtk_t *rtk, const double *v, const double *R, const int *vflg,
                  int nv, double thres)
{
    double fact=thres*thres;
    int i,stat=1,sat1,sat2,type,freq;
    char *stype;

    trace(3,"valpos  : nv=%d thres=%.1f\n",nv,thres);

    /* post-fit residual test */
    for (i=0;i<nv;i++) {
        if (v[i]*v[i]<=fact*R[i+i*nv]) continue;
        sat1=(vflg[i]>>16)&0xFF;
        sat2=(vflg[i]>> 8)&0xFF;
        type=(vflg[i]>> 4)&0xF;
        freq=vflg[i]&0xF;
        stype=type==0?"L":(type==1?"P":"C");
        errmsg(rtk,"large residual (sat=%2d-%2d %s%d v=%6.3f sig=%.3f)\n",
              sat1,sat2,stype,freq+1,v[i],SQRT(R[i+i*nv]));
    }
    return stat;
}
/* relpos()relative positioning ------------------------------------------------------
 *  args:  rtk      IO      gps solution structure
           obs      I       satellite observations
           nu       I       # of user observations (rover)
           nr       I       # of ref observations  (base)
           nav      I       satellite navigation data
 */
static int relpos(rtk_t *rtk, const obsd_t *obs, int nu, int nr,
                  const nav_t *nav)
{
    prcopt_t *opt=&rtk->opt;
    gtime_t time=obs[0].time;
    double *rs,*dts,*var,*y,*e,*azel,*freq,*v,*H,*R,*xp,*Pp,*xa,*bias,dt;
    int i,j,f,n=nu+nr,ns,ny,nv,sat[MAXSAT],iu[MAXSAT],ir[MAXSAT];
    int info,vflg[MAXOBS*NFREQ*2+1],svh[MAXOBS*2];
    int stat=rtk->opt.mode<=PMODE_DGPS?SOLQ_DGPS:SOLQ_FLOAT;
    int nf=opt->ionoopt==IONOOPT_IFLC?1:opt->nf;

    /* time diff between base and rover observations */
    dt=timediff(time,obs[nu].time);
    trace(3,"relpos  : dt=%.3f nu=%d nr=%d\n",dt,nu,nr);

    /* define local matrices, n=total observations, base + rover */
    rs=mat(6,n);            /* range to satellites */
    dts=mat(2,n);           /* satellite clock biases */
    var=mat(1,n);
    y=mat(nf*2,n);
    e=mat(3,n);
    azel=zeros(2,n);        /* [az, el] */
    freq=zeros(nf,n);

    /* init satellite status arrays */
    for (i=0;i<MAXSAT;i++) {
        rtk->ssat[i].sys=satsys(i+1,NULL); /* gnss system */
        for (j=0;j<NFREQ;j++) {
            rtk->ssat[i].vsat[j]=0;  /* valid satellite */
                rtk->ssat[i].snr_rover[j]=0;
                rtk->ssat[i].snr_base[j] =0;
        }
    }
    /* compute satellite positions, velocities and clocks for base and rover */
    satposs(time,obs,n,nav,opt->sateph,rs,dts,var,svh);

    /* calculate [range - measured pseudorange] for base station (phase and code)
         output is in y[nu:nu+nr], see call for rover below for more details                                                 */
    trace(3,"base station:\n");
    if (!zdres(1,obs+nu,nr,rs+nu*6,dts+nu*2,var+nu,svh+nu,nav,rtk->rb,opt,
               y+nu*nf*2,e+nu*3,azel+nu*2,freq+nu*nf)) {
        errmsg(rtk,"initial base station position error\n");

        free(rs); free(dts); free(var); free(y); free(e); free(azel);
        free(freq);
        return 0;
    }
    /* time-interpolation of base residuals (for post-processing)  */
    if (opt->intpref) {
        dt=intpres(time,obs+nu,nr,nav,rtk,y+nu*nf*2);
    }
    /* select common satellites between rover and base-station */
    if ((ns=selsat(obs,azel,nu,nr,opt,sat,iu,ir))<=0) {
        errmsg(rtk,"no common satellite\n");

        free(rs); free(dts); free(var); free(y); free(e); free(azel);
        free(freq);
        return 0;
    }
    /* update kalman filter states (pos,vel,acc,ionosp, troposp, sat phase biases) */
    trace(4,"before udstate: x="); tracemat(4,rtk->x,1,NR(opt),13,4);
    udstate(rtk,obs,sat,iu,ir,ns,nav);
    trace(4,"after udstate x="); tracemat(4,rtk->x,1,NR(opt),13,4);

    for (i=0;i<ns;i++) for (j=0;j<nf;j++) {

        /* snr of base and rover receiver */
        rtk->ssat[sat[i]-1].snr_rover[j]=obs[iu[i]].SNR[j];
        rtk->ssat[sat[i]-1].snr_base[j] =obs[ir[i]].SNR[j];
    }

    /* initialize Pp,xa to zero, xp to rtk->x */
    xp=mat(rtk->nx,1); Pp=zeros(rtk->nx,rtk->nx); xa=mat(rtk->nx,1);
    matcpy(xp,rtk->x,rtk->nx,1);
    matcpy(Pp,rtk->P,rtk->nx,rtk->nx);

    ny=ns*nf*2+2;
    v=mat(ny,1); H=zeros(rtk->nx,ny); R=mat(ny,ny); bias=mat(rtk->nx,1);

    trace(3,"rover:  dt=%.3f\n",dt);
    for (i=0;i<opt->niter;i++) {
        /* calculate zero diff residuals [range - measured pseudorange] for rover (phase and code)
            output is in y[0:nu-1], only shared input with base is nav
                obs  = sat observations
                nu   = # of sats
                rs   = range to sats
                dts  = sat clock biases (rover)
                svh  = sat health flags
                nav  = sat nav data
                xp   = kalman states
                opt  = options
                y    = zero diff residuals (code and phase)
                e    = line of sight unit vectors to sats
                azel = [az, el] to sats                                   */
        if (!zdres(0,obs,nu,rs,dts,var,svh,nav,xp,opt,y,e,azel,freq)) {
            errmsg(rtk,"rover initial position error\n");
            stat=SOLQ_NONE;
            break;
        }
        /* calculate double-differenced residuals and create state matrix from sat angles
                O rtk->ssat[i].resp[j] = residual pseudorange error
                O rtk->ssat[i].resc[j] = residual carrier phase error
                I dt = time diff between base and rover observations
                I Pp = covariance matrix of float solution
                I sat = list of common sats
                I iu,ir = user and ref indices to sats
                I ns = # of sats
                O v = double diff residuals (phase and code)
                O H = partial derivatives
                O R = double diff measurement error covariances
                O vflg = list of sats used for dd  */
        if ((nv=ddres(rtk,obs,dt,xp,Pp,sat,y,e,azel,freq,iu,ir,ns,v,H,R,vflg))<4) {
            errmsg(rtk,"not enough double-differenced residual, n=%d\n", nv);
            stat=SOLQ_NONE;
            break;
        }
        /* kalman filter measurement update, updates x,y,z,sat phase biases, etc
                K=P*H*(H'*P*H+R)^-1
                xp=x+K*v
                Pp=(I-K*H')*P                  */
        trace(3,"before filter x=");tracemat(3,rtk->x,1,9,13,6);
        if ((info=filter(xp,Pp,H,v,R,rtk->nx,nv))) {
            errmsg(rtk,"filter error (info=%d)\n",info);
            stat=SOLQ_NONE;
            break;
        }
        trace(3,"after filter x=");tracemat(3,xp,1,9,13,6);
        trace(4,"x(%d)=",i+1); tracemat(4,xp,1,NR(opt),13,4);
    }
    /* calc zero diff residuals again after kalman filter update */
    if (stat!=SOLQ_NONE&&zdres(0,obs,nu,rs,dts,var,svh,nav,xp,opt,y,e,azel,freq)) {

        /* calc double diff residuals again after kalman filter update for float solution */
        nv=ddres(rtk,obs,dt,xp,Pp,sat,y,e,azel,freq,iu,ir,ns,v,NULL,R,vflg);
        
        /* validation of float solution, always returns 1, msg to trace file if large residual */
        if (valpos(rtk,v,R,vflg,nv,4.0)) {

            /* copy states */
            matcpy(rtk->x,xp,rtk->nx,1);
            matcpy(rtk->P,Pp,rtk->nx,rtk->nx);

            /* update valid satellite status for ambiguity control */
            rtk->sol.ns=0;
            for (i=0;i<ns;i++) for (f=0;f<nf;f++) {
                if (!rtk->ssat[sat[i]-1].vsat[f]) continue;
                rtk->ssat[sat[i]-1].outc[f]=0;
                if (f==0) rtk->sol.ns++; /* valid satellite count by L1 */
            }
            /* too few valid phases */
            if (rtk->sol.ns<4) stat=SOLQ_DGPS;
        }
        else stat=SOLQ_NONE;
    }
    /* resolve integer ambiguity by LAMBDA */
    if (stat==SOLQ_FLOAT) {
        /* if valid fixed solution, process it */
        if (manage_amb_LAMBDA(rtk,bias,xa,sat,nf,ns)>1) {

            /* find zero-diff residuals for fixed solution */
            if (zdres(0,obs,nu,rs,dts,var,svh,nav,xa,opt,y,e,azel,freq)) {

                /* post-fit residuals for fixed solution (xa includes fixed phase biases, rtk->xa does not) */
                nv=ddres(rtk,obs,dt,xa,Pp,sat,y,e,azel,freq,iu,ir,ns,v,NULL,R,vflg);

                /* validation of fixed solution, always returns valid */
                if (valpos(rtk,v,R,vflg,nv,4.0)) {

                    /* hold integer ambiguity if meet minfix count */
                    if (++rtk->nfix>=rtk->opt.minfix) {
                        if (rtk->opt.modear==ARMODE_FIXHOLD||rtk->opt.glomodear==GLO_ARMODE_FIXHOLD)
                            holdamb(rtk,xa);
                        /* switch to kinematic after qualify for hold if in static-start mode */
                        if (rtk->opt.mode==PMODE_STATIC_START) {
                            rtk->opt.mode=PMODE_KINEMA;
                            trace(3,"Fix and hold complete: switch to kinematic mode\n");
                        }
                    }
                    stat=SOLQ_FIX;
                }
            }
        }
    }

    /* save solution status (fixed or float) */
    if (stat==SOLQ_FIX) {
        for (i=0;i<3;i++) {
            rtk->sol.rr[i]=rtk->xa[i];
            rtk->sol.qr[i]=(float)rtk->Pa[i+i*rtk->na];
        }
        rtk->sol.qr[3]=(float)rtk->Pa[1];
        rtk->sol.qr[4]=(float)rtk->Pa[1+2*rtk->na];
        rtk->sol.qr[5]=(float)rtk->Pa[2];

        if (rtk->opt.dynamics) { /* velocity and covariance */
            for (i=3;i<6;i++) {
                rtk->sol.rr[i]=rtk->xa[i];
                rtk->sol.qv[i-3]=(float)rtk->Pa[i+i*rtk->na];
    }
            rtk->sol.qv[3]=(float)rtk->Pa[4+3*rtk->na];
            rtk->sol.qv[4]=(float)rtk->Pa[5+4*rtk->na];
            rtk->sol.qv[5]=(float)rtk->Pa[5+3*rtk->na];
        }
    }
    else {  /* float solution */
        for (i=0;i<3;i++) {
            rtk->sol.rr[i]=rtk->x[i];
            rtk->sol.qr[i]=(float)rtk->P[i+i*rtk->nx];
        }
        rtk->sol.qr[3]=(float)rtk->P[1];
        rtk->sol.qr[4]=(float)rtk->P[1+2*rtk->nx];
        rtk->sol.qr[5]=(float)rtk->P[2];

        if (rtk->opt.dynamics) { /* velocity and covariance */
            for (i=3;i<6;i++) {
                rtk->sol.rr[i]=rtk->x[i];
                rtk->sol.qv[i-3]=(float)rtk->P[i+i*rtk->nx];
            }
            rtk->sol.qv[3]=(float)rtk->P[4+3*rtk->nx];
            rtk->sol.qv[4]=(float)rtk->P[5+4*rtk->nx];
            rtk->sol.qv[5]=(float)rtk->P[5+3*rtk->nx];
        }
        rtk->nfix=0;
    }
    trace(3,"sol_rr= ");tracemat(3,rtk->sol.rr,1,6,15,3);
    /* save phase measurements */
    for (i=0;i<n;i++) for (j=0;j<nf;j++) {
        if (obs[i].L[j]==0.0) continue;
        rtk->ssat[obs[i].sat-1].pt[obs[i].rcv-1][j]=obs[i].time;
        rtk->ssat[obs[i].sat-1].ph[obs[i].rcv-1][j]=obs[i].L[j];
    }
    for (i=0;i<MAXSAT;i++) for (j=0;j<nf;j++) {
        /* Don't lose track of which sats were used to try and resolve the ambiguities */
        /* if (rtk->ssat[i].fix[j]==2&&stat!=SOLQ_FIX) rtk->ssat[i].fix[j]=1; */
        if (rtk->ssat[i].slip[j]&1) rtk->ssat[i].slipc[j]++;
        /* inc lock count if this sat used for good fix */
        if (!rtk->ssat[i].vsat[j]) continue;
        if (rtk->ssat[i].lock[j]<0||(rtk->nfix>0&&rtk->ssat[i].fix[j]>=2))
            rtk->ssat[i].lock[j]++;
    }
    free(rs); free(dts); free(var); free(y); free(e); free(azel); free(freq);
    free(xp); free(Pp);  free(xa);  free(v); free(H); free(R); free(bias);

    if (stat!=SOLQ_NONE) rtk->sol.stat=stat;

    return stat!=SOLQ_NONE;
}
/* initialize RTK control ------------------------------------------------------
* initialize RTK control struct
* args   : rtk_t    *rtk    IO  TKk control/result struct
*          prcopt_t *opt    I   positioning options (see rtklib.h)
* return : none
*-----------------------------------------------------------------------------*/
extern void rtkinit(rtk_t *rtk, const prcopt_t *opt)
{
    sol_t sol0={{0}};
    ambc_t ambc0={{{0}}};
    ssat_t ssat0={0};
    int i;

    trace(3,"rtkinit :\n");

    rtk->sol=sol0;
    for (i=0;i<6;i++) rtk->rb[i]=0.0;
    rtk->nx=opt->mode<=PMODE_FIXED?NX(opt):pppnx(opt);
    rtk->na=opt->mode<=PMODE_FIXED?NR(opt):pppnx(opt);
    rtk->tt=0.0;
    rtk->epoch=0;
    rtk->x=zeros(rtk->nx,1);
    rtk->P=zeros(rtk->nx,rtk->nx);
    rtk->xa=zeros(rtk->na,1);
    rtk->Pa=zeros(rtk->na,rtk->na);
    rtk->nfix=rtk->neb=0;
    for (i=0;i<MAXSAT;i++) {
        rtk->ambc[i]=ambc0;
        rtk->ssat[i]=ssat0;
    }
    rtk->holdamb=0;
    rtk->excsat=0;
    rtk->nb_ar=0;
    for (i=0;i<MAXERRMSG;i++) rtk->errbuf[i]=0;
    rtk->opt=*opt;
    rtk->initial_mode=rtk->opt.mode;
    rtk->sol.thres=(float)opt->thresar[0];
}
/* free rtk control ------------------------------------------------------------
* free memory for rtk control struct
* args   : rtk_t    *rtk    IO  rtk control/result struct
* return : none
*-----------------------------------------------------------------------------*/
extern void rtkfree(rtk_t *rtk)
{
    trace(3,"rtkfree :\n");

    rtk->nx=rtk->na=0;
    free(rtk->x ); rtk->x =NULL;
    free(rtk->P ); rtk->P =NULL;
    free(rtk->xa); rtk->xa=NULL;
    free(rtk->Pa); rtk->Pa=NULL;
}
/* precise positioning ---------------------------------------------------------
* input observation data and navigation message, compute rover position by
* precise positioning
* args   : rtk_t *rtk       IO  RTK control/result struct
*            rtk->sol       IO  solution
*                .time      O   solution time
*                .rr[]      IO  rover position/velocity
*                               (I:fixed mode,O:single mode)
*                .dtr[0]    O   receiver clock bias (s)
*                .dtr[1-5]  O   receiver GLO/GAL/BDS/IRN/QZS-GPS time offset (s)
*                .Qr[]      O   rover position covarinace
*                .stat      O   solution status (SOLQ_???)
*                .ns        O   number of valid satellites
*                .age       O   age of differential (s)
*                .ratio     O   ratio factor for ambiguity validation
*            rtk->rb[]      IO  base station position/velocity
*                               (I:relative mode,O:moving-base mode)
*            rtk->nx        I   number of all states
*            rtk->na        I   number of integer states
*            rtk->ns        O   number of valid satellites in use
*            rtk->tt        O   time difference between current and previous (s)
*            rtk->x[]       IO  float states pre-filter and post-filter
*            rtk->P[]       IO  float covariance pre-filter and post-filter
*            rtk->xa[]      O   fixed states after AR
*            rtk->Pa[]      O   fixed covariance after AR
*            rtk->ssat[s]   IO  satellite {s+1} status
*                .sys       O   system (SYS_???)
*                .az   [r]  O   azimuth angle   (rad) (r=0:rover,1:base)
*                .el   [r]  O   elevation angle (rad) (r=0:rover,1:base)
*                .vs   [r]  O   data valid single     (r=0:rover,1:base)
*                .resp [f]  O   freq(f+1) pseudorange residual (m)
*                .resc [f]  O   freq(f+1) carrier-phase residual (m)
*                .vsat [f]  O   freq(f+1) data vaild (0:invalid,1:valid)
*                .fix  [f]  O   freq(f+1) ambiguity flag
*                               (0:nodata,1:float,2:fix,3:hold)
*                .slip [f]  O   freq(f+1) cycle slip flag
*                               (bit8-7:rcv1 LLI, bit6-5:rcv2 LLI,
*                                bit2:parity unknown, bit1:slip)
*                .lock [f]  IO  freq(f+1) carrier lock count
*                .outc [f]  IO  freq(f+1) carrier outage count
*                .slipc[f]  IO  freq(f+1) cycle slip count
*                .rejc [f]  IO  freq(f+1) data reject count
*                .gf        IO  geometry-free phase (L1-L2 or L1-L5) (m)
*            rtk->nfix      IO  number of continuous fixes of ambiguity
*            rtk->neb       IO  bytes of error message buffer
*            rtk->errbuf    IO  error message buffer
*            rtk->tstr      O   time string for debug
*            rtk->opt       I   processing options
*          obsd_t *obs      I   observation data for an epoch
*                               obs[i].rcv=1:rover,2:reference
*                               sorted by receiver and satellte
*          int    n         I   number of observation data
*          nav_t  *nav      I   navigation messages
* return : status (0:no solution,1:valid solution)
* notes  : before calling function, base station position rtk->sol.rb[] should
*          be properly set for relative mode except for moving-baseline
*-----------------------------------------------------------------------------*/
extern int rtkpos(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav)
{
    prcopt_t *opt=&rtk->opt;
    sol_t solb={{0}};
    gtime_t time;
    int i,nu,nr;
    char msg[128]="";

    trace(3,"rtkpos  : time=%s n=%d\n",time_str(obs[0].time,3),n);
    trace(4,"obs=\n"); traceobs(4,obs,n);
    /*trace(5,"nav=\n"); tracenav(5,nav);*/

    /* set base station position */
    if (opt->refpos<=POSOPT_RINEX&&opt->mode!=PMODE_SINGLE&&
        opt->mode!=PMODE_MOVEB) {
        for (i=0;i<6;i++) rtk->rb[i]=i<3?opt->rb[i]:0.0;
    }
    /* count rover/base station observations */
    for (nu=0;nu   <n&&obs[nu   ].rcv==1;nu++) ;
    for (nr=0;nu+nr<n&&obs[nu+nr].rcv==2;nr++) ;

    time=rtk->sol.time; /* previous epoch */

    /* rover position and time by single point positioning, skip if
     position variance smaller than threshold */
    if (rtk->P[0]==0||rtk->P[0]>STD_PREC_VAR_THRESH) {
        if (!pntpos(obs,nu,nav,&rtk->opt,&rtk->sol,NULL,rtk->ssat,msg)) {
            errmsg(rtk,"point pos error (%s)\n",msg);

            if (!rtk->opt.dynamics) {
                outsolstat(rtk,nav);
                return 0;
            }
        }
    } else rtk->sol.time=obs[0].time;
    if (time.time!=0) rtk->tt=timediff(rtk->sol.time,time);

    /* return to static start if long delay without rover data */
    if (fabs(rtk->tt)>300&&rtk->initial_mode==PMODE_STATIC_START) {
        rtk->opt.mode=PMODE_STATIC_START;
        for (i=0;i<3;i++) initx(rtk,rtk->sol.rr[i],VAR_POS,i);
        if (rtk->opt.dynamics) {
            for (i=3;i<6;i++) initx(rtk,1E-6,VAR_VEL,i);
            for (i=6;i<9;i++) initx(rtk,1E-6,VAR_ACC,i);
        }
        trace(3,"No data for > 5 min: switch back to static mode:\n");
    }

    /* single point positioning */
    if (opt->mode==PMODE_SINGLE) {
        outsolstat(rtk,nav);
        return 1;
    }
    /* suppress output of single solution */
    if (!opt->outsingle) {
        rtk->sol.stat=SOLQ_NONE;
    }
    /* precise point positioning */
    if (opt->mode>=PMODE_PPP_KINEMA) {
        pppos(rtk,obs,nu,nav);
        outsolstat(rtk,nav);
        return 1;
    }
    /* check number of data of base station and age of differential */
    if (nr==0) {
        errmsg(rtk,"no base station observation data for rtk\n");
        outsolstat(rtk,nav);
        return 1;
    }
    if (opt->mode==PMODE_MOVEB) { /*  moving baseline */
        /* estimate position/velocity of base station,
           skip if position varinace below threshold*/
        if (rtk->P[0]==0||rtk->P[0]>STD_PREC_VAR_THRESH) {
            if (!pntpos(obs+nu,nr,nav,&rtk->opt,&solb,NULL,NULL,msg)) {
                errmsg(rtk,"base station position error (%s)\n",msg);
                return 0;
            }
            /* if base position uninitialized, use full position */
            if (fabs(rtk->rb[0])<0.1)
                for (i=0;i<3;i++) rtk->rb[i]=solb.rr[i];
            /* else filter base position to reduce noise from single precision solution */
            else
                for (i=0;i<3;i++) {
                    rtk->rb[i]=0.95*rtk->rb[i]+0.05*solb.rr[i];
                    rtk->rb[i+3]=0; /* set velocity to zero */
                }
        } else solb.time=obs[nu].time;
        trace(3,"basex= %.3f %.3f\n",rtk->rb[0],solb.rr[0]);

        rtk->sol.age=(float)timediff(rtk->sol.time,solb.time);

        if (fabs(rtk->sol.age)>MIN(TTOL_MOVEB,opt->maxtdiff)) {
            errmsg(rtk,"time sync error for moving-base (age=%.1f)\n",rtk->sol.age);
            return 0;
        }

        /* time-synchronized position of base station */
        /* single position velocity solution too noisy to be helpful */
        /*for (i=0;i<3;i++) rtk->rb[i]+=rtk->rb[i+3]*rtk->sol.age; */

    trace(3,"base pos: "); tracemat(3,rtk->rb,1,3,13,4);
    }
    else {
        rtk->sol.age=(float)timediff(obs[0].time,obs[nu].time);

        if (fabs(rtk->sol.age)>opt->maxtdiff) {
            errmsg(rtk,"age of differential error (age=%.1f)\n",rtk->sol.age);
            outsolstat(rtk,nav);
            return 1;
        }
    }
    /* relative potitioning */
    relpos(rtk,obs,nu,nr,nav);
    rtk->epoch++;
    outsolstat(rtk,nav);

    return 1;
}
