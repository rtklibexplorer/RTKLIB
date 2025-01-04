/*------------------------------------------------------------------------------
* pntpos.c : standard positioning
*
*          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
*
* version : $Revision:$ $Date:$
* history : 2010/07/28 1.0  moved from rtkcmn.c
*                           changed api:
*                               pntpos()
*                           deleted api:
*                               pntvel()
*           2011/01/12 1.1  add option to include unhealthy satellite
*                           reject duplicated observation data
*                           changed api: ionocorr()
*           2011/11/08 1.2  enable snr mask for single-mode (rtklib_2.4.1_p3)
*           2012/12/25 1.3  add variable snr mask
*           2014/05/26 1.4  support galileo and beidou
*           2015/03/19 1.5  fix bug on ionosphere correction for GLO and BDS
*           2018/10/10 1.6  support api change of satexclude()
*           2020/11/30 1.7  support NavIC/IRNSS in pntpos()
*                           no support IONOOPT_LEX option in ioncorr()
*                           improve handling of TGD correction for each system
*                           use E1-E5b for Galileo dual-freq iono-correction
*                           use API sat2freq() to get carrier frequency
*                           add output of velocity estimation error in estvel()
*-----------------------------------------------------------------------------*/
#include "rtklib.h"

/* constants/macros ----------------------------------------------------------*/

#define SQR(x)      ((x)*(x))
#define MAX(x,y)    ((x)>=(y)?(x):(y))

#if 0 /* enable GPS-QZS time offset estimation */
#define NX          (4+5)       /* # of estimated parameters */
#else
#define NX          (4+4)       /* # of estimated parameters */
#endif
#define MAXITR      10          /* max number of iteration for point pos */
#define ERR_ION     5.0         /* ionospheric delay Std (m) */
#define ERR_TROP    3.0         /* tropspheric delay Std (m) */
#define ERR_SAAS    0.3         /* Saastamoinen model error Std (m) */
#define ERR_BRDCI   0.5         /* broadcast ionosphere model error factor */
#define ERR_CBIAS   0.3         /* code bias error Std (m) */
#define REL_HUMI    0.7         /* relative humidity for Saastamoinen model */
#define MIN_EL      (5.0*D2R)   /* min elevation for measurement error (rad) */
# define MAX_GDOP   30          /* max gdop for valid solution  */

/* get group delay parameter (m) ---------------------------------------------*/
static double gettgd(int sat, const nav_t *nav, int type)
{
    int i,sys=satsys(sat,NULL);
    
    if (sys==SYS_GLO) {
        for (i=0;i<nav->ng;i++) {
            if (nav->geph[i].sat==sat) break;
        }
        return (i>=nav->ng)?0.0:-nav->geph[i].dtaun*CLIGHT;
    }
    else {
        for (i=0;i<nav->n;i++) {
            if (nav->eph[i].sat==sat) break;
        }
        return (i>=nav->n)?0.0:nav->eph[i].tgd[type]*CLIGHT;
    }
}
/* test SNR mask -------------------------------------------------------------*/
static int snrmask(const obsd_t *obs, const double *azel, const prcopt_t *opt)
{
    int f2;

    if (testsnr(0,0,azel[1],obs->SNR[0]*SNR_UNIT,&opt->snrmask)) {
        return 0;
    }
    if (opt->ionoopt==IONOOPT_IFLC) {
        f2=seliflc(opt->nf,satsys(obs->sat,NULL));
        if (testsnr(0,f2,azel[1],obs->SNR[f2]*SNR_UNIT,&opt->snrmask)) return 0;
    }
    return 1;
}
/* iono-free or "pseudo iono-free" pseudorange with code bias correction -----*/
static double prange(const obsd_t *obs, const nav_t *nav, const prcopt_t *opt,
                     double *var)
{
    double P1,P2,gamma,b1,b2;
    int sat,sys,f2,bias_ix;

    sat=obs->sat;
    sys=satsys(sat,NULL);
    P1=obs->P[0];
    f2=seliflc(opt->nf,satsys(obs->sat,NULL));
    P2=obs->P[f2];
    *var=0.0;
    
    if (P1==0.0||(opt->ionoopt==IONOOPT_IFLC&&P2==0.0)) return 0.0;
    bias_ix=code2bias_ix(sys,obs->code[0]);  /* L1 code bias */
    if (bias_ix>0) { /* 0=ref code */
        P1+=nav->cbias[sat-1][0][bias_ix-1];
    }
    /* GPS code biases are L1/L2, Galileo are L1/L5 */
    if (sys==SYS_GAL&&f2==1) {
        /* skip code bias, no GAL L2 bias available */
    }
    else {  /* apply L2 or L5 code bias */
        bias_ix=code2bias_ix(sys,obs->code[f2]);
        if (bias_ix>0) { /* 0=ref code */
            P2+=nav->cbias[sat-1][1][bias_ix-1]; /* L2 or L5 code bias */
        }
    }
    if (opt->ionoopt==IONOOPT_IFLC) { /* dual-frequency */
        
        if (sys==SYS_GPS||sys==SYS_QZS) { /* L1-L2 or L1-L5 */
            gamma=f2==1?SQR(FREQL1/FREQL2):SQR(FREQL1/FREQL5);
            return (P2-gamma*P1)/(1.0-gamma);
        }
        else if (sys==SYS_GLO) { /* G1-G2 or G1-G3 */
            gamma=f2==1?SQR(FREQ1_GLO/FREQ2_GLO):SQR(FREQ1_GLO/FREQ3_GLO);
            return (P2-gamma*P1)/(1.0-gamma);
        }
        else if (sys==SYS_GAL) { /* E1-E5b, E1-E5a */
            gamma=f2==1?SQR(FREQL1/FREQE5b):SQR(FREQL1/FREQL5);
            if (f2==1&&getseleph(SYS_GAL)) { /* F/NAV */
                P2-=gettgd(sat,nav,0)-gettgd(sat,nav,1); /* BGD_E5aE5b */
            }
            return (P2-gamma*P1)/(1.0-gamma);
        }
        else if (sys==SYS_CMP) { /* B1-B2 */
            gamma=SQR(((obs->code[0]==CODE_L2I)?FREQ1_CMP:FREQL1)/FREQ2_CMP);
            if      (obs->code[0]==CODE_L2I) b1=gettgd(sat,nav,0); /* TGD_B1I */
            else if (obs->code[0]==CODE_L1P) b1=gettgd(sat,nav,2); /* TGD_B1Cp */
            else b1=gettgd(sat,nav,2)+gettgd(sat,nav,4); /* TGD_B1Cp+ISC_B1Cd */
            b2=gettgd(sat,nav,1); /* TGD_B2I/B2bI (m) */
            return ((P2-gamma*P1)-(b2-gamma*b1))/(1.0-gamma);
        }
        else if (sys==SYS_IRN) { /* L5-S */
            gamma=SQR(FREQL5/FREQs);
            return (P2-gamma*P1)/(1.0-gamma);
        }
    }
    else { /* single-freq (L1/E1/B1) */
        *var=SQR(ERR_CBIAS);
        
        if (sys==SYS_GPS||sys==SYS_QZS) { /* L1 */
            b1=gettgd(sat,nav,0); /* TGD (m) */
            return P1-b1;
        }
        else if (sys==SYS_GLO) { /* G1 */
            gamma=SQR(FREQ1_GLO/FREQ2_GLO);
            b1=gettgd(sat,nav,0); /* -dtaun (m) */
            return P1-b1/(gamma-1.0);
        }
        else if (sys==SYS_GAL) { /* E1 */
            if (getseleph(SYS_GAL)) b1=gettgd(sat,nav,0); /* BGD_E1E5a */
            else                    b1=gettgd(sat,nav,1); /* BGD_E1E5b */
            return P1-b1;
        }
        else if (sys==SYS_CMP) { /* B1I/B1Cp/B1Cd */
            if      (obs->code[0]==CODE_L2I) b1=gettgd(sat,nav,0); /* TGD_B1I */
            else if (obs->code[0]==CODE_L1P) b1=gettgd(sat,nav,2); /* TGD_B1Cp */
            else b1=gettgd(sat,nav,2)+gettgd(sat,nav,4); /* TGD_B1Cp+ISC_B1Cd */
            return P1-b1;
        }
        else if (sys==SYS_IRN) { /* L5 */
            gamma=SQR(FREQs/FREQL5);
            b1=gettgd(sat,nav,0); /* TGD (m) */
            return P1-gamma*b1;
        }
    }
    return P1;
}
/* ionospheric correction ------------------------------------------------------
* compute ionospheric correction
* args   : gtime_t time     I   time
*          nav_t  *nav      I   navigation data
*          int    sat       I   satellite number
*          double *pos      I   receiver position {lat,lon,h} (rad|m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
*          int    ionoopt   I   ionospheric correction option (IONOOPT_???)
*          double *ion      O   ionospheric delay (L1) (m)
*          double *var      O   ionospheric delay (L1) variance (m^2)
* return : status(1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int ionocorr(gtime_t time, const nav_t *nav, int sat, const double *pos,
                    const double *azel, int ionoopt, double *ion, double *var)
{
    int err=0;

    trace(4,"ionocorr: time=%s opt=%d sat=%2d pos=%.3f %.3f azel=%.3f %.3f\n",
          time_str(time,3),ionoopt,sat,pos[0]*R2D,pos[1]*R2D,azel[0]*R2D,
          azel[1]*R2D);
    
    /* SBAS ionosphere model */
    if (ionoopt==IONOOPT_SBAS) {
        if (sbsioncorr(time,nav,pos,azel,ion,var)) return 1;
        err=1;
    }
    /* IONEX TEC model */
    if (ionoopt==IONOOPT_TEC) {
        if (iontec(time,nav,pos,azel,1,ion,var)) return 1;
        err=1;
    }
    /* QZSS broadcast ionosphere model */
    if (ionoopt==IONOOPT_QZS&&norm(nav->ion_qzs,8)>0.0) {
        *ion=ionmodel(time,nav->ion_qzs,pos,azel);
        *var=SQR(*ion*ERR_BRDCI);
        return 1;
    }
    /* GPS broadcast ionosphere model */
    if (ionoopt==IONOOPT_BRDC||err==1) {
        *ion=ionmodel(time,nav->ion_gps,pos,azel);
        *var=SQR(*ion*ERR_BRDCI);
        return 1;
    }
    *ion=0.0;
    *var=ionoopt==IONOOPT_OFF?SQR(ERR_ION):0.0;
    return 1;
}
/* tropospheric correction -----------------------------------------------------
* compute tropospheric correction
* args   : gtime_t time     I   time
*          nav_t  *nav      I   navigation data
*          double *pos      I   receiver position {lat,lon,h} (rad|m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
*          int    tropopt   I   tropospheric correction option (TROPOPT_???)
*          double *trp      O   tropospheric delay (m)
*          double *var      O   tropospheric delay variance (m^2)
* return : status(1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int tropcorr(gtime_t time, const nav_t *nav, const double *pos,
                    const double *azel, int tropopt, double *trp, double *var)
{
    trace(4,"tropcorr: time=%s opt=%d pos=%.3f %.3f azel=%.3f %.3f\n",
          time_str(time,3),tropopt,pos[0]*R2D,pos[1]*R2D,azel[0]*R2D,
          azel[1]*R2D);
    
    /* Saastamoinen model */
    if (tropopt==TROPOPT_SAAS||tropopt==TROPOPT_EST||tropopt==TROPOPT_ESTG) {
        *trp=tropmodel(time,pos,azel,REL_HUMI);
        *var=SQR(ERR_SAAS/(sin(azel[1])+0.1));
        return 1;
    }
    /* SBAS (MOPS) troposphere model */
    if (tropopt==TROPOPT_SBAS) {
        *trp=sbstropcorr(time,pos,azel,var);
        return 1;
    }
    /* no correction */
    *trp=0.0;
    *var=tropopt==TROPOPT_OFF?SQR(ERR_TROP):0.0;
    return 1;
}
/* pseudorange residuals -----------------------------------------------------*/
static int rescode(int iter, const obsd_t *obs, int n, const double *rs,
                   const double *dts, const double *vare, const int *svh,
                   const nav_t *nav, const double *x, const prcopt_t *opt,
                   const ssat_t *ssat, double *v, double *H, double *var,
                   double *azel, int *vsat, double *resp, int *ns)
{
    gtime_t time;
    double r,freq,dion=0.0,dtrp=0.0,vmeas,vion=0.0,vtrp=0.0,rr[3],pos[3],dtr,e[3],P;
    int i,j,nv=0,sat,sys,mask[NX-3]={0};

    for (i=0;i<3;i++) rr[i]=x[i];
    dtr=x[3];
    
    ecef2pos(rr,pos);
    trace(3,"rescode: rr=%.3f %.3f %.3f\n",rr[0], rr[1], rr[2]);
    
    for (i=*ns=0;i<n&&i<MAXOBS;i++) {
        vsat[i]=0; azel[i*2]=azel[1+i*2]=resp[i]=0.0;
        time=obs[i].time;
        sat=obs[i].sat;
        if (!(sys=satsys(sat,NULL))) continue;
        
        /* reject duplicated observation data */
        if (i<n-1&&i<MAXOBS-1&&sat==obs[i+1].sat) {
            trace(2,"duplicated obs data %s sat=%d\n",time_str(time,3),sat);
            i++;
            continue;
        }
        /* excluded satellite? */
        if (satexclude(sat,vare[i],svh[i],opt)) continue;
        
        /* geometric distance and elevation mask*/
        if ((r=geodist(rs+i*6,rr,e))<=0.0) continue;
        if (satazel(pos,e,azel+i*2)<opt->elmin) continue;
        
        if (iter>0) {
            /* test SNR mask */
            if (!snrmask(obs+i,azel+i*2,opt)) continue;
        
            /* ionospheric correction */
            if (!ionocorr(time,nav,sat,pos,azel+i*2,opt->ionoopt,&dion,&vion)) {
                continue;
            }
            if ((freq=sat2freq(sat,obs[i].code[0],nav))==0.0) continue;
            dion*=SQR(FREQL1/freq);
            vion*=SQR(FREQL1/freq);
        
            /* tropospheric correction */
            if (!tropcorr(time,nav,pos,azel+i*2,opt->tropopt,&dtrp,&vtrp)) {
                continue;
            }
        }
        /* pseudorange with code bias correction */
        if ((P=prange(obs+i,nav,opt,&vmeas))==0.0) continue;
        
        /* pseudorange residual */
        v[nv]=P-(r+dtr-CLIGHT*dts[i*2]+dion+dtrp);
        trace(4,"sat=%d: v=%.3f P=%.3f r=%.3f dtr=%.6f dts=%.6f dion=%.3f dtrp=%.3f\n",
            sat,v[nv],P,r,dtr,dts[i*2],dion,dtrp);
        
        /* design matrix */
        for (j=0;j<NX;j++) {
            H[j+nv*NX]=j<3?-e[j]:(j==3?1.0:0.0);
        }
        /* time system offset and receiver bias correction */
        if      (sys==SYS_GLO) {v[nv]-=x[4]; H[4+nv*NX]=1.0; mask[1]=1;}
        else if (sys==SYS_GAL) {v[nv]-=x[5]; H[5+nv*NX]=1.0; mask[2]=1;}
        else if (sys==SYS_CMP) {v[nv]-=x[6]; H[6+nv*NX]=1.0; mask[3]=1;}
        else if (sys==SYS_IRN) {v[nv]-=x[7]; H[7+nv*NX]=1.0; mask[4]=1;}
#if 0 /* enable QZS-GPS time offset estimation */
        else if (sys==SYS_QZS) {v[nv]-=x[8]; H[8+nv*NX]=1.0; mask[5]=1;}
#endif
        else mask[0]=1;

        vsat[i]=1; resp[i]=v[nv]; (*ns)++;
        
        /* variance of pseudorange error */
        var[nv]=vare[i]+vmeas+vion+vtrp;
        if (ssat)
           var[nv++]+=varerr(sat,sys,azel[1+i*2],SNR_UNIT*ssat[sat-1].snr_rover[0],
               opt->err[5],0,0,opt->nf,opt,&obs[i]);
        else
           var[nv++]+=varerr(sat,sys,azel[1+i*2],opt->err[5],opt->err[5],0,0,opt->nf,opt,&obs[i]);
        trace(4,"sat=%2d azel=%5.1f %4.1f res=%7.3f sig=%5.3f\n",obs[i].sat,
              azel[i*2]*R2D,azel[1+i*2]*R2D,resp[i],sqrt(var[nv-1]));
    }
    /* constraint to avoid rank-deficient */
    for (i=0;i<NX-3;i++) {
        if (mask[i]) continue;
        v[nv]=0.0;
        for (j=0;j<NX;j++) H[j+nv*NX]=j==i+3?1.0:0.0;
        var[nv++]=0.01;
    }
    return nv;
}
/* validate solution ---------------------------------------------------------*/
static int valsol(const double *azel, const int *vsat, int n,
                  const prcopt_t *opt, const double *v, int nv, int nx,
                  char *msg)
{
    double azels[MAXOBS*2],dop[4],vv;
    int i,ns;
    
    trace(3,"valsol  : n=%d nv=%d\n",n,nv);
    
    /* Chi-square validation of residuals */
    vv=dot(v,v,nv);
    if (nv>nx&&vv>chisqr[nv-nx-1]) {
        sprintf(msg,"Warning: large chi-square error nv=%d vv=%.1f cs=%.1f",nv,vv,chisqr[nv-nx-1]);
        /* return 0; */ /* threshold too strict for all use cases, report error but continue on */
    }
    /* large GDOP check */
    for (i=ns=0;i<n;i++) {
        if (!vsat[i]) continue;
        azels[  ns*2]=azel[  i*2];
        azels[1+ns*2]=azel[1+i*2];
        ns++;
    }
    dops(ns,azels,opt->elmin,dop);
    if (dop[0]<=0.0||dop[0]>MAX_GDOP) {
        sprintf(msg,"gdop error nv=%d gdop=%.1f",nv,dop[0]);
        return 0;
    }
    return 1;
}
/* estimate receiver position ------------------------------------------------*/
static int estpos(const obsd_t *obs, int n, const double *rs, const double *dts,
                  const double *vare, const int *svh, const nav_t *nav,
                  const prcopt_t *opt, const ssat_t *ssat, sol_t *sol, double *azel,
                  int *vsat, double *resp, char *msg)
{
    double x[NX]={0},dx[NX],Q[NX*NX],*v,*H,*var,sig;
    int i,j,k,info,stat,nv,ns;
    
    trace(3,"estpos  : n=%d\n",n);
    
    v=mat(n+4,1); H=mat(NX,n+4); var=mat(n+4,1);
    
    for (i=0;i<3;i++) x[i]=sol->rr[i];

    for (i=0;i<MAXITR;i++) {

        /* pseudorange residuals (m) */
        nv=rescode(i,obs,n,rs,dts,vare,svh,nav,x,opt,ssat,v,H,var,azel,vsat,resp,
                   &ns);
        
        if (nv<NX) {
            sprintf(msg,"lack of valid sats ns=%d",nv);
            break;
        }
        /* weight by variance (lsq uses sqrt of weight */
        for (j=0;j<nv;j++) {
            sig=sqrt(var[j]);
            v[j]/=sig;
            for (k=0;k<NX;k++) H[k+j*NX]/=sig;
        }
        /* least square estimation */
        if ((info=lsq(H,v,NX,nv,dx,Q))) {
            sprintf(msg,"lsq error info=%d",info);
            break;
        }
        for (j=0;j<NX;j++) {
            x[j]+=dx[j];
        }
        if (norm(dx,NX)<1E-4) {
            sol->type=0;
            sol->time=timeadd(obs[0].time,-x[3]/CLIGHT);
            sol->dtr[0]=x[3]/CLIGHT; /* receiver clock bias (s) */
            sol->dtr[1]=x[4]/CLIGHT; /* GLO-GPS time offset (s) */
            sol->dtr[2]=x[5]/CLIGHT; /* GAL-GPS time offset (s) */
            sol->dtr[3]=x[6]/CLIGHT; /* BDS-GPS time offset (s) */
            sol->dtr[4]=x[7]/CLIGHT; /* IRN-GPS time offset (s) */
            for (j=0;j<6;j++) sol->rr_init[j]=j<3?x[j]:0.0;
//            for (j=0;j<3;j++) sol->qr[j]=(float)Q[j+j*NX];
//            sol->qr[3]=(float)Q[1];    /* cov xy */
//            sol->qr[4]=(float)Q[2+NX]; /* cov yz */
//            sol->qr[5]=(float)Q[2];    /* cov zx */
            sol->ns=(uint8_t)ns;
            sol->age=sol->ratio=0.0;
            trace(3,"clk offsets: %12.3f %12.3f %12.3f %12.3f\n",x[3],x[4],x[5],x[6]);
            
            /* validate solution */
            if ((stat=valsol(azel,vsat,n,opt,v,nv,NX,msg))) {
                sol->stat=opt->sateph==EPHOPT_SBAS?SOLQ_SBAS:SOLQ_SINGLE;
            }
            free(v); free(H); free(var);
            return stat;
        }
    }
    if (i>=MAXITR) sprintf(msg,"iteration divergent i=%d",i);
    
    free(v); free(H); free(var);
    return 0;
}
/* RAIM FDE (failure detection and exclusion) -------------------------------*/
static int raim_fde(const obsd_t *obs, int n, const double *rs,
                    const double *dts, const double *vare, const int *svh,
                    const nav_t *nav, const prcopt_t *opt, const ssat_t *ssat, 
                    sol_t *sol, double *azel, int *vsat, double *resp, char *msg)
{
    obsd_t *obs_e;
    sol_t sol_e={{0}};
    char tstr[32],name[16],msg_e[128];
    double *rs_e,*dts_e,*vare_e,*azel_e,*resp_e,rms_e,rms=100.0;
    int i,j,k,nvsat,stat=0,*svh_e,*vsat_e,sat=0;
    
    trace(3,"raim_fde: %s n=%2d\n",time_str(obs[0].time,0),n);
    
    if (!(obs_e=(obsd_t *)malloc(sizeof(obsd_t)*n))) return 0;
    rs_e = mat(6,n); dts_e = mat(2,n); vare_e=mat(1,n); azel_e=zeros(2,n);
    svh_e=imat(1,n); vsat_e=imat(1,n); resp_e=mat(1,n); 
    
    for (i=0;i<n;i++) {
        
        /* satellite exclusion */
        for (j=k=0;j<n;j++) {
            if (j==i) continue;
            obs_e[k]=obs[j];
            matcpy(rs_e +6*k,rs +6*j,6,1);
            matcpy(dts_e+2*k,dts+2*j,2,1);
            vare_e[k]=vare[j];
            svh_e[k++]=svh[j];
        }
        /* estimate receiver position without a satellite */
        if (!estpos(obs_e,n-1,rs_e,dts_e,vare_e,svh_e,nav,opt,ssat,&sol_e,azel_e,
                    vsat_e,resp_e,msg_e)) {
            trace(3,"raim_fde: exsat=%2d (%s)\n",obs[i].sat,msg);
            continue;
        }
        for (j=nvsat=0,rms_e=0.0;j<n-1;j++) {
            if (!vsat_e[j]) continue;
            rms_e+=SQR(resp_e[j]);
            nvsat++;
        }
        if (nvsat<5) {
            trace(3,"raim_fde: exsat=%2d lack of satellites nvsat=%2d\n",
                  obs[i].sat,nvsat);
            continue;
        }
        rms_e=sqrt(rms_e/nvsat);
        
        trace(3,"raim_fde: exsat=%2d rms=%8.3f\n",obs[i].sat,rms_e);
        
        if (rms_e>rms) continue;
        
        /* save result */
        for (j=k=0;j<n;j++) {
            if (j==i) continue;
            matcpy(azel+2*j,azel_e+2*k,2,1);
            vsat[j]=vsat_e[k];
            resp[j]=resp_e[k++];
        }
        stat=1;
        sol_e.eventime = sol->eventime;
        *sol=sol_e;
        sat=obs[i].sat;
        rms=rms_e;
        vsat[i]=0;
        strcpy(msg,msg_e);
    }
    if (stat) {
        time2str(obs[0].time,tstr,2); satno2id(sat,name);
        trace(2,"%s: %s excluded by raim\n",tstr+11,name);
    }
    free(obs_e);
    free(rs_e ); free(dts_e ); free(vare_e); free(azel_e);
    free(svh_e); free(vsat_e); free(resp_e);
    return stat;
}
/* range rate residuals ------------------------------------------------------*/
static int resdop(rtk_t *rtk, const obsd_t *obs, const int ns,
                  const int *iu, const int nf,
                  const double *rs_dop, const double *dts_dop,
                  const double *azel_dop, const double *rs_car, const double *dts_car,
                  const double *azel_car, const nav_t *nav, const double *rr,
                  const double *x, double *v,
                  double *H, int *svh)
{
    double freq,rate,pos[3],E[9],a[3],e[3],vs[3],cosel,sig;
    const double *azel,*rs;
    double dts,D,L,Lp,dL,Dp;
    int i,j,ix,f,nv=0,sat;
    
    trace(3,"resdop  : ns=%d\n",ns);
    ecef2pos(rr,pos); xyz2enu(pos,E);
    
    for (i=0;i<ns;i++) {
        ix=iu[i];
        sat=obs[ix].sat;
        for (f=0;f<nf;f++) {
            //trace(3,"sat=%d f=%d:\n",sat,f);
            /* excluded satellite?  */
            if (satexclude(sat,0,svh[i],&rtk->opt)) {
                trace(3,"Sat excluded\n");
                continue;
            }
            Dp=rtk->ssat[sat-1].D[f];
            //D=Dp!=0?(obs[ix].D[f]+Dp)/2:obs[ix].D[f];  // average velocity over last epoch (matches TDCP)
            D=obs[ix].D[f];    // instantaneous velocity, matches kalman filter
            L=obs[ix].L[f];
            Lp=rtk->ssat[sat-1].ph[0][f];
            freq=sat2freq(sat,obs[ix].code[f],nav);
        
            if (obs[ix].D[f]==0.0||freq==0.0) {
                //trace(3,"skip sat=%d f=%d D=%.2f freq=%.2f slip=%d\n",sat,f,obs[ix].D[f],freq,rtk->ssat[sat-1].slip[f]);
            continue;
        }

            /* Weight observation */
            sig=100*sqrt(varerr(sat,rtk->ssat[sat-1].sys,azel_dop[1+ix*2],
                            SNR_UNIT*rtk->ssat[sat-1].snr_rover[f],rtk->opt.err[5],
                            0,1,f,&rtk->opt,&obs[ix]));

            /* default to doppler */
            sig*= rtk->opt.err[4]; /* doppler weight */
            rs=&rs_dop[ix*6];
            azel=&azel_dop[ix*2];
            dts=dts_dop[ix*2+1];
            if (azel_dop[ix*2+1]<rtk->opt.elmin) continue; //sig*=10;
            if (testsnr(0,f,azel_dop[ix*2+1],obs[ix].SNR[f]*SNR_UNIT,&rtk->opt.snrmask)) continue; //sig*=10;

            /* time differenced carrier phase residuals */
            if (fabs(timediff(obs->time, rtk->ssat[sat-1].pt[0][f]))<fabs(1.5 * rtk->tt)) {
                dL = (Lp - L) / rtk->tt;
                //if (x[3]!=rtk->drift)  /* if not first iteration */
                    trace(3, "sat=%d f=%d: D=%.2f dL=%.2f L=%.2f Lp=%.2f\n",sat,f,
                        D, dL, L, Lp);
                /* use TDCP if looks like valid data */
                //if (fabs(D-dL)<5&&L!=0&&Lp!=0) {
                if (fabs(D-dL)<0&&L!=0&&Lp!=0) {    // disable TDCP
                //if (!(rtk->ssat[sat-1].slip[f]&1)&&L!=0&&Lp!=0) {
                    D=dL;  /* use carrier phase difference */
                    rs=&rs_car[ix*6];
                    azel=&azel_car[ix*2];
                    dts=dts_car[ix*2+1];
                    sig/=rtk->opt.err[4]; /* reduce variance for delta carrier phase */
                    //if (rtk->opt.mode==PMODE_VEL) rtk->ssat[sat-1].outc[f]=0;
                }
                if (L==0||rtk->ssat[sat-1].slip[f]&1) sig*=rtk->opt.err[4];  /* increase variance if no valid phase */
            } else sig*=10;  /* increase variance if gap before last obs */
               // continue;  /* skip if gap before last obs */
        /* LOS (line-of-sight) vector in ECEF */
            cosel=cos(azel[1]);
            a[0]=sin(azel[0])*cosel;
            a[1]=cos(azel[0])*cosel;
            a[2]=sin(azel[1]);
        matmul("TN",3,1,3,1.0,E,a,0.0,e);
        
        /* satellite velocity relative to receiver in ECEF */
        for (j=0;j<3;j++) {
                vs[j]=rs[j+3]-x[j];
        }
        /* range rate with earth rotation correction */
            rate=dot(vs,e,3)+OMGE/CLIGHT*(rs[4]*rr[0]+rs[1]*x[0]-
                                          rs[3]*rr[1]-rs[0]*x[1]);
        /* range rate residual (m/s) */
            v[nv]=(-D*CLIGHT/freq-(rate+x[3]-CLIGHT*dts))/sig;
            //if (x[3]!=rtk->drift) /* if not first iteration */
                trace(3,"sat=%d f=%d: rs[3]=%.6f D=%.2f f=%.1f rate=%.3f x[3]=%.3f dts=%.6f v=%.2f sig=%.4f\n",
                    sat, f, rs[3], D, freq, rate, x[3], dts*1e6, v[nv]*sig, sig);
        /* design matrix */
        for (j=0;j<4;j++) {
            H[j+nv*4]=((j<3)?-e[j]:1.0)/sig;
        }
        /* skip outliers if not first iteration */
        //if (x[3]==rtk->drift||sig>1||fabs(v[nv]*sig) < 0.25)   // 0.5
        if (x[3]==rtk->drift||fabs(v[nv]*sig) < 5.0)   // 0.5
        //if (rtk->epoch<5||fabs(v[nv]*sig) < 5.0)   // 0.5
            nv++;
        else
            trace(3,"outlier: sat=%d f=%d\n",sat, f);
        }
    }
    return nv;
}
/* estimate receiver velocity ------------------------------------------------*/
extern int estvel(rtk_t *rtk, const obsd_t *obs, const int nobs,
                   const int ns, const int *iu,
                   const double *rs_dop, const double *dts_dop, const nav_t *nav,
                   const prcopt_t *opt, sol_t *sol, const double *azel_dop)
{
    double x[4],dx[4],Q[16],vel[3],*v,*H,*var;
    double *rs_car,*dts_car,*azel_car;
    gtime_t time;
    double tt,pos[3],r,e[3],sig=0;
    int i,j,nv=0,svh[MAXOBS*2],info;
//#define MAX_CLOCK_DRIFT 3.0   /* pixel 4xl */
#define MAX_CLOCK_DRIFT 1000.0    /* sm-325f */
#define MAX_UD_VEL 5.0 //2.0
#define MAX_UD_ACCEL 100.0 // 1.0

    v=mat(1,ns*2); H=mat(4,ns*2);
    rs_car=mat(6,nobs); dts_car=mat(2,nobs);azel_car=zeros(2,nobs);
    var=zeros(2,nobs);

    rtk->estvel_fail=1;
    for (i=0;i<3;i++) dx[i]=x[i]=rtk->x[i+3];
    x[3]=rtk->drift;

    /* get azel and range for use with carrier phase measurements */
    trace(3,"estvel: tt=%.3f\n",rtk->tt);
    time=timeadd(sol->time,rtk->tt/2);
    satposs(time,obs,nobs,nav,opt->sateph,rs_car,dts_car,var,svh);
    ecef2pos(sol->rr,pos);
    for (i=0;i<ns;i++) {
        /* calculate sat angles for carrier phase */
        r=geodist(rs_car+iu[i]*6,sol->rr,e);
        satazel(pos,e,azel_car+iu[i]*2);
    }
    
    for (i=0;i<MAXITR;i++) {
        
        /* range rate residuals (m/s) */
        if ((nv=resdop(rtk,obs,ns,iu,opt->nf,rs_dop,dts_dop,azel_dop,rs_car,dts_car,
            azel_car,nav,sol->rr,x,v,H,svh))<4) {
                trace(3,"estvel fail for too few sats: nv=%d\n",nv);
                rtk->estvel_fail=1;
            break;
        }
        /* least square estimation */
        if ((info=lsq(H,v,4,nv,dx,Q))) {
            trace(1,"lsq error info=%d\n",info);
            break;
        }
        for (j=0;j<4;j++) x[j]+=dx[j];
        trace(3,"drift=%.2f x[3]=%.2f diff=%.2f\n",rtk->drift,x[3],x[3]-rtk->drift);
        trace(3,"estvel : vx=%.3f vy=%.3f vz=%.3f, ns=%d nv=%d\n",x[0],x[1],x[2],ns,nv);
        ecef2enu(pos,x,vel);
        trace(3,"   enu : ve=%.3f vn=%.3f vu=%.3f udaccel=%.3f\n",vel[0],vel[1],vel[2],vel[2]-rtk->vel_ud);
        if (norm(dx,4)<1E-6) {
            if ((fabs(x[3]-rtk->drift)<MAX_CLOCK_DRIFT||rtk->drift==0)&&
                (fabs(vel[2]-rtk->vel_ud)<MAX_UD_ACCEL&&fabs(vel[2])<MAX_UD_VEL)) {  /* skip clock jump */
                matcpy(sol->rr+3,x,3,1);
                sol->qv[0]=(float)Q[0];  /* xx */
                sol->qv[1]=(float)Q[5];  /* yy */
                sol->qv[2]=(float)Q[10]; /* zz */
                sol->qv[3]=(float)Q[1];  /* xy */
                sol->qv[4]=(float)Q[6];  /* yz */
                sol->qv[5]=(float)Q[2];  /* zx */
                rtk->estvel_fail=0;
                rtk->acc_ud = rtk->tt==0?0:(vel[2]-rtk->vel_ud)/rtk->tt;
                rtk->vel_ud=vel[2];
            } else {
                trace(3,"estvel fail, drift:=%.2f->%.2f ud_vel=%.2f\n",rtk->drift,x[3],vel[2]);
                nv=0;
                rtk->vel_ud=rtk->acc_ud=0;
                //rtk->drift=0;
            }
            rtk->drift=x[3];         /* save clock drift for next epoch */
            break;
        }
    }
    free(v);free(H);free(var);
    free(rs_car);free(dts_car);free(azel_car);
    return(nv);
}
/* single-point positioning ----------------------------------------------------
* compute receiver position, velocity, clock bias by single-point positioning
* with pseudorange and doppler observables
* args   : obsd_t *obs      I   observation data
*          int    n         I   number of observation data
*          nav_t  *nav      I   navigation data
*          prcopt_t *opt    I   processing options
*          sol_t  *sol      IO  solution
*          double *azel     IO  azimuth/elevation angle (rad) (NULL: no output)
*          ssat_t *ssat     IO  satellite status              (NULL: no output)
*          char   *msg      O   error message for error exit
* return : status(1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int pntpos(const obsd_t *obs, int n, const nav_t *nav,
                  const prcopt_t *opt, sol_t *sol, double *azel, ssat_t *ssat,
                  char *msg)
{
    prcopt_t opt_=*opt;
    double *rs,*dts,*var,*azel_,*resp;
    int i,stat,vsat[MAXOBS]={0},svh[MAXOBS];
    
    trace(3,"pntpos  : tobs=%s n=%d\n",time_str(obs[0].time,3),n);
    
    sol->stat=SOLQ_NONE;
    
    if (n<=0) {
        strcpy(msg,"no observation data");
        return 0;
    }
    sol->time=obs[0].time;
    msg[0]='\0';
    sol->eventime = obs[0].eventime;
    
    rs=mat(6,n); dts=mat(2,n); var=mat(1,n); azel_=zeros(2,n); resp=mat(1,n);
    
    if (ssat) {
        for (i=0;i<MAXSAT;i++) {
            ssat[i].snr_rover[0]=0;
            ssat[i].snr_base[0]=0;
        }
        for (i=0;i<n;i++)
            ssat[obs[i].sat-1].snr_rover[0]=obs[i].SNR[0];
    }
    
    if (opt_.mode!=PMODE_SINGLE) { /* for precise positioning */
        opt_.ionoopt=IONOOPT_BRDC;
        opt_.tropopt=TROPOPT_SAAS;
    }
    /* satellite positions, velocities and clocks */
    satposs(sol->time,obs,n,nav,opt_.sateph,rs,dts,var,svh);
    
    /* estimate receiver position and time with pseudorange */
    stat=estpos(obs,n,rs,dts,var,svh,nav,&opt_,ssat,sol,azel_,vsat,resp,msg);
    
    /* RAIM FDE */
    if (!stat&&n>=6&&opt->posopt[4]) {
        stat=raim_fde(obs,n,rs,dts,var,svh,nav,&opt_,ssat,sol,azel_,vsat,resp,msg);
    }
    /* estimate receiver velocity with Doppler */
/*    if (stat) {
        estvel(obs,n,rs,dts,nav,&opt_,sol,azel_,vsat);
    }  */
    if (azel) {
        for (i=0;i<n*2;i++) azel[i]=azel_[i];
    }
    if (ssat) {
        for (i=0;i<MAXSAT;i++) {
            ssat[i].vs=0;
            ssat[i].azel[0]=ssat[i].azel[1]=0.0;
            ssat[i].resp[0]=ssat[i].resc[0]=0.0;
        }
        for (i=0;i<n;i++) {
            ssat[obs[i].sat-1].azel[0]=azel_[  i*2];
            ssat[obs[i].sat-1].azel[1]=azel_[1+i*2];
            if (!vsat[i]) continue;
            ssat[obs[i].sat-1].vs=1;
            ssat[obs[i].sat-1].resp[0]=resp[i];
        }
    }
    free(rs); free(dts); free(var); free(azel_); free(resp);
    return stat;
}
