/*------------------------------------------------------------------------------
* convbin.c : convert receiver binary log file to rinex obs/nav, sbas messages
*
*          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
*
* options : -DWIN32 use windows file path separator
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 22:13:04 $
* history : 2008/06/22 1.0 new
*           2009/06/17 1.1 support glonass
*           2009/12/19 1.2 fix bug on disable of glonass
*                          fix bug on improper header for rtcm2 and rtcm3
*           2010/07/18 1.3 add option -v, -t, -h, -x
*           2011/01/15 1.4 add option -ro, -hc, -hm, -hn, -ht, -ho, -hr, -ha,
*                            -hp, -hd, -y, -c, -q 
*                          support gw10 and javad receiver, galileo, qzss
*                          support rinex file name convention
*           2012/10/22 1.5 add option -scan, -oi, -ot, -ol
*                          change default rinex version to 2.11
*                          fix bug on default output directory (/ -> .)
*                          support galileo nav (LNAV) output
*                          support compass
*           2012/11/19 1.6 fix bug on setting code mask in rinex options
*           2013/02/18 1.7 support binex
*           2013/05/19 1.8 support auto format for file path with wild-card
*           2014/02/08 1.9 add option -span -trace -mask
*           2014/08/26 1.10 add Trimble RT17 support
*           2014/12/26 1.11 add option -nomask
*           2016/01/23 1.12 enable septentrio
*           2016/05/25 1.13 fix bug on initializing output file paths in
*                           convbin()
*           2016/06/09 1.14 fix bug on output file with -v 3.02
*           2016/07/01 1.15 support log format CMR/CMR+
*           2016/07/31 1.16 add option -halfc
*           2017/05/26 1.17 add input format tersus
*           2017/06/06 1.18 fix bug on output beidou and irnss nav files
*                           add option -tt
*           2018/10/10 1.19 default options are changed.
*                             scan input file: off - on
*                             number of freq: 2 -> 3
*                           add option -noscan
*           2020/11/30 1.20 include NavIC in default systems
*                           force option -scan
*                           delete option -noscan
*                           suppress warnings
*-----------------------------------------------------------------------------*/
#define _POSIX_C_SOURCE 199506
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <sys/stat.h>
#include "rtklib.h"

#define PRGNAME   "CONVBIN"
#define TRACEFILE "convbin.trace"
#define NOUTFILE        9       /* number of output files */

/* help text -----------------------------------------------------------------*/
static const char *help[]={
"",
" Synopsis",
"",
" convbin [option ...] file", 
"",
" Description",
"",
" Convert RTCM, receiver raw data log and RINEX file to RINEX and SBAS/LEX",
" message file. SBAS message file complies with RTKLIB SBAS/LEX message",
" format. It supports the following messages or files:",
"",
" RTCM 2                 : Type 1, 3, 9, 14, 16, 17, 18, 19, 22",
" RTCM 3                 : Type 1002, 1004, 1005, 1006, 1010, 1012, 1019, 1020",
"                          Type 1071-1127 (MSM except for compact msg)",
" NovAtel OEMV/4,OEMStar : RANGECMPB, RANGEB, RAWEPHEMB, IONUTCB, RAWWASSFRAMEB",
" u-blox LEA-4T/5T/6T/8/9: RXM-RAW, RXM-RAWX, RXM-SFRB",
" Swift Piksi Multi      : ",
" Hemisphere             : BIN76, BIN80, BIN94, BIN95, BIN96",
" SkyTraq S1315F         : msg0xDD, msg0xE0, msg0xDC",
" GW10                   : msg0x08, msg0x03, msg0x27, msg0x20",
" Javad                  : [R*],[r*],[*R],[*r],[P*],[p*],[*P],[*p],[D*],[*d],",
"                          [E*],[*E],[F*],[TC],[GE],[NE],[EN],[QE],[UO],[IO],",
"                          [WD]",
" NVS                    : BINR",
" BINEX                  : big-endian, regular CRC, forward record (0xE2)",
"                          0x01-01,0x01-02,0x01-03,0x01-04,0x01-06,0x7f-05",
" Trimble                : RT17",
" Septentrio             : SBF",
" RINEX                  : OBS, NAV, GNAV, HNAV, LNAV, QNAV",
"",
" Options [default]",
"",
"     file         input receiver binary log file",
"     -ts y/m/d h:m:s  start time [all]",
"     -te y/m/d h:m:s  end time [all]",
"     -tr y/m/d h:m:s  approximated time for RTCM",
"     -ti tint     observation data interval (s) [all]",
"     -tt ttol     observation data epoch tolerance (s) [0.005]",
"     -span span   time span (h) [all]",
"     -r format    log format type",
"                  rtcm2= RTCM 2",
"                  rtcm3= RTCM 3",
"                  nov  = NovAtel OEM/4/V/6/7,OEMStar",
"                  ubx  = ublox LEA-4T/5T/6T/7T/M8T/F9",
"                  sbp  = Swift Navigation SBP",
"                  hemis= Hemisphere Eclipse/Crescent",
"                  stq  = SkyTraq S1315F",
"                  javad= Javad GREIS",
"                  nvs  = NVS NV08C BINR",
"                  binex= BINEX",
"                  rt17 = Trimble RT17",
"                  sbf  = Septentrio SBF",
"                  unicore = Unicore binary data output",
"                  rinex= RINEX",
"     -ro opt      receiver options",
"     -f freq      number of frequencies [all]",
"     -hc comment  rinex header: comment line",
"     -hm marker   rinex header: marker name",
"     -hn markno   rinex header: marker number",
"     -ht marktype rinex header: marker type",
"     -ho observ   rinex header: observer name and agency separated by /",
"     -hr rec      rinex header: receiver number, type and version separated by /",
"     -ha ant      rinex header: antenna number and type separated by /",
"     -hp pos      rinex header: approx position x/y/z separated by /",
"     -hd delta    rinex header: antenna delta h/e/n separated by /",
"     -v ver       rinex version [3.04]",
"     -od          include doppler frequency in rinex obs [off]",
"     -os          include snr in rinex obs [off]",
"     -oi          include iono correction in rinex nav header [off]",
"     -ot          include time correction in rinex nav header [off]",
"     -ol          include leap seconds in rinex nav header [off]",
"     -halfc       half-cycle ambiguity correction [off]",
"     -sortsats    sort observations by the RTKLib satellite index [off]",
"     -mask   [sig[,...]] signal mask(s) (sig={G|R|E|J|S|C|I}L{1C|1P|1W|...})",
"     -nomask [sig[,...]] signal no mask (same as above)",
"     -x sat       exclude satellite",
"     -y sys       exclude systems (G:GPS,R:GLO,E:GAL,J:QZS,S:SBS,C:BDS,I:IRN)",
"     --glofcn [-7 to 6][,...]] GLONASS fcn for R01 to R32",
"     -d dir       output directory [same as input file]",
"     -c staid     use RINEX file name convention with staid [off]",
"     -o ofile     output RINEX OBS file",
"     -n nfile     output RINEX NAV file",
"     -g gfile     output RINEX GNAV file",
"     -h hfile     output RINEX HNAV file",
"     -q qfile     output RINEX QNAV file",
"     -l lfile     output RINEX LNAV file",
"     -b cfile     output RINEX CNAV file",
"     -i ifile     output RINEX INAV file",
"     -s sfile     output SBAS message file",
"     -trace level output trace level [off]",
"",
" If any output file specified, default output files (<file>.obs,",
" <file>.nav, <file>.gnav, <file>.hnav, <file>.qnav, <file>.lnav,",
" <file>.cnav, <file>.inav and <file>.sbs) are used. To obtain week number info",
" for RTCM file, use -tr option to specify the approximated log start time.",
" Without -tr option, the program obtains the week number from the time-tag file",
" (if it exists) or the last modified time of the log file instead.",
"",
" If receiver type is not specified, type is recognized by the input",
" file extension as follows.",
"     *.rtcm2       RTCM 2",
"     *.rtcm3       RTCM 3",
"     *.gps         NovAtel OEM4/V/6/7,OEMStar",
"     *.ubx         u-blox LEA-4T/5T/6T/7T/M8T/F9",
"     *.sbp         Swift Navigation SBP",
"     *.bin         Hemisphere Eclipse/Crescent",
"     *.stq         SkyTraq S1315F",
"     *.jps         Javad GREIS",
"     *.bnx,*binex  BINEX",
"     *.rt17        Trimble RT17",
"     *.sbf         Septentrio SBF",
"     *.unc         Unicore binary data output",
"     *.obs,*.*o    RINEX OBS",
"     *.rnx         RINEX OBS",
"     *.nav,*.*n    RINEX NAV",
};
/* print help ----------------------------------------------------------------*/
static void printhelp(void)
{
    int i;
    for (i=0;i<(int)(sizeof(help)/sizeof(*help));i++) fprintf(stderr,"%s\n",help[i]);
    exit(0);
}
/* show message --------------------------------------------------------------*/
extern int showmsg(const char *format, ...)
{
    va_list arg;
    va_start(arg,format); vfprintf(stderr,format,arg); va_end(arg);
    fprintf(stderr,*format?"\r":"\n");
    return 0;
}
/* convert main --------------------------------------------------------------*/
static int convbin(int format, rnxopt_t *opt, const char *ifile, char **file,
                   char *dir)
{
    int i,def;
    static char work[1024],ofile_[NOUTFILE][1024]={"","","","","","","","",""};
    char ifile_[1024],*ofile[NOUTFILE],*p;
    char *extnav=(opt->rnxver<=299||opt->navsys==SYS_GPS)?"N":"P";
    char *extlog="sbs";
    
    /* replace wild-card (*) in input file by 0 */
    strcpy(ifile_,ifile);
    for (p=ifile_;*p;p++) if (*p=='*') *p='0';
    
    def=!file[0]&&!file[1]&&!file[2]&&!file[3]&&!file[4]&&!file[5]&&!file[6]&&
        !file[7]&&!file[8];
    
    for (i=0;i<NOUTFILE;i++) ofile[i]=ofile_[i];
    
    if (file[0]) strcpy(ofile[0],file[0]);
    else if (*opt->staid) {
        strcpy(ofile[0],"%r%n0.%yO");
    }
    else if (def) {
        strcpy(ofile[0],ifile_);
        if ((p=strrchr(ofile[0],'.'))) strcpy(p,".obs");
        else strcat(ofile[0],".obs");
    }
    if (file[1]) strcpy(ofile[1],file[1]);
    else if (*opt->staid) {
        strcpy(ofile[1],"%r%n0.%y");
        strcat(ofile[1],extnav);
    }
    else if (def) {
        strcpy(ofile[1],ifile_);
        if ((p=strrchr(ofile[1],'.'))) strcpy(p,".nav");
        else strcat(ofile[1],".nav");
    }
    if (file[2]) strcpy(ofile[2],file[2]);
    else if (opt->rnxver<=299&&*opt->staid) {
        strcpy(ofile[2],"%r%n0.%yG");
    }
    else if (opt->rnxver<=299&&def) {
        strcpy(ofile[2],ifile_);
        if ((p=strrchr(ofile[2],'.'))) strcpy(p,".gnav");
        else strcat(ofile[2],".gnav");
    }
    if (file[3]) strcpy(ofile[3],file[3]);
    else if (opt->rnxver<=299&&*opt->staid) {
        strcpy(ofile[3],"%r%n0.%yH");
    }
    else if (opt->rnxver<=299&&def) {
        strcpy(ofile[3],ifile_);
        if ((p=strrchr(ofile[3],'.'))) strcpy(p,".hnav");
        else strcat(ofile[3],".hnav");
    }
    if (file[4]) strcpy(ofile[4],file[4]);
    else if (opt->rnxver<=299&&*opt->staid) {
        strcpy(ofile[4],"%r%n0.%yQ");
    }
    else if (opt->rnxver<=299&&def) {
        strcpy(ofile[4],ifile_);
        if ((p=strrchr(ofile[4],'.'))) strcpy(p,".qnav");
        else strcat(ofile[4],".qnav");
    }
    if (file[5]) strcpy(ofile[5],file[5]);
    else if (opt->rnxver<=299&&*opt->staid) {
        strcpy(ofile[5],"%r%n0.%yL");
    }
    else if (opt->rnxver<=299&&def) {
        strcpy(ofile[5],ifile_);
        if ((p=strrchr(ofile[5],'.'))) strcpy(p,".lnav");
        else strcat(ofile[5],".lnav");
    }
    if (file[6]) strcpy(ofile[6],file[6]);
    else if (opt->rnxver<=299&&*opt->staid) {
        strcpy(ofile[6],"%r%n0.%yC");
    }
    else if (opt->rnxver<=299&&def) {
        strcpy(ofile[6],ifile_);
        if ((p=strrchr(ofile[6],'.'))) strcpy(p,".cnav");
        else strcat(ofile[6],".cnav");
    }
    if (file[7]) strcpy(ofile[7],file[7]);
    else if (opt->rnxver<=299&&*opt->staid) {
        strcpy(ofile[7],"%r%n0.%yI");
    }
    else if (opt->rnxver<=299&&def) {
        strcpy(ofile[7],ifile_);
        if ((p=strrchr(ofile[7],'.'))) strcpy(p,".inav");
        else strcat(ofile[7],".inav");
    }
    if (file[8]) strcpy(ofile[8],file[8]);
    else if (*opt->staid) {
        strcpy(ofile[8],"%r%n0_%y.");
        strcat(ofile[8],extlog);
    }
    else if (def) {
        strcpy(ofile[8],ifile_);
        if ((p=strrchr(ofile[8],'.'))) strcpy(p,".");
        else strcat(ofile[8],".");
        strcat(ofile[8],extlog);
    }
    for (i=0;i<NOUTFILE;i++) {
        if (!*dir||!*ofile[i]) continue;
        if ((p=strrchr(ofile[i],RTKLIB_FILEPATHSEP))) strcpy(work,p+1);
        else strcpy(work,ofile[i]);
        sprintf(ofile[i],"%s%c%s",dir,RTKLIB_FILEPATHSEP,work);
    }
    fprintf(stderr,"input file  : %s (%s)\n",ifile,formatstrs[format]);
    
    if (*ofile[0]) fprintf(stderr,"->rinex obs : %s\n",ofile[0]);
    if (*ofile[1]) fprintf(stderr,"->rinex nav : %s\n",ofile[1]);
    if (*ofile[2]) fprintf(stderr,"->rinex gnav: %s\n",ofile[2]);
    if (*ofile[3]) fprintf(stderr,"->rinex hnav: %s\n",ofile[3]);
    if (*ofile[4]) fprintf(stderr,"->rinex qnav: %s\n",ofile[4]);
    if (*ofile[5]) fprintf(stderr,"->rinex lnav: %s\n",ofile[5]);
    if (*ofile[6]) fprintf(stderr,"->rinex cnav: %s\n",ofile[6]);
    if (*ofile[7]) fprintf(stderr,"->rinex inav: %s\n",ofile[7]);
    if (*ofile[8]) fprintf(stderr,"->sbas log  : %s\n",ofile[8]);
    
    if (!convrnx(format,opt,ifile,ofile)) {
        fprintf(stderr,"\n");
        return 0;
    }
    fprintf(stderr,"\n");
    return 1;
}
/* set signal mask -----------------------------------------------------------*/
static void setmask(const char *argv, rnxopt_t *opt, int mask)
{
    char buff[1024],*p;
    int i;
    
    strcpy(buff,argv);
    char *r;
    for (p=strtok_r(buff,",",&r);p;p=strtok_r(NULL,",",&r)) {
        if (strlen(p)<4||p[1]!='L') continue;
        if      (p[0]=='G') i=RNX_SYS_GPS;
        else if (p[0]=='R') i=RNX_SYS_GLO;
        else if (p[0]=='E') i=RNX_SYS_GAL;
        else if (p[0]=='J') i=RNX_SYS_QZS;
        else if (p[0]=='S') i=RNX_SYS_SBS;
        else if (p[0]=='C') i=RNX_SYS_CMP;
        else if (p[0]=='I') i=RNX_SYS_IRN;
        else continue;
        int code=obs2code(p+2);
        if (code != CODE_NONE) {
            opt->mask[i][code-1]=mask?'1':'0';
        }
    }
}
// Set GLONASS fcn -----------------------------------------------------------
static void setglofcn(const char *argv, rnxopt_t *opt) {
  char buff[1024];
  strncpy(buff, argv, sizeof(buff));
  buff[1023] = '\0';
  char *p = buff;
  for (int i = 0; i < 32; i++) {
    if (p == NULL) break;
    char *fcnstr = p;
    for (;;) {
      int c = *p++;
      if (c == ',') {
        p[-1] = '\0';
        break;
      }
      if (c == '\0') {
        p = NULL;
        break;
      }
    }
    if (strlen(fcnstr) < 1) continue;
    int fcn;
    int r = sscanf(fcnstr, "%d", &fcn);
    if (r != 1) {
      fprintf(stderr, "GLONASS R%02d fcn invalid '%s'\n", i + 1, fcnstr);
      continue;
    }
    if (fcn < -7 || fcn > 6) {
      fprintf(stderr, "GLONASS R%02d fcn %d out of range [-7 to 6]\n", i + 1, fcn);
      continue;
    }
    opt->glofcn[i] = fcn + 8;
  }
}
/* get start time of input file -----------------------------------------------*/
static int get_filetime(const char *file, gtime_t *time)
{
    FILE *fp;
    uint32_t time_time;
    uint8_t buff[64];
    char path[1024],*paths[1],path_tag[1024];

    paths[0]=path;
    
    if (!expath(file,paths,1)) return 0;
    
    /* get start time of time-tag file */
    sprintf(path_tag,"%.1019s.tag",path);
    if ((fp=fopen(path_tag,"rb"))) {
        if (fread(buff,64,1,fp)==1&&!strncmp((char *)buff,"TIMETAG",7)&&
            fread(&time_time,4,1,fp)==1) {
            time->time=time_time; 
            time->sec=0.0;
            fclose(fp);
            return 1;
        }
        fclose(fp);
    }
    /* Get modified time of input file. */
    struct stat st;
    if (!stat(path, &st)) {
        struct tm tm;
        if (gmtime_r(&st.st_mtime, &tm)) {
          double ep[6];
          ep[0] = tm.tm_year + 1900;
          ep[1] = tm.tm_mon + 1;
          ep[2] = tm.tm_mday;
          ep[3] = tm.tm_hour;
          ep[4] = tm.tm_min;
          ep[5] = tm.tm_sec;
          *time = utc2gpst(epoch2time(ep));
          return 1;
        }
    }
    return 0;
}
/* parse command line options ------------------------------------------------*/
static int cmdopts(int argc, char **argv, rnxopt_t *opt, char **ifile,
                   char **ofile, char **dir, int *trace)
{
    double eps[]={1980,1,1,0,0,0},epe[]={2037,12,31,0,0,0};
    double epr[]={2010,1,1,0,0,0},span=0.0;
    int i,j,k,sat,nf=6,format=-1;
    char *p,*sys,*fmt="",*paths[1],path[1024],buff[256];
    
    opt->rnxver=304;
    opt->obstype=OBSTYPE_PR|OBSTYPE_CP;
    opt->navsys=SYS_GPS|SYS_GLO|SYS_GAL|SYS_QZS|SYS_SBS|SYS_CMP|SYS_IRN;
    opt->ttol = 0.005;
    
    for (i=0;i<RNX_NUMSYS;i++) {
        for (j=0;j<MAXCODE;j++) opt->mask[i][j]='1';
        opt->mask[i][MAXCODE]='\0';
    }
    
    for (i=1;i<argc;i++) {
        if (!strcmp(argv[i],"-ts")&&i+2<argc) {
            sscanf(argv[++i],"%lf/%lf/%lf",eps,eps+1,eps+2);
            sscanf(argv[++i],"%lf:%lf:%lf",eps+3,eps+4,eps+5);
            opt->ts=epoch2time(eps);
        }
        else if (!strcmp(argv[i],"-te")&&i+2<argc) {
            sscanf(argv[++i],"%lf/%lf/%lf",epe,epe+1,epe+2);
            sscanf(argv[++i],"%lf:%lf:%lf",epe+3,epe+4,epe+5);
            opt->te=epoch2time(epe);
        }
        else if (!strcmp(argv[i],"-tr")&&i+2<argc) {
            sscanf(argv[++i],"%lf/%lf/%lf",epr,epr+1,epr+2);
            sscanf(argv[++i],"%lf:%lf:%lf",epr+3,epr+4,epr+5);
            opt->trtcm=epoch2time(epr);
        }
        else if (!strcmp(argv[i],"-ti")&&i+1<argc) {
            opt->tint=atof(argv[++i]);
        }
        else if (!strcmp(argv[i],"-tt")&&i+1<argc) {
            opt->ttol=atof(argv[++i]);
        }
        else if (!strcmp(argv[i],"-span")&&i+1<argc) {
            span=atof(argv[++i]);
        }
        else if (!strcmp(argv[i],"-r" )&&i+1<argc) {
            fmt=argv[++i];
        }
        else if (!strcmp(argv[i],"-ro")&&i+1<argc) {
            strcpy(opt->rcvopt,argv[++i]);
        }
        else if (!strcmp(argv[i],"-f" )&&i+1<argc) {
            nf=atoi(argv[++i]);
        }
        else if (!strcmp(argv[i],"-hc")&&i+1<argc) {
            rnxcomment(opt, argv[++i]);
        }
        else if (!strcmp(argv[i],"-hm")&&i+1<argc) {
            strcpy(opt->marker,argv[++i]);
        }
        else if (!strcmp(argv[i],"-hn")&&i+1<argc) {
            strcpy(opt->markerno,argv[++i]);
        }
        else if (!strcmp(argv[i],"-ht")&&i+1<argc) {
            strcpy(opt->markertype,argv[++i]);
        }
        else if (!strcmp(argv[i],"-ho")&&i+1<argc) {
            strcpy(buff,argv[++i]);
            char *r;
            for (j=0,p=strtok_r(buff,"/",&r);j<2&&p;j++,p=strtok_r(NULL,"/",&r)) {
                strcpy(opt->name[j],p);
            }
        }
        else if (!strcmp(argv[i],"-hr")&&i+1<argc) {
            strcpy(buff,argv[++i]);
            char *r;
            for (j=0,p=strtok_r(buff,"/",&r);j<3&&p;j++,p=strtok_r(NULL,"/",&r)) {
                strcpy(opt->rec[j],p);
            }
        }
        else if (!strcmp(argv[i],"-ha")&&i+1<argc) {
            strcpy(buff,argv[++i]);
            char *r;
            for (j=0,p=strtok_r(buff,"/",&r);j<3&&p;j++,p=strtok_r(NULL,"/",&r)) {
                strcpy(opt->ant[j],p);
            }
        }
        else if (!strcmp(argv[i],"-hp")&&i+1<argc) {
            strcpy(buff,argv[++i]);
            char *r;
            for (j=0,p=strtok_r(buff,"/",&r);j<3&&p;j++,p=strtok_r(NULL,"/",&r)) {
                opt->apppos[j]=atof(p);
            }
        }
        else if (!strcmp(argv[i],"-hd")&&i+1<argc) {
            strcpy(buff,argv[++i]);
            char *r;
            for (j=0,p=strtok_r(buff,"/",&r);j<3&&p;j++,p=strtok_r(NULL,"/",&r)) {
                opt->antdel[j]=atof(p);
            }
        }
        else if (!strcmp(argv[i],"-v" )&&i+1<argc) {
            opt->rnxver=(int)(atof(argv[++i])*100.0);
        }
        else if (!strcmp(argv[i],"-od")) {
            opt->obstype|=OBSTYPE_DOP;
        }
        else if (!strcmp(argv[i],"-os")) {
            opt->obstype|=OBSTYPE_SNR;
        }
        else if (!strcmp(argv[i],"-oi")) {
            opt->outiono=1;
        }
        else if (!strcmp(argv[i],"-ot")) {
            opt->outtime=1;
        }
        else if (!strcmp(argv[i],"-ol")) {
            opt->outleaps=1;
        }
        else if (!strcmp(argv[i],"-scan")) {
            /* obsolete */ ;
        }
        else if (!strcmp(argv[i],"-halfc")) {
            opt->halfcyc=1;
        }
        else if (!strcmp(argv[i],"-sortsats")) {
            opt->sortsats=1;
        }
        else if (!strcmp(argv[i],"-mask")&&i+1<argc) {
            for (j=0;j<RNX_NUMSYS;j++) {
              for (k=0;k<MAXCODE;k++) opt->mask[j][k]='0';
              opt->mask[j][MAXCODE]='\0';
            }
            setmask(argv[++i],opt,1);
        }
        else if (!strcmp(argv[i],"-nomask")&&i+1<argc) {
            setmask(argv[++i],opt,0);
        }
        else if (!strcmp(argv[i],"-x" )&&i+1<argc) {
            if ((sat=satid2no(argv[++i]))) opt->exsats[sat-1]=1;
        }
        else if (!strcmp(argv[i],"-y" )&&i+1<argc) {
            sys=argv[++i];
            if      (!strcmp(sys,"G")) opt->navsys&=~SYS_GPS;
            else if (!strcmp(sys,"R")) opt->navsys&=~SYS_GLO;
            else if (!strcmp(sys,"E")) opt->navsys&=~SYS_GAL;
            else if (!strcmp(sys,"J")) opt->navsys&=~SYS_QZS;
            else if (!strcmp(sys,"S")) opt->navsys&=~SYS_SBS;
            else if (!strcmp(sys,"C")) opt->navsys&=~SYS_CMP;
            else if (!strcmp(sys,"I")) opt->navsys&=~SYS_IRN;
        }
        else if (!strcmp(argv[i], "--glofcn") && i + 1 < argc) {
            setglofcn(argv[++i], opt);
        }
        else if (!strcmp(argv[i],"-d" )&&i+1<argc) {
            *dir=argv[++i];
        }
        else if (!strcmp(argv[i],"-c" )&&i+1<argc) {
            strcpy(opt->staid,argv[++i]);
        }
        else if (!strcmp(argv[i],"-o" )&&i+1<argc) ofile[0]=argv[++i];
        else if (!strcmp(argv[i],"-n" )&&i+1<argc) ofile[1]=argv[++i];
        else if (!strcmp(argv[i],"-g" )&&i+1<argc) ofile[2]=argv[++i];
        else if (!strcmp(argv[i],"-h" )&&i+1<argc) ofile[3]=argv[++i];
        else if (!strcmp(argv[i],"-q" )&&i+1<argc) ofile[4]=argv[++i];
        else if (!strcmp(argv[i],"-l" )&&i+1<argc) ofile[5]=argv[++i];
        else if (!strcmp(argv[i],"-b" )&&i+1<argc) ofile[6]=argv[++i];
        else if (!strcmp(argv[i],"-i" )&&i+1<argc) ofile[7]=argv[++i];
        else if (!strcmp(argv[i],"-s" )&&i+1<argc) ofile[8]=argv[++i];
        else if (!strcmp(argv[i],"-trace" )&&i+1<argc) {
            *trace=atoi(argv[++i]);
        }
        else if (!strcmp(argv[i], "--version")) {
            fprintf(stderr, "convbin RTKLIB %s %s\n", VER_RTKLIB, PATCH_LEVEL);
            exit(0);
        }
        else if (!strncmp(argv[i],"-",1)) printhelp();
        
        else *ifile=argv[i];
    }
    if (span>0.0&&opt->ts.time) {
        opt->te=timeadd(opt->ts,span*3600.0-1e-3);
    }
    if (nf>=1) opt->freqtype|=FREQTYPE_L1;
    if (nf>=2) opt->freqtype|=FREQTYPE_L2;
    if (nf>=3) opt->freqtype|=FREQTYPE_L3;
    if (nf>=4) opt->freqtype|=FREQTYPE_L4;
    if (nf>=5) opt->freqtype|=FREQTYPE_L5;
    if (nf>=6) opt->freqtype|=FREQTYPE_L6;
    if (nf>=7) opt->freqtype|=FREQTYPE_ALL;
    
    if (opt->trtcm.time == 0) {
        // Use the start or end time if supplied. Otherwise use the file time.
        if (opt->ts.time != 0) opt->trtcm = opt->ts;
        else if (opt->te.time != 0) opt->trtcm = opt->te;
        else get_filetime(*ifile, &opt->trtcm);
    }
    if (*fmt) {
        if      (!strcmp(fmt,"rtcm2")) format=STRFMT_RTCM2;
        else if (!strcmp(fmt,"rtcm3")) format=STRFMT_RTCM3;
        else if (!strcmp(fmt,"nov"  )) format=STRFMT_OEM4;
        /* else if (!strcmp(fmt,"cnav" )) format=STRFMT_CNAV; */
        else if (!strcmp(fmt,"ubx"  )) format=STRFMT_UBX;
        else if (!strcmp(fmt,"sbp"  )) format=STRFMT_SBP;
        else if (!strcmp(fmt,"hemis")) format=STRFMT_CRES;
        else if (!strcmp(fmt,"stq"  )) format=STRFMT_STQ;
        else if (!strcmp(fmt,"javad")) format=STRFMT_JAVAD;
        else if (!strcmp(fmt,"nvs"  )) format=STRFMT_NVS;
        else if (!strcmp(fmt,"binex")) format=STRFMT_BINEX;
        else if (!strcmp(fmt,"rt17" )) format=STRFMT_RT17;
        else if (!strcmp(fmt,"sbf"  )) format=STRFMT_SEPT;
        else if (!strcmp(fmt,"unicore")) format=STRFMT_UNICORE;
        /* else if (!strcmp(fmt,"tersus")) format=STRFMT_TERSUS; */
        else if (!strcmp(fmt,"rinex")) format=STRFMT_RINEX;
    }
    else {
        paths[0]=path;
        if (!expath(*ifile,paths,1)||!(p=strrchr(path,'.'))) return -1;
        if      (!strcmp(p,".rtcm2"))  format=STRFMT_RTCM2;
        else if (!strcmp(p,".rtcm3"))  format=STRFMT_RTCM3;
        else if (!strcmp(p,".gps"  ))  format=STRFMT_OEM4;
        /* else if (!strcmp(p,"cnav" )) format=STRFMT_CNAV; */
        else if (!strcmp(p,".ubx"  ))  format=STRFMT_UBX;
        else if (!strcmp(p,".sbp"  ))  format=STRFMT_SBP;
        else if (!strcmp(p,".bin"  ))  format=STRFMT_CRES;
        else if (!strcmp(p,".stq"  ))  format=STRFMT_STQ;
        else if (!strcmp(p,".jps"  ))  format=STRFMT_JAVAD;
        else if (!strcmp(p,".bnx"  ))  format=STRFMT_BINEX;
        else if (!strcmp(p,".binex"))  format=STRFMT_BINEX;
        else if (!strcmp(p,".rt17" ))  format=STRFMT_RT17;
        else if (!strcmp(p,".sbf"  ))  format=STRFMT_SEPT;
        else if (!strcmp(p,".unc"  ))  format=STRFMT_UNICORE;
        /* else if (!strcmp(p,".trs"  ))  format=STRFMT_TERSUS; */
        else if (!strcmp(p,".obs"  ))  format=STRFMT_RINEX;
        else if (!strcmp(p+3,"o"   ))  format=STRFMT_RINEX;
        else if (!strcmp(p+3,"O"   ))  format=STRFMT_RINEX;
        else if (!strcmp(p,".rnx"  ))  format=STRFMT_RINEX;
        else if (!strcmp(p,".nav"  ))  format=STRFMT_RINEX;
        else if (!strcmp(p+3,"n"   ))  format=STRFMT_RINEX;
        else if (!strcmp(p+3,"N"   ))  format=STRFMT_RINEX;
    }
    return format;
}
/* main ----------------------------------------------------------------------*/
int main(int argc, char **argv)
{
    rnxopt_t opt={{0}};
    int format,trace=0,stat;
    char *ifile="",*ofile[NOUTFILE]={0},*dir="";
    
    /* parse command line options */
    format=cmdopts(argc,argv,&opt,&ifile,ofile,&dir,&trace);
    
    if (!*ifile) {
        fprintf(stderr,"no input file\n");
        return EXIT_FAILURE;
    }
    if (format<0) {
        fprintf(stderr,"input format can not be recognized\n");
        return EXIT_FAILURE;
    }
    sprintf(opt.prog,"%s %s %s",PRGNAME,VER_RTKLIB,PATCH_LEVEL);
    if (trace>0) {
        traceopen(TRACEFILE);
        tracelevel(trace);
    }
    stat=convbin(format,&opt,ifile,ofile,dir);
    
    traceclose();
    
    return stat?0:EXIT_FAILURE;
}
