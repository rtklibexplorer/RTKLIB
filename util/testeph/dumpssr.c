/*------------------------------------------------------------------------------
* dumpssr.c : dump ssr messages in rtcm log
*
* 2010/06/10 new
*-----------------------------------------------------------------------------*/
#include <stdio.h>
#include "rtklib.h"

/* print ssr messages --------------------------------------------------------*/
static void printhead(int topt, int mopt)
{
    int i;

    printf("%% %s  SAT ",topt?"  DAY      TIME  ":"   GPST  ");

    if (mopt&1) {
        printf(" UDI IOD URA REF ");
    }
    if (mopt&2) {
        printf("%8s %8s %8s %8s %8s %8s ","DR","DA","DC","DDR","DDA","DDC");
    }
    if (mopt&4) {
        printf("%8s %8s %8s %8s ","DCLK","DDCLK","DDDCLK","HRCLK");
    }
    if (mopt&8) {
        for (i=0;i<12;i++) printf("   B%02d ",i+1);
    }
    printf("\n");
}
/* print ssr messages --------------------------------------------------------*/
static void printssrmsg(int sat, const ssr_t *ssr, int topt, int mopt)
{
    double tow;
    int week;
    char tstr[40],id[8];

    if (topt) {
        time2str(ssr->t0[0],tstr,0);
        printf("%s ",tstr);
    }
    else {
        tow=time2gpst(ssr->t0[0],&week);
        printf("%4d %6.0f ",week,tow);
    }
    satno2id(sat,id);
    printf("%4s ",id);

    if (mopt&1) {
        printf("%4.0f %3d %3d %3d ", ssr->udi[0], ssr->iode, ssr->ura, ssr->refd);
    }
    if (mopt&2) {
        printf("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",ssr->deph[0],ssr->deph[1],
               ssr->deph[2],ssr->ddeph[0],ssr->ddeph[1],ssr->ddeph[2]);
    }
    if (mopt&4) {
        printf("%8.3f %8.3f %8.3f %8.3f ",ssr->dclk[0],ssr->dclk[1],ssr->dclk[2],
               ssr->hrclk);
    }
    if (mopt&8) {
        printf("%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f ",
               ssr->cbias[0],ssr->cbias[1],ssr->cbias[2],ssr->cbias[3],
               ssr->cbias[4],ssr->cbias[5],ssr->cbias[6],ssr->cbias[7],
               ssr->cbias[8],ssr->cbias[9],ssr->cbias[10],ssr->cbias[11]);
    }

    printf("\n");
}

static int compute_index(const int n, const int m, const int n_degree) {
    // Compute the index for spherical harmonics coefficients
    return n - m + m * (n_degree + 1) - (m * (m - 1)) / 2;
}

static int compute_update_interval_code(const int update_interval_s) {
    // Convert update interval code to seconds
    switch (update_interval_s) {
        case     1: return  0;
        case     2: return  1;
        case     5: return  2;
        case    10: return  3;
        case    15: return  4;
        case    30: return  5;
        case    60: return  6;
        case   120: return  7;
        case   240: return  8;
        case   300: return  9;
        case   600: return 10;
        case   900: return 11;
        case  1800: return 12;
        case  3600: return 13;
        case  7200: return 14;
        case 10800: return 15;
        default: return 6;
    }
}

/* Print the ION SSR using BNC format

See format description in Section 2.8 of
https://software.rtcm-ntrip.org/export/HEAD/ntrip/trunk/BNC/src/bnchelp.html#correct

Sample format:

> VTEC 2022 10 01 23 59 45.0 6 1 UNKNOWN
 1  3  2   450000.0
   22.6900     0.0000     0.0000
    0.2000    11.6050     0.0000
   -8.7500    -0.0200     2.0250
   -0.2900    -1.5900     0.3700
    0.0000     0.0000     0.0000
    0.0000     1.5350     0.0000
    0.0000     0.1150    -1.3850
    0.0000    -2.0150     0.2500
*/
static void print_vtec_ssr_bnc_format(
    FILE *fp, const rtcm_t* rtcm, const char *stream) {

    double ep[6];
    int year, month, day, hour, min;
    int update_interval_code;
    double sec;
    int idx;
    double coeff;

    const ssr_ion_t *ssr_ion = &rtcm->ssr_ion;
    const gtime_t *gtime = &rtcm->time;

    if (!ssr_ion || ssr_ion->n_layers == 0) {
        return; /* No ionospheric data available */
    }

    /* Convert GPS time to calendar time */
    time2epoch(*gtime, ep);
    year = (int)ep[0];
    month = (int)ep[1];
    day = (int)ep[2];
    hour = (int)ep[3];
    min = (int)ep[4];
    sec = ep[5];

    update_interval_code = compute_update_interval_code(ssr_ion->update_interval_s);

    /* Print BNC VTEC header */
    fprintf(fp, "> VTEC %04d %02d %02d %02d %02d %04.1f %d %d %s\n",
            year, month, day, hour, min, sec, update_interval_code, ssr_ion->n_layers, stream);

    for (int i = 0; i < ssr_ion->n_layers; i++) {
        const ionlayer_sphharm_t *layer = &ssr_ion->ionlayers_sph_harm[i];

        if (layer->n_degree <= 0 || layer->m_order <= 0) {
            continue; /* Skip invalid layers */
        }

        /* Print layer header: layer_id, n_degree, m_order, height_km */
        fprintf(fp, "%2d %2d %2d %10.1f\n",
                i + 1, layer->n_degree, layer->m_order, layer->height_km * 1000.0);

        /* Print cosine coefficients */
        for (int n = 0; n <= layer->n_degree; n++) {
            for (int m = 0; m <= layer->m_order; m++) {
                double coeff = 0.0;
                if (m <= n) {
                    idx = compute_index(n, m, layer->n_degree);
                    coeff = layer->cos_coeff[idx];
                }
                if (m > 0)
                    fprintf(fp, " ");
                fprintf(fp, "%10.4f", coeff);
            }
            fprintf(fp, "\n");
        }

        /* Print sine coefficients */
        for (int n = 0; n <= layer->n_degree; n++) {
            for (int m = 0; m <= layer->m_order; m++) {
                coeff = 0.0;

                if (n > 0 && m > 0 && m <= n) {  // First row and col always zero
                    idx = compute_index(n - 1, m - 1, layer->n_degree - 1);
                    coeff = layer->sin_coeff[idx];
                }
                if (m > 0)
                    fprintf(fp, " ");

                fprintf(fp, "%10.4f", coeff);
            }
            fprintf(fp, "\n");
        }
    }
}


/* dump ssr messages ---------------------------------------------------------*/
static void dumpssrmsg(FILE *fp, int sat, int topt, int mopt, const char *stream)
{
    static rtcm_t rtcm;
    static gtime_t t0[MAXSAT]={{0}};
    int i,stat;

    init_rtcm(&rtcm);

    while ((stat=input_rtcm3f(&rtcm,fp))>=0) {

        if (stat!=10) continue; /* ssr message */

        for (i=0;i<MAXSAT;i++) {
            if (timediff(rtcm.ssr[i].t0[0],t0[i])==0.0) continue;
            t0[i]=rtcm.ssr[i].t0[0];

            if (!sat||i+1==sat) {
                printssrmsg(i+1,rtcm.ssr+i,topt,mopt);
            }
        }
        if (mopt&16) {
            print_vtec_ssr_bnc_format(stdout, &rtcm, stream);
        }
    }
}
/* main ----------------------------------------------------------------------*/
int main(int argc, char **argv)
{
    const char *usage="dumpssr [-t][-s sat][-i][-o][-c][-b][--vtec][--stream name][-h][-x tr] file";

    FILE *fp;
    char *file="";
    char *stream="UNKNOWN";
    int i,sat=0,topt=0,mopt=0,trl=0;

    for (i=0;i<argc;i++) {
        if      (!strcmp(argv[i],"-t")) topt =1;
        else if (!strcmp(argv[i],"-i")) mopt|=1;
        else if (!strcmp(argv[i],"-o")) mopt|=2;
        else if (!strcmp(argv[i],"-c")) mopt|=4;
        else if (!strcmp(argv[i],"-b")) mopt|=8;
        else if (!strcmp(argv[i],"--vtec")) mopt|=16;  /* VTEC */
        else if (!strcmp(argv[i],"-s")&&i+1<argc) sat=atoi(argv[++i]);
        else if (!strcmp(argv[i],"-x")&&i+1<argc) trl=atoi(argv[++i]);
        else if (!strcmp(argv[i],"--stream")&&i+1<argc) stream = argv[++i];
        else if (!strcmp(argv[i],"-h")) {
            fprintf(stderr,"usage: %s\n",usage);
            return 0;
        }
        else file=argv[i];
    }
    if (!mopt) mopt=0xFF;

    if (!(fp=fopen(file,"rb"))) {
        fprintf(stderr,"file open error: %s\n",file);
        return -1;
    }
    if (trl>0) {
        traceopen("dumpssr.trace");
        tracelevel(trl);
    }
    printhead(topt,mopt);

    dumpssrmsg(fp, sat, topt, mopt, stream);

    fclose(fp);
    traceclose();

    return 0;
}
