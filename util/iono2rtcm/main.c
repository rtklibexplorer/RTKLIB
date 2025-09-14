//-----------------------------------------------------------------------------
// rnx2rtcm.c : RINEX to RTCM converter
//
//          Copyright (C) 2012 by T.TAKASU, All rights reserved.
//
// Version : $Revision: 1.1 $ $Date: 2008/07/17 21:55:16 $
// History : 2012/12/12  1.0 new
//-----------------------------------------------------------------------------
#include "rtklib.h"

#define TRACEFILE "iono2rtcm.trace"  // Debug trace file
#define MIN(x,y)    ((x)<(y)?(x):(y))

// Print usage ---------------------------------------------------------------
static const char *help[] = {"",
                             "usage: iono2rtcm [options] [infile]",
                             "",
                             "options:",
                             "  -out outfile        output RTCM file",
                             "  -x   level          debug trace level",
                             ""};
static void print_help(void) {
    for (int i = 0; i < sizeof(help) / sizeof(*help); i++) fprintf(stderr, "%s\n", help[i]);
    exit(0);
}

static int compute_index(const int n, const int m, const int n_degree) {
    // Compute the index for spherical harmonics coefficients
    return n - m + m * (n_degree + 1) - (m * (m - 1)) / 2;
}

static int compute_update_interval(const int update_interval_code) {

    // Convert update interval code to seconds
    switch (update_interval_code) {
        case 0: return 1;
        case 1: return 2;
        case 2: return 5;
        case 3: return 10;
        case 4: return 15;
        case 5: return 30;
        case 6: return 60;
        case 7: return 120;
        case 8: return 240;
        case 9: return 300;
        case 10: return 600;
        case 11: return 900;
        case 12: return 1800;
        case 13: return 3600;
        case 14: return 7200;
        case 15: return 10800;
        default: return 60;  // Default to 1 minute if unknown
    }

}

/* This function parses the BNC VTEC file and fills the rtcm structure.

The format of the BNC VTEC file for an example of N(degree)=3 and M(order)=2

C_00  0    0
C_10 C_11  0
C_20 C_21 C_22
C_30 C_31 C_32
 0    0    0
 0   S_11  0
 0   S_21 S_22
 0   S_31 S_32

Note however than in the RTCM3 the coefficients are transposed
*/
static int parse_bnc_vtec(const char *filename, rtcm_t *rtcm) {

    FILE *fp;
    char line[1024], mountpoint[128], *p;
    int n_degree = 0, m_order = 0;
    int year, month, day, hour, min, n_blocks, layer_number;
    int update_interval_code;
    double sec, height_m;
    gtime_t time;

    trace(3, "parse_bnc_vtec: file=%s\n", filename);

    if (!(fp = fopen(filename, "r"))) {
        trace(1, "parse_bnc_vtec: file open error %s\n", filename);
        return -1;
    }

    // Initialize ionospheric SSR structure
    memset(&rtcm->ssr_ion, 0, sizeof(ssr_ion_t));

    while (fgets(line, sizeof(line), fp)) {
        if (line[0] != '>') continue; // Skip non-header lines initially

        if (strstr(line, "VTEC") == NULL) {
            continue; // Skip non-VTEC lines
        }
        // Parse header line: > VTEC 2025 07 15 10 00 45.0 2 1 IONO00IGS1
        if (sscanf(line, "> VTEC %d %d %d %d %d %lf %d %d %127s",
                  &year, &month, &day, &hour, &min, &sec, &update_interval_code, &n_blocks, mountpoint) != 9) {
            trace(1, "parse_bnc_vtec: invalid header format\n");
            continue;
        }

        time = epoch2time((double[]){year, month, day, hour, min, sec});
        rtcm->time = time;

        rtcm->ssr_ion.update_interval_s = compute_update_interval(update_interval_code);;

        trace(4, "VTEC header: %04d/%02d/%02d %02d:%02d:%05.2f update_interval=%d n_blocks=%d\n",
              year, month, day, hour, min, sec,
              rtcm->ssr_ion.update_interval_s, n_blocks);

        // Read next line with degree and order info
        if (fgets(line, sizeof(line), fp) == NULL) {
            trace(1, "parse_bnc_vtec: invalid BNC format\n");
            continue;
        }

        if (sscanf(line, " %d %d %d %lf", &layer_number, &n_degree, &m_order, &height_m) != 4) {
            trace(4, "Invalid format for layer header [ %s ]\n", line);
            continue;

        } else {
            trace(4, "Spherical harmonics: degree=%d order=%d height(m)=%.1f\n",
                  n_degree, m_order, height_m);
        }

        // Validate degree and order limits
        if (n_degree > 15) n_degree = 15;
        if (m_order > 15) m_order = 15;

        // Set up ionospheric layer
        ssr_ion_t* ssr_ion = &rtcm->ssr_ion;
        ionlayer_sphharm_t *layer = &ssr_ion->ionlayers_sph_harm[0];

        ssr_ion->n_layers = 1; // Assuming only one layer
        layer->height_km = height_m / 1000.0; // Convert height from meters to kilometers
        layer->n_degree = n_degree;
        layer->m_order = m_order;

        // Initialize coefficients
        memset(layer->cos_coeff, 0, sizeof(layer->cos_coeff));
        memset(layer->sin_coeff, 0, sizeof(layer->sin_coeff));

        // Read cosine coefficients (C_nm)
        for (int n = 0; n <= layer->n_degree; n++) {
            if (!fgets(line, sizeof(line), fp)) break;

            p = line;

            for (int m = 0; m <= MIN(n, layer->m_order); m++) {
                double coeff;
                if (sscanf(p, "%lf", &coeff) == 1) {
                    int index = compute_index(n, m, layer->n_degree);
                    layer->cos_coeff[index] = coeff;
                    // Skip to next coefficient (11 characters for each coefficient)
                    p += 11;
                } else {
                    break;
                }
            }

        }

        // Read sine coefficients (S_nm) - skip n=0 terms (they don't exist)
        for (int n = 0; n <= layer->n_degree; n++) {
            if (!fgets(line, sizeof(line), fp)) break;

            if (n == 0) continue; // Skip S_n0 (doesn't exist), start from m=1

            p = line + 11; // Skip first coefficient (S_0m)
            for (int m = 1; m <= MIN(n, layer->m_order); m++) {

                double coeff;
                if (sscanf(p, "%lf", &coeff) == 1) {
                    int index = compute_index(n - 1, m - 1, layer->n_degree - 1);
                    layer->sin_coeff[index] = coeff;
                    // Skip to next number
                    p += 11;
                } else {
                    break;
                }
            }
        }
    }

    fclose(fp);
    return 0;
}

// Main ----------------------------------------------------------------------
int main(int argc, char **argv) {

    int ret = 1;
    char *infile = NULL, *outfile = "";
    int trlevel = 0;

    rtcm_t rtcm = {0};

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-out") && i + 1 < argc)
            outfile = argv[++i];

        else if (!strcmp(argv[i], "-x") && i + 1 < argc)
            trlevel = atoi(argv[++i]);

        else if (!strncmp(argv[i], "-", 1))
            print_help();

        else
            infile = argv[i];
    }

    if (trlevel > 0) {
        traceopen(TRACEFILE);
        tracelevel(trlevel);
    }

    if (infile == NULL) {
        fprintf(stderr, "Input file is required.\n");
        print_help();
        return 2;
    }

    FILE *fp = stdout;
    if (*outfile && !(fp = fopen(outfile, "wb"))) {
        fprintf(stderr, "file open error: %s\n", outfile);
        return 0;
    }

    if (parse_bnc_vtec(infile, &rtcm) != 0) {
        fprintf(stderr, "Error parsing VTEC file: %s\n", infile);
        goto end;
    }

    if (!gen_rtcm3(&rtcm, 4076, 201, 0))
        goto end;

    if (fwrite(rtcm.buff, rtcm.nbyte, 1, fp) < 1)
        goto end;

    ret = 0;
end:

    fclose(fp);

    if (trlevel > 0) {
        traceclose();
    }
    return ret;
}
