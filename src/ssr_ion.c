/*------------------------------------------------------------------------------
* ionex.c : ionospheric SSR corrections
*
* references:
*     [1] IGS, "IGS State Space Representation (SSR)", IGS SSR v1.00 2020-10-05,
*         Format Version 1, October 5th 2000
*         Available at: https://files.igs.org/pub/data/format/igs_ssr_v1.pdf"
*-----------------------------------------------------------------------------*/

#include "rtklib.h"

#define MIN(x,y)    ((x)<(y)?(x):(y))

/* Compute the index for spherical harmonics coefficients */
extern int compute_index_ssr_iono_coeff_arrays(const int n, const int m, const int n_degree) {
    return n - m + m * (n_degree + 1) - (m * (m - 1)) / 2;
}

/** \brief Compute the fully normalized Legendre functions (\$\tilde P_n^m(x)\$)

Compute the Legendre polynomial, based on Section 6.7 of [1] (page 294). This
is equivalent to the function sph_harm_y in the scipy library (\$Y_n^m(\theta, \phi)\$), but
accepting a normalized input to the range [-1, 1].

References:
[1] Press, William H., Saul A. Teukolsky, William T. Vetterling, and Brian P. Flannery.
"Numerical recipes 3rd edition." Cambridge: New York (2007).
[2] Holmes, S. A. and W. E. Featherstone, 2002. "A unified approach to the Clenshaw
summation and the recursive computation of very high degree and order normalised
associated Legendre functions". Journal of Geodesy, 76(5), pp. 279-299.
[3] https://mitgcm.org/~mlosch/geoidcookbook/node11.html
[4] Heiskanen and Moritz, Physical Geodesy, 1967, eq. 1-62

* args   : int n          I   degree
*          int m          I   order
*          double u       I   input value, between -1 and 1
*          double p       O   \tilde P_n^m(u)
* return : >= 0 if successful
*/
extern int compute_legendre_polynomial(const int n, const int m, const double u, double* p) {

    int ret = -1;
	double fact, pll;
    double pmm = 1.0; // P_m^m

	if (m < 0 || m > n || fabs(u) > 1.0 || p == NULL) {
        goto end;
    }

	if (m > 0) {
		double omx2 = (1.0 - u) * (1.0 + u);
		fact = 1.0;
		for (int i = 1; i <= m; i++) {
			pmm *= omx2 * fact / (fact + 1.0);
			fact += 2.0;
		}
	}

	pmm = sqrt((2 * m + 1) * pmm / (4.0 * PI));

	if (m & 1) {  // If m is odd, negate pmm, mimics (-1)^m behavior
		pmm = -pmm;
    }

	if (n == m) {
        *p = pmm; // P_m^m(u) = pmm

    } else {
		double pmmp1 = u * sqrt(2.0 * m + 3.0) * pmm;
		if (n == (m+1)) {
            *p = pmmp1; // P_(m+1)^m(u) = pmmp1

        } else {
			double oldfact = sqrt(2.0 * m + 3.0);

			for (int i = m + 2; i <= n; i++) {
				fact = sqrt((4.0 * i * i - 1.0)/(i * i - m * m));
				pll = (u * pmmp1 - pmm / oldfact) * fact;
				oldfact = fact;
				pmm = pmmp1;
				pmmp1 = pll;
			}
			*p = pll;
		}
	}

    ret = 0; // Success
end:
    return ret;
}

/** \brief Compute VTEC using spherical harmonics coefficients */
static int compute_vtec_for_layer(
    const ionlayer_sphharm_t* layer,
    const int epoch_s,
    const double* pos,
    double* vtec) {

    // Offset to approximate TEC maximum at 14:00 local time
    // Note: the ICD for the IGS SSR (end of page 16) specifies 50400 s (14h)
    // instead of 7200 s (2h). However the computation of the VTEC indicates
    // that the IGS SSR products are computed based on these 2h (not 14h).
    // This will likely require revision in the future
    static const double EPOCH_OFFSET_S = 7200.0;  // 50400.0;
    double t, C_nm, S_nm; // Coefficients for spherical harmonics
    double pnm; // Legendre function value
    double lambda_s;  // Longitude at sun fixed reference frame

    double ret = -1;

    if (pos == NULL || vtec == NULL) {
        goto end;
    }

    *vtec = 0.0; // Initialize VTEC for this layer

    /* mean sun fixed and phase shifted longitude of ionospheric pierce point
      The mean sun fixed longitude phase shifted by 2 h to the approximate TEC
      maximum at 14:00 local time (resp. 50400 s) */
    t = (epoch_s + 86400) % 86400;
    lambda_s = pos[1] + (t - EPOCH_OFFSET_S) / 43200.0 * PI;
    lambda_s = fmod(lambda_s + 2 * PI, 2 * PI); // Normalize to [0, 2*PI)

    for (int n = 0; n <= layer->n_degree; n++) {
        for (int m = 0; m <= MIN(layer->m_order, n); m++) {

            int idx_cos = compute_index_ssr_iono_coeff_arrays(n, m, layer->n_degree);
            C_nm = layer->cos_coeff[idx_cos];

            S_nm = 0.0;

            if (n > 0 && m > 0) {
                int idx_sin = compute_index_ssr_iono_coeff_arrays(n - 1, m - 1, layer->n_degree -1);
                S_nm = layer->sin_coeff[idx_sin];
            }

            ret = compute_legendre_polynomial(n, m, sin(pos[0]), &pnm);
            if (ret < 0) {
                goto end;
            }

            *vtec += (C_nm * cos(m * lambda_s) + S_nm * sin(m * lambda_s)) * pnm;
        }
    }

    ret = 0;
end:
    return ret;

}

/** \brief Compute STEC from spherical harmonics
 *
 * compute ionospheric pierce point (ipp) position and slant factor
* args   : ssr_ion_t* ssr_ion  I   SSR ionospheric correction data
*          int epoch_s      I   Seconds of the day [0-86399)
*          double *pos      I   receiver position {lat,lon,h} (rad,m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
*          double re_km     I   earth radius (km)
*          double* stec     O   Slant Total Electron Content
* return : 0 or positive if successful, negative otherwise
* notes  : The pierce point can be computed with methods such as ionppp()
*-----------------------------------------------------------------------------*/
extern int compute_stec_from_spherical_harmonics(
    const ssr_ion_t* ssr_ion,
    const int epoch_s,
    const double* pos,
    const double* azel,
    double re_km,
    double* stec) {

    double vtec;
    double pospp[3];
    double psi_pp;

    int ret = -1;

    *stec = 0.0;

    /* aggregate all STEC values */
    for (int l = 0; l < ssr_ion->n_layers; l ++) {
        const ionlayer_sphharm_t* layer = &ssr_ion->ionlayers_sph_harm[l];

        ionppp(pos, azel, re_km, layer->height_km, pospp);

        ret = compute_vtec_for_layer(layer, epoch_s, pospp, &vtec);
        if (ret < 0) {
            goto end;
        }
        psi_pp = PI / 2.0 - azel[1] - asin((re_km + pospp[2]/1000.0) / (re_km + layer->height_km) * cos(azel[1]));

        *stec += vtec / sin(azel[1] + psi_pp);

        // *stec += vtec[l] / cos(asin((re_km + pospp[2]/1000.0) / (re_km + layer->height_km) * cos(azel[1])));

    }

    ret = 0;
end:
    return ret;
}
