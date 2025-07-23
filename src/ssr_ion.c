/*------------------------------------------------------------------------------
* ionex.c : ionospheric SSR corrections
*
* references:
*     [1] IGS, "IGS State Space Representation (SSR)", IGS SSR v1.00 2020-10-05,
*         Format Version 1, October 5th 2000
*         Available at: https://files.igs.org/pub/data/format/igs_ssr_v1.pdf"
*-----------------------------------------------------------------------------*/

#include "rtklib.h"

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
