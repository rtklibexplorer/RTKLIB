/*------------------------------------------------------------------------------
* rtklib unit test driver : SSR ionospheric functions
*-----------------------------------------------------------------------------*/
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "../../src/rtklib.h"

/* Test compute_legendre_polynomial function */
void test_legendre_polynomial(void)
{
    double result;
    double tolerance = 1e-10;

    fprintf(stderr, "Testing compute_legendre_polynomial function...\n");

    /* Test basic Legendre polynomials P_n^m(x) */

    /* Various test cases for sectorial */
    struct legendre_test_cases {
        int n;
        int m;
        double u;
        double result;
    } TEST_CASES[] = {
        /** test cases built using the following Python code

        \verbatim
        from math import acos
        from scipy.special import sph_harm_y

        test_cases = [
            ( 1,  0, acos(0.5   )),
            ( 2,  1, acos(0.5   )),
            ( 3,  2, acos(0.1   )),
            ( 4,  1, acos(-0.3   )),
            ( 5,  3, acos(0.7   )),
            (10,  5, acos(0.9   )),
            (15, 15, acos(0.7071)),
            (20, 10, acos(0.1   )),
            (25,  5, acos(-0.9   ))
        ]

        for t in test_cases:
            result = sph_harm_y(*t, 0)
            print(f"{t} {result}")
        \endverbatim
        */
        {.n =  1, .m =  0, .u =  0.5   , .result =  0.24430125595145988},
        {.n =  2, .m =  1, .u =  0.5   , .result = -0.33452327177864455},
        {.n =  3, .m =  2, .u =  0.1   , .result =  0.10117656216689495},
        {.n =  4, .m =  1, .u = -0.3   , .result = -0.3208718590203563},
        {.n =  5, .m =  3, .u =  0.7   , .result = -0.429650274134896},
        {.n = 10, .m =  5, .u =  0.9   , .result = -0.2643099927773962},
        {.n = 15, .m = 15, .u =  0.7071, .result = -0.0032983285307971377},
        {.n = 20, .m = 10, .u =  0.1   , .result =  0.07522105324402778},
        {.n = 25, .m =  5, .u = -0.9   , .result =  0.3756544620315938},
    };
    size_t n_test_cases = sizeof(TEST_CASES) / sizeof(TEST_CASES[0]);

    for (int i = 0; i < n_test_cases; i ++) {
        const struct legendre_test_cases* tc = &TEST_CASES[i];
        assert (compute_legendre_polynomial(tc->n, tc->m, tc->u, &result) == 0);;
        assert (fabs(result - tc->result) < tolerance);
    }

    fprintf(stderr, "Testing compute_legendre_polynomial function (degraded cases)...\n");
    assert(compute_legendre_polynomial(-1, 0, 0.0, &result) < 0);
    assert(compute_legendre_polynomial(0, 1, 0.0, &result) < 0);
    assert(compute_legendre_polynomial(15, 15, 2.0, &result) < 0);
    assert(compute_legendre_polynomial(15, 15, 0.0, NULL) < 0);

}

/* Main test function */
int main(void)
{
    printf("=== Test Legendre Polynomial ===\n");

    test_legendre_polynomial();

    printf("\n=== All SSR ion tests completed ===\n");
    return 0;
}
