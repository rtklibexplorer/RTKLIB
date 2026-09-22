/* RTK server cleanup must release owned precise navigation products.
 * Run with AddressSanitizer/LeakSanitizer to detect missing frees.
 * LSAN_OPTIONS=use_stacks=0:use_registers=0 prevents stale allocation
 * pointers from hiding leaks at exit; no allocations should survive cleanup.
 */
#include "../../src/rtklib.h"

/* Model the ownership transferred to nav by decodefile(). Test each
 * product independently as either array can remain NULL in practice.
 */
static int init_products(nav_t* nav, int products)
{
    if (products & 1) {
        nav->peph = (peph_t*)calloc(2, sizeof(*nav->peph));
        if (!nav->peph) {
            return 0;
        }
        nav->ne = nav->nemax = 2;
    }
    if (products & 2) {
        nav->pclk = (pclk_t*)calloc(3, sizeof(*nav->pclk));
        if (!nav->pclk) {
            return 0;
        }
        nav->nc = nav->ncmax = 3;
    }
    return 1;
}

static int run_case(int products)
{
    rtksvr_t* svr;
    int result = 1;

    svr = (rtksvr_t*)calloc(1, sizeof(*svr));
    if (!svr) {
        return 1;
    }
    if (!rtksvrinit(svr)) {
        free(svr);
        return 1;
    }
    result = init_products(&svr->nav, products) ? 0 : 1;
    rtksvrfree(svr);

    /* No worker was started. Dispose of the mutexes initialized by init;
     * freeing the outer object also makes leaked nav arrays unreachable.
     */
#ifdef WIN32
    DeleteCriticalSection(&svr->lock);
    for (int i = 0; i < MAXSTRRTK; i++) {
        DeleteCriticalSection(&svr->stream[i].lock);
    }
#else
    pthread_mutex_destroy(&svr->lock);
    for (int i = 0; i < MAXSTRRTK; i++) {
        pthread_mutex_destroy(&svr->stream[i].lock);
    }
#endif
    free(svr);
    return result;
}

int main(int argc, char** argv)
{
    if (argc != 2 || strlen(argv[1]) != 1 || argv[1][0] < '0' || argv[1][0] > '3') {
        return 1;
    }
    return run_case(argv[1][0] - '0');
}
