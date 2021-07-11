#define __USE_C99_MATH

#include <stdbool.h>
double F_cval(int, int, double);
double F_Distribution(double, int, int);
double Chi_Square_Distribution(double, int);
double X2_cval(int, double);
double T_cval(int, double);
double Student_t_Distribution(double, int);
void   RB_init(ring_buffer_t*, int64_t);
bool   RB_push(ring_buffer_t*, int8_t, double*);
double RB_pop(ring_buffer_t*);

