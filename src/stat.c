/*------------------------------------------------------------------------------
* stat.c: Statistical calculations
*-----------------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>                    // required for powl(), fabsl(), expl() and logl().
#include <float.h>                   // required for LDBL_EPSILON.
#include <limits.h>

static long double const pi 		  = 3.14159265358979323846264338L;
static double max_double_arg 		  = 171.0;
static long double max_long_double_arg 	  = 1755.5L;
static const long double ln_LDBL_MAX 	  = 1.13565234062941435e+4L;
static const long double log_sqrt_2pi 	  = 9.18938533204672741780329736e-1L;
static long double const exp_g_o_sqrt_2pi = +6.23316569877722552586386e+3L;

// Bernoulli numbers B(2),B(4),B(6),...,B(20).  Only B(2),...,B(6) currently //
// used.                                                                     //

static const long double B[] = {   1.0L / (long double)(6 * 2 * 1),
                                  -1.0L / (long double)(30 * 4 * 3),
                                   1.0L / (long double)(42 * 6 * 5),
                                  -1.0L / (long double)(30 * 8 * 7),
                                   5.0L / (long double)(66 * 10 * 9),
                                -691.0L / (long double)(2730 * 12 * 11),
                                   7.0L / (long double)(6 * 14 * 13),
                               -3617.0L / (long double)(510 * 16 * 15),
                               43867.0L / (long double)(796 * 18 * 17),
                             -174611.0L / (long double)(330 * 20 * 19) 
                           };
/* static const int n = sizeof(B) / sizeof(long double); */

static long double const e =  2.71828182845904523536028747L;
static long double const g =  9.65657815377331589457187L;
static long double const a[] = {
                                 +1.14400529453851095667309e+4L,
                                 -3.23988020152318335053598e+4L,
                                 +3.50514523505571666566083e+4L,
                                 -1.81641309541260702610647e+4L,
                                 +4.63232990536666818409138e+3L,
                                 -5.36976777703356780555748e+2L,
                                 +2.28754473395181007645155e+1L,
                                 -2.17925748738865115560082e-1L,
                                 +1.08314836272589368860689e-4L
                              };

//                         Internally Defined Routines                        //

static long double xGamma(long double x);

double Gamma_Function_Max_Arg( void ) 	    { return max_double_arg; }
long double xGamma_Function_Max_Arg( void ) { return max_long_double_arg; }
double abs_val(double y) 		    { if(y<0) return(y*-1); else return(y); }

static long double xLnGamma_Asymptotic_Expansion( long double x ) {
   const int  m = 3;
   long double term[3];
   long double sum = 0.0L;
   long double xx = x * x;
   long double xj = x;
   long double lngamma = log_sqrt_2pi - xj + (xj - 0.5L) * logl(xj);
   int i;

   for (i = 0; i < m; i++) { term[i] = B[i] / xj; xj *= xx; }
   for (i = m - 1; i >= 0; i--) sum += term[i]; 
   return lngamma + sum;
}

long double xGamma_Function(long double x)
{
   long double sin_x;
   long double rg;
   long int ix;
   if ( x > 0.0L ) {
      if (x <= max_long_double_arg) return xGamma(x);
      else return LDBL_MAX;
   }
   if ( x > -(long double)LONG_MAX) {
      ix = (long int) x;
      if ( x == (long double) ix) return LDBL_MAX;
   }
   sin_x = sinl(pi * x);
   if ( sin_x == 0.0L ) return LDBL_MAX;
   if ( x < -(max_long_double_arg - 1.0L) ) return 0.0L;
   rg = xGamma(1.0L - x) * sin_x / pi;
   if ( rg != 0.0L ) return (1.0L / rg);
   return LDBL_MAX;
}

static long double Duplication_Formula( long double two_x )
{
   long double x = 0.5L * two_x;
   long double g;
/*   double two_n = 1.0; */
   int n = (int) two_x - 1;

   g = powl(2.0L, two_x - 1.0L - (long double) n);
   g = ldexpl(g,n);
   g /= sqrt(pi);
   g *= xGamma_Function(x);
   g *= xGamma_Function(x + 0.5L);

   return g;
}

static long double xGamma(long double x)
{

   long double xx = (x < 1.0L) ? x + 1.0L : x;
   long double temp;
   int const n = sizeof(a) / sizeof(long double);
   int i;

   if (x > 1755.5L) return LDBL_MAX;

   if (x > 900.0L) return Duplication_Formula(x);

   temp = 0.0L;
   for (i = n-1; i >= 0; i--) {
      temp += ( a[i] / (xx + (long double) i) );
   }
   temp += 1.0L;
   temp *= ( powl((g + xx - 0.5L) / e, xx - 0.5L) / exp_g_o_sqrt_2pi );
   return (x < 1.0L) ?  temp / x : temp;
}

long double xLn_Gamma_Function(long double x)
{
   if (x <= Gamma_Function_Max_Arg()) return logl(xGamma_Function(x));
   return xLnGamma_Asymptotic_Expansion( x );
}

long double xLn_Beta_Function(long double a, long double b)
{
   if ( (a + b) <= (long double) Gamma_Function_Max_Arg() ) {
      if ( a == 1.0L && b == 1.0L ) return 0.0L;
      else return logl( xGamma_Function(a) /
                             ( xGamma_Function(a + b) / xGamma_Function(b) ));
   }
   return xLn_Gamma_Function(a) + xLn_Gamma_Function(b)
                                                  - xLn_Gamma_Function(a+b);
}

double Ln_Beta_Function(double a, double b)
{
   return (double) xLn_Beta_Function( (long double) a, (long double) b );
}

long double xBeta_Function(long double a, long double b)
{
   long double lnbeta;
   if ( (a + b) <= Gamma_Function_Max_Arg() )
      return xGamma_Function(a) / (xGamma_Function(a + b) / xGamma_Function(b));
   lnbeta = xLn_Gamma_Function(a) + xLn_Gamma_Function(b)
                                                 - xLn_Gamma_Function(a + b);
   return (lnbeta > ln_LDBL_MAX) ? (long double) LDBL_MAX : expl(lnbeta);
}

static long double Beta_Continued_Fraction( long double x, long double a,
                                                                 long double b)
{
   long double Am1 = 1.0L;
   long double A0 = 0.0L;
   long double Bm1 = 0.0L;
   long double B0 = 1.0L;
   long double e = 1.0L;
   long double Ap1 = A0 + e * Am1;
   long double Bp1 = B0 + e * Bm1;
   long double f_less = Ap1 / Bp1;
   long double f_greater = 0.0L;
   long double aj = a;
   long double am = a;
   static long double eps = 10.0L * LDBL_EPSILON;
   int j = 0;
   int m = 0;
   int k = 1;

   if ( x == 0.0L ) return 0.0L;
   
   while ( (2.0L * fabsl(f_greater - f_less) > eps * fabsl(f_greater + f_less)) ) {
      Am1 = A0;
      A0 = Ap1;
      Bm1 = B0;
      B0 = Bp1;
      am = a + m;
      e = - am * (am + b) * x / ( (aj + 1.0L) * aj );
      Ap1 = A0 + e * Am1;
      Bp1 = B0 + e * Bm1;
      k = (k + 1) & 3;
      if (k == 1) f_less = Ap1/Bp1;
      else if (k == 3) f_greater = Ap1/Bp1;
      if ( fabsl(Bp1) > 1.0L) {
         Am1 = A0 / Bp1;
         A0 = Ap1 / Bp1;
         Bm1 = B0 / Bp1;
         B0 = 1.0;
      } else {
         Am1 = A0;
         A0 = Ap1;
         Bm1 = B0;
         B0 = Bp1;
      }
      m++;
      j += 2;
      aj = a + j;
      e = m * ( b - m ) * x / ( ( aj - 1.0L) * aj  );
      Ap1 = A0 + e * Am1;
      Bp1 = B0 + e * Bm1;
      k = (k + 1) & 3;
      if (k == 1) f_less = Ap1/Bp1;
      else if (k == 3) f_greater = Ap1/Bp1;
   }
   return expl( a * logl(x) + b * logl(1.0L - x) + logl(Ap1 / Bp1) ) /
                                                ( a * xBeta_Function(a,b) );
}

static long double xBeta_Distribution(double xx, double aa, double bb) 
{
   long double x = (long double) xx;
   long double a = (long double) aa;
   long double b = (long double) bb;
   if ( aa > 1.0 && bb > 1.0 ) {
      if ( x <= (a - 1.0L) / ( a + b - 2.0L ) )
         return Beta_Continued_Fraction(x, a, b);
      else
         return 1.0L - Beta_Continued_Fraction( 1.0L - x, b, a );
   }
   if ( aa < 1.0 && bb < 1.0 )  
      return (a * xBeta_Distribution(xx, aa + 1.0, bb) 
                      + b * xBeta_Distribution(xx, aa, bb + 1.0) ) / (a + b); 
   if ( aa == 1.0 )
      return 1.0L - powl(1.0L - x, b) / ( b * xBeta_Function(a,b) );

   if ( bb == 1.0 ) return powl(x, a) / ( a * xBeta_Function(a,b) );
   if ( aa < 1.0 )  
      return xBeta_Distribution(xx, aa + 1.0, bb)
            + powl(x, a) * powl(1.0L - x, b) / ( a * xBeta_Function(a,b) );
   return xBeta_Distribution(xx, aa, bb + 1.0)
            - powl(x, a) * powl(1.0L - x, b) / ( b * xBeta_Function(a,b) );
}

double Beta_Distribution(double x, double a, double b)
{
   if ( x <= 0.0 ) return 0.0;
   if ( x >= 1.0 ) return 1.0;
   return (double) xBeta_Distribution( x, a, b);
}

double F_Density( double x, int v1, int v2 )
{
   double ln_density;
   double v12 = (double)v1 / 2.0;
   double v22 = (double)v2 / 2.0;
   if ( x <= 0.0 ) return 0.0;
   ln_density = v12*log((double)v1) + v22 * log((double)v2)
                + (v12 - 1.0) * log(x) - (v12 + v22) * log((double)v2+v1*x)
                - Ln_Beta_Function(v12, v22);
   return exp(ln_density);
}

double F_Distribution(double f, int v1, int v2)
{
   double a = (double) v1 / 2.0;
   double b = (double) v2 / 2.0;
   double g = a*f;
   if ( f <= 0.0 ) return 0.0;
   return Beta_Distribution( g / (b + g), a, b);
}

double F_cval(int v1, int v2, double pr) {
   double f0 = 0.0, f = 1.0, diff, delta = 0.1, p;
   int n = 0;

   while ( (p = F_Distribution(f,v1,v2)) < pr) {
      f += delta;
      if (p > 0.999) {
         delta /= 10.0;
         f = (f - delta) / 10;
      }
   }
  
   f0 = f - delta;
   while ( (p = F_Distribution(f0,v1,v2)) > pr) {
      f0 -= delta;
      if (p < 0.001) {
         delta /= 10.0;
         f0 = (f0 + delta) / 10;
      }   
   }

   while (fabs(f - f0) > 1.e-6) {
      diff = F_Distribution(f, v1, v2) - pr;
      diff /= F_Density(f,v1,v2);
      diff /= 2.0;
      f0 = f;
      f -= diff;
      n++;
      if (n > 40) exit(0);
   }
   return f;
}

/* Needed for X2-Test, Values ... */


static long double const factorials[] = {
       1.000000000000000000000e+0L,          //   0!
       1.000000000000000000000e+0L,          //   1!
       2.000000000000000000000e+0L,          //   2!
       6.000000000000000000000e+0L,          //   3!
       2.400000000000000000000e+1L,          //   4!
       1.200000000000000000000e+2L,          //   5!
       7.200000000000000000000e+2L,          //   6!
       5.040000000000000000000e+3L,          //   7!
       4.032000000000000000000e+4L,          //   8!
       3.628800000000000000000e+5L,          //   9!
       3.628800000000000000000e+6L,          //  10!
       3.991680000000000000000e+7L,          //  11!
       4.790016000000000000000e+8L,          //  12!
       6.227020800000000000000e+9L,          //  13!
       8.717829120000000000000e+10L,         //  14!
       1.307674368000000000000e+12L,         //  15!
       2.092278988800000000000e+13L,         //  16!
       3.556874280960000000000e+14L,         //  17!
       6.402373705728000000000e+15L,         //  18!
       1.216451004088320000000e+17L,         //  19!
       2.432902008176640000000e+18L,         //  20!
       5.109094217170944000000e+19L,         //  21!
       1.124000727777607680000e+21L,         //  22!
       2.585201673888497664000e+22L,         //  23!
       6.204484017332394393600e+23L,         //  24!
       1.551121004333098598400e+25L,         //  25!
       4.032914611266056355840e+26L,         //  26!
       1.088886945041835216077e+28L,         //  27!
       3.048883446117138605015e+29L,         //  28!
       8.841761993739701954544e+30L,         //  29!
       2.652528598121910586363e+32L,         //  30!
       8.222838654177922817726e+33L,         //  31!
       2.631308369336935301672e+35L,         //  32!
       8.683317618811886495518e+36L,         //  33!
       2.952327990396041408476e+38L,         //  34!
       1.033314796638614492967e+40L,         //  35!
       3.719933267899012174680e+41L,         //  36!
       1.376375309122634504632e+43L,         //  37!
       5.230226174666011117600e+44L,         //  38!
       2.039788208119744335864e+46L,         //  39!
       8.159152832478977343456e+47L,         //  40!
       3.345252661316380710817e+49L,         //  41!
       1.405006117752879898543e+51L,         //  42!
       6.041526306337383563736e+52L,         //  43!
       2.658271574788448768044e+54L,         //  44!
       1.196222208654801945620e+56L,         //  45!
       5.502622159812088949850e+57L,         //  46!
       2.586232415111681806430e+59L,         //  47!
       1.241391559253607267086e+61L,         //  48!
       6.082818640342675608723e+62L,         //  49!
       3.041409320171337804361e+64L,         //  50!
       1.551118753287382280224e+66L,         //  51!
       8.065817517094387857166e+67L,         //  52!
       4.274883284060025564298e+69L,         //  53!
       2.308436973392413804721e+71L,         //  54!
       1.269640335365827592597e+73L,         //  55!
       7.109985878048634518540e+74L,         //  56!
       4.052691950487721675568e+76L,         //  57!
       2.350561331282878571829e+78L,         //  58!
       1.386831185456898357379e+80L,         //  59!
       8.320987112741390144276e+81L,         //  60!
       5.075802138772247988009e+83L,         //  61!
       3.146997326038793752565e+85L,         //  62!
       1.982608315404440064116e+87L,         //  63!
       1.268869321858841641034e+89L,         //  64!
       8.247650592082470666723e+90L,         //  65!
       5.443449390774430640037e+92L,         //  66!
       3.647111091818868528825e+94L,         //  67!
       2.480035542436830599601e+96L,         //  68!
       1.711224524281413113725e+98L,         //  69!
       1.197857166996989179607e+100L,        //  70!
       8.504785885678623175212e+101L,        //  71!
       6.123445837688608686152e+103L,        //  72!
       4.470115461512684340891e+105L,        //  73!
       3.307885441519386412260e+107L,        //  74!
       2.480914081139539809195e+109L,        //  75!
       1.885494701666050254988e+111L,        //  76!
       1.451830920282858696341e+113L,        //  77!
       1.132428117820629783146e+115L,        //  78!
       8.946182130782975286851e+116L,        //  79!
       7.156945704626380229481e+118L,        //  80!
       5.797126020747367985880e+120L,        //  81!
       4.753643337012841748421e+122L,        //  82!
       3.945523969720658651190e+124L,        //  83!
       3.314240134565353266999e+126L,        //  84!
       2.817104114380550276949e+128L,        //  85!
       2.422709538367273238177e+130L,        //  86!
       2.107757298379527717214e+132L,        //  87!
       1.854826422573984391148e+134L,        //  88!
       1.650795516090846108122e+136L,        //  89!
       1.485715964481761497310e+138L,        //  90!
       1.352001527678402962552e+140L,        //  91!
       1.243841405464130725548e+142L,        //  92!
       1.156772507081641574759e+144L,        //  93!
       1.087366156656743080274e+146L,        //  94!
       1.032997848823905926260e+148L,        //  95!
       9.916779348709496892096e+149L,        //  96!
       9.619275968248211985333e+151L,        //  97!
       9.426890448883247745626e+153L,        //  98!
       9.332621544394415268170e+155L,        //  99!
       9.332621544394415268170e+157L,        // 100!
       9.425947759838359420852e+159L,        // 101!
       9.614466715035126609269e+161L,        // 102!
       9.902900716486180407547e+163L,        // 103!
       1.029901674514562762385e+166L,        // 104!
       1.081396758240290900504e+168L,        // 105!
       1.146280563734708354534e+170L,        // 106!
       1.226520203196137939352e+172L,        // 107!
       1.324641819451828974500e+174L,        // 108!
       1.443859583202493582205e+176L,        // 109!
       1.588245541522742940425e+178L,        // 110!
       1.762952551090244663872e+180L,        // 111!
       1.974506857221074023537e+182L,        // 112!
       2.231192748659813646597e+184L,        // 113!
       2.543559733472187557120e+186L,        // 114!
       2.925093693493015690688e+188L,        // 115!
       3.393108684451898201198e+190L,        // 116!
       3.969937160808720895402e+192L,        // 117!
       4.684525849754290656574e+194L,        // 118!
       5.574585761207605881323e+196L,        // 119!
       6.689502913449127057588e+198L,        // 120!
       8.094298525273443739682e+200L,        // 121!
       9.875044200833601362412e+202L,        // 122!
       1.214630436702532967577e+205L,        // 123!
       1.506141741511140879795e+207L,        // 124!
       1.882677176888926099744e+209L,        // 125!
       2.372173242880046885677e+211L,        // 126!
       3.012660018457659544810e+213L,        // 127!
       3.856204823625804217357e+215L,        // 128!
       4.974504222477287440390e+217L,        // 129!
       6.466855489220473672507e+219L,        // 130!
       8.471580690878820510985e+221L,        // 131!
       1.118248651196004307450e+224L,        // 132!
       1.487270706090685728908e+226L,        // 133!
       1.992942746161518876737e+228L,        // 134!
       2.690472707318050483595e+230L,        // 135!
       3.659042881952548657690e+232L,        // 136!
       5.012888748274991661035e+234L,        // 137!
       6.917786472619488492228e+236L,        // 138!
       9.615723196941089004197e+238L,        // 139!
       1.346201247571752460588e+241L,        // 140!
       1.898143759076170969429e+243L,        // 141!
       2.695364137888162776589e+245L,        // 142!
       3.854370717180072770522e+247L,        // 143!
       5.550293832739304789551e+249L,        // 144!
       8.047926057471991944849e+251L,        // 145!
       1.174997204390910823948e+254L,        // 146!
       1.727245890454638911203e+256L,        // 147!
       2.556323917872865588581e+258L,        // 148!
       3.808922637630569726986e+260L,        // 149!
       5.713383956445854590479e+262L,        // 150!
       8.627209774233240431623e+264L,        // 151!
       1.311335885683452545607e+267L,        // 152!
       2.006343905095682394778e+269L,        // 153!
       3.089769613847350887959e+271L,        // 154!
       4.789142901463393876336e+273L,        // 155!
       7.471062926282894447084e+275L,        // 156!
       1.172956879426414428192e+278L,        // 157!
       1.853271869493734796544e+280L,        // 158!
       2.946702272495038326504e+282L,        // 159!
       4.714723635992061322407e+284L,        // 160!
       7.590705053947218729075e+286L,        // 161!
       1.229694218739449434110e+289L,        // 162!
       2.004401576545302577600e+291L,        // 163!
       3.287218585534296227263e+293L,        // 164!
       5.423910666131588774984e+295L,        // 165!
       9.003691705778437366474e+297L,        // 166!
       1.503616514864999040201e+300L,        // 167!
       2.526075744973198387538e+302L,        // 168!
       4.269068009004705274939e+304L,        // 169!
       7.257415615307998967397e+306L         // 170!
                                        };

static const int N = sizeof(factorials) / sizeof(long double);



long double xFactorial(int n) {
   if ( n < 0 ) return 0.0L;
   if ( n >= N ) return  (long double) DBL_MAX;
   return factorials[n];
}

static long double xMedium_x(long double x, long double nu)
{
   long double coef;
   long double term = 1.0L / nu;
   long double corrected_term = term;
   long double temp_sum = term;
   long double correction = -temp_sum + corrected_term;
   long double sum1 = temp_sum;
   long double sum2;
   long double epsilon = 0.0L;
   int i;

   if (nu > Gamma_Function_Max_Arg()) {
      coef = expl( nu * logl(x) - x - xLn_Gamma_Function(nu) );
      if (coef > 0.0L) epsilon = DBL_EPSILON/coef;
   } else {
      coef = powl(x, nu) * expl(-x) / xGamma_Function(nu);
      epsilon = DBL_EPSILON/coef;
   }
   if (epsilon <= 0.0L) epsilon = (long double) DBL_EPSILON;

   for (i = 1; term > epsilon * sum1; i++) {
      term *= x / (nu + i);
      corrected_term = term + correction;
      temp_sum = sum1 + corrected_term;
      correction = (sum1 - temp_sum) + corrected_term;
      sum1 = temp_sum;
   }
   sum2 = sum1;
   sum1 *= coef;
   correction += sum2 - sum1 / coef;
   term *= x / (nu + i);
   sum2 = term + correction;
   for (i++; (term + correction) > epsilon * sum2; i++) {
      term *= x / (nu + i);
      corrected_term = term + correction;
      temp_sum = sum2 + corrected_term;
      correction = (sum2 - temp_sum) + corrected_term;
      sum2 = temp_sum;
   }

   sum2 += correction;
   sum2 *= coef;
   return sum1 + sum2;
}

static long double xLarge_x(long double x, long double nu)
{
   long double temp = 1.0L / nu;
   long double sum = temp;
   long double coef;
   int i = 0;
   int n;

   n = (int)(x - nu - 1.0L) + 1;
   for (i = 1; i < n; i++) {
      temp *= x / (nu + i);
      sum += temp;
   }
   if ( nu <= Gamma_Function_Max_Arg() ) {
      coef = powl(x, nu) * expl(-x) / xGamma_Function(nu);
      return xMedium_x(x, nu + n) + coef * sum;
   } else {
      return expl(logl(sum) + nu * logl(x) - x - xLn_Gamma_Function(nu)) +
                                                        xMedium_x(x, nu + n); 
   }   
}

#define Nterms 20
static long double xSmall_x(long double x, long double nu)
{
   long double terms[Nterms];
   long double x_term = -x;
   long double x_power = 1.0L;
   long double sum;
   int i;

   for (i = 0; i < Nterms; i++) {
      terms[i] = (x_power / xFactorial(i)) / (i + nu);
      x_power *= x_term;
   }
   sum = terms[Nterms-1];
   for (i = Nterms-2; i >= 0; i--) sum += terms[i];
   if ( nu <= Gamma_Function_Max_Arg() )
      return powl(x,nu) * sum / xGamma_Function(nu);
   else return expl(nu * logl(x) + logl(sum) - xLn_Gamma_Function(nu));
}

long double xEntire_Incomplete_Gamma_Function(long double x, long double nu)
{

   if (x == 0.0L) return 0.0L;
   if (fabsl(x) <= 1.0L) return xSmall_x(x, nu);
   if (fabsl(x) < (nu + 1.0L) ) return xMedium_x(x, nu);
   return xLarge_x(x, nu);
}

double Entire_Incomplete_Gamma_Function(double x, double nu)
{
   return (double) xEntire_Incomplete_Gamma_Function((long double)x,
                                                              (long double)nu);
}

double Gamma_Distribution(double x, double nu) {
   return  ( x <= 0.0 ) ? 0.0 : Entire_Incomplete_Gamma_Function(x,nu);
}
double Gamma_Function(double x)
{
   long double g;

   if ( x > max_double_arg ) return DBL_MAX;
   g = xGamma_Function( (long double) x);
   if (fabsl(g) < DBL_MAX) return (double) g;
   return (g < 0.0L) ? -DBL_MAX : DBL_MAX;

}

double Ln_Gamma_Function(double x)
{
   if (x <= Gamma_Function_Max_Arg()) return log(Gamma_Function(x));
   return (double) xLnGamma_Asymptotic_Expansion( (long double) x );
}


double Chi_Square_Density( double x, int n )
{
   double n2 = 0.5 * (double) n;
   double ln_density;

   if ( x < 0.0 ) return 0.0;
   if ( x == 0.0 ) {
      if ( n == 1 ) return DBL_MAX;
      if ( n == 2 ) return 0.5;
      return 0.0;
   }
   ln_density = (n2 - 1.0) * log(0.5 * x) - 0.5 * x - Ln_Gamma_Function(n2);
   return 0.5 * exp(ln_density);
}


double Chi_Square_Distribution(double x, int n)
{
   if ( x <= 0.0 ) return 0.0;
   return Gamma_Distribution( 0.5 * x, 0.5 * (double) n);
}

double X2_cval(int nu, double pr) {
   double t0 = 0.0;
   double t = 2.0;
   double diff;
   double delta = 0.1;
   double p;
   int n = 0;

   t = 0.0;
   while ( (p = Chi_Square_Distribution(t,nu)) < pr) {
      t += delta;
      if (p > 0.999999) {
         delta /= 10.0;
         t = (t - delta) / 10;
      }
   }   
   t0 = t - delta;
   if (t0 < 0.0) t0 = 0.0;
   while ( (p = Chi_Square_Distribution(t0,nu)) > pr) {
      t0 -= delta;
      if (p < 0.001) {
         delta /= 10.0;
         t0 = (t0 + delta) / 10;
      }   
   }
   while (fabs(t - t0) > 1.e-6) {
      if (t < 0.0) t = 0.0;
      diff = Chi_Square_Distribution(t, nu) - pr;
      diff /= Chi_Square_Density(t,nu);
      diff /= 2.0;
      t0 = t;
      t -= diff;
      n++;
      if (n > 40) exit(0);
   }
   return t;
}

double Student_t_Distribution(double x, int n)
{
   double a = (double) n / 2.0;
   double beta = Beta_Distribution( 1.0 / (1.0 + x * x / n), a, 0.5);

   if ( x > 0.0 ) return 1.0 - 0.5 * beta;
   else if ( x < 0.0) return 0.5 * beta;
   return 0.5;
}

double Student_t_Density( double x, int n )
{
   double ln_density;

   ln_density = -(double)(n+1)/2.0 * log(1.0 + x * x /(double)n)
                - 0.5*log((double)n)
                - Ln_Beta_Function(0.5 * (double)n, 0.5);

   return exp(ln_density);
}

double T_cval(int nu, double pr) {
   double t0 = 0.0;
   double t = 2.0;
   double diff;
   double delta = 0.1;
   double p;
   int n = 0;

   t = 0.0;
   while ( (p = Student_t_Distribution(t,nu)) < pr) {
      t += delta;
      if (p > 0.99999999) {
         delta /= 10.0;
         t = (t - delta) / 10;
      }
   }   
   t0 = t - delta;
   while ( (p = Student_t_Distribution(t0,nu)) > pr) {
      t0 -= delta;
      if (p < 0.001) {
         delta /= 10.0;
         t0 = (t0 + delta) / 10;
      }   
   }
   while (fabs(t - t0) > 1.e-6) {
      diff = Student_t_Distribution(t, nu) - pr;
      diff /= Student_t_Density(t,nu);
      diff /= 2.0;
      t0 = t;
      t -= diff;
      n++;
      if (n > 40) exit(0);
   }
   return t;
}

/* NOTE main() 
https://blog.minitab.com/blog/adventures-in-statistics-2/the-american-statistical-associations-statement-on-the-use-of-p-values
T() Die folgende Tabelle zeigt ausgewählte Werte der inversen Verteilungsfunktion der T-Verteilung: T(1-a|df). 

*/
void amain(int argc, char *argv[]) {
   int ndof, ddof;
   double p,F1,t;
   p=0.95; ndof=2; ddof=5; F1=F_cval(ndof,ddof,p);
   printf("p=%f n1=%d n2=%d F=%.3f\n",p,ndof,ddof,F1);
   printf("p=%f n1=%d n2=%d F=%.3f\n",p,ndof,ddof,F_Distribution(F1,ndof,ddof));   
   ndof = 10; t = abs_val(T_cval(ndof,p)); // 1.812
   printf("p=%.2f n=%d t=%.3f\n",p,ndof,t);		  // Einseitig
   printf("t=%.3f df=%d Verify_P=%.3f\n",t,ndof,Student_t_Distribution(t,ndof));

}
