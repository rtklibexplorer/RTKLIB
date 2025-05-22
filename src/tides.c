/*------------------------------------------------------------------------------
* tides.c : tidal displacement corrections
*
*          Copyright (C) 2015-2017 by T.TAKASU, All rights reserved.
*
* references :
*     [1] D.D.McCarthy, IERS Technical Note 21, IERS Conventions 1996, July 1996
*     [2] D.D.McCarthy and G.Petit, IERS Technical Note 32, IERS Conventions
*         2003, November 2003
*     [3] D.A.Vallado, Fundamentals of Astrodynamics and Applications 2nd ed,
*         Space Technology Library, 2004
*     [4] J.Kouba, A Guide to using International GNSS Service (IGS) products,
*         May 2009
*     [5] G.Petit and B.Luzum (eds), IERS Technical Note No. 36, IERS
*         Conventions (2010), 2010
*
* version : $Revision:$ $Date:$
* history : 2015/05/10 1.0  separated from ppp.c
*           2015/06/11 1.1  fix bug on computing days in tide_oload() (#128)
*           2017/04/11 1.2  fix bug on calling geterp() in timdedisp()
*-----------------------------------------------------------------------------*/
#include "rtklib.h"

#define SQR(x)      ((x)*(x))

// The following few functions are in support of dehanttideinel. This code is
// a translation of the respective iers fortran code to C and to RTKLIB, and
// the reference code is included in the lib/iers/src/ directory.

// This subroutine gives the in-phase and out-of-phase corrections induced by
// mantle anelasticity in the long period band.
static void step2lon_(const double xsta[3], double t, double xcorsta[3]) {
  static const double datdi[5][9] = {{0., 0., 0., 1., 0., .47, .23, .16, .07},
                                     {0., 2., 0., 0., 0., -.2, -.12, -.11, -.05},
                                     {1., 0., -1., 0., 0., -.11, -.08, -.09, -.04},
                                     {2., 0., 0., 0., 0., -.13, -.11, -.15, -.07},
                                     {2., 0., 0., 1., 0., -.05, -.05, -.06, -.03}};

  //  Compute the phase angles in degrees.
  double s = ((t * 1.85139e-6 - .0014663889) * t + 481267.88194) * t + 218.31664563;
  double pr = (((t * 7e-9 + 2.1e-8) * t + 3.08889e-4) * t + 1.396971278) * t;
  s += pr;
  double h = (((t * -6.54e-9 + 2e-8) * t + 3.0322222e-4) * t + 36000.7697489) * t + 280.46645;
  double p =
      (((t * 5.263e-8 - 1.24991e-5) * t - .01032172222) * t + 4069.01363525) * t + 83.35324312;
  double zns =
      (((t * 1.65e-8 - 2.13944e-6) * t - .00207561111) * t + 1934.13626197) * t + 234.95544499;
  double ps =
      (((t * -3.34e-9 - 1.778e-8) * t + 4.5688889e-4) * t + 1.71945766667) * t + 282.93734098;
  double rsta = sqrt(xsta[0] * xsta[0] + xsta[1] * xsta[1] + xsta[2] * xsta[2]);
  double sinphi = xsta[2] / rsta;
  double cosphi = sqrt(xsta[0] * xsta[0] + xsta[1] * xsta[1]) / rsta;
  double cosla = xsta[0] / cosphi / rsta;
  double sinla = xsta[1] / cosphi / rsta;
  // Reduce angles to between the range 0 and 360.
  s = fmod(s, 360);
  //  tau = fmod(tau, 360)
  h = fmod(h, 360);
  p = fmod(p, 360);
  zns = fmod(zns, 360);
  ps = fmod(ps, 360);
  double dr_tot = 0, dn_tot = 0.;
  for (int i = 0; i < 3; ++i) xcorsta[i] = 0;
  for (int j = 0; j < 5; ++j) {
    double thetaf = (datdi[j][0] * s + datdi[j][1] * h + datdi[j][2] * p + datdi[j][3] * zns +
                     datdi[j][4] * ps) *
                    D2R;
    double dr = datdi[j][5] * (sinphi * sinphi * 3 - 1) / 2 * cos(thetaf) +
                datdi[j][7] * (sinphi * sinphi * 3 - 1) / 2 * sin(thetaf);
    double dn = datdi[j][6] * (cosphi * sinphi * 2) * cos(thetaf) +
                datdi[j][8] * (cosphi * sinphi * 2) * sin(thetaf);
    double de = 0;
    dr_tot += dr;
    dn_tot += dn;
    xcorsta[0] += dr * cosla * cosphi - de * sinla - dn * sinphi * cosla;
    xcorsta[1] += dr * sinla * cosphi + de * cosla - dn * sinphi * sinla;
    xcorsta[2] += dr * sinphi + dn * cosphi;
  }
  for (int i = 0; i < 3; ++i) xcorsta[i] /= 1e3;
}

// This subroutine gives the in-phase and out-of-phase corrections induced by
// mantle anelasticity in the diurnal band.
static void step2diu_(const double xsta[3], double fhr, double t, double xcorsta[3]) {
  static const double datdi[31][9] = {{-3, 0, 2, 0, 0, -0.01, 0, 0, 0},
                                      {-3, 2, 0, 0, 0, -0.01, 0, 0, 0},
                                      {-2, 0, 1, -1, 0, -0.02, 0, 0, 0},
                                      {-2, 0, 1, 0, 0, -0.08, 0, -0.01, 0.01},
                                      {-2, 2, -1, 0, 0, -0.02, 0, 0, 0},
                                      {-1, 0, 0, -1, 0, -0.10, 0, 0, 0},
                                      {-1, 0, 0, 0, 0, -0.51, 0, -0.02, 0.03},
                                      {-1, 2, 0, 0, 0, 0.01, 0, 0, 0},
                                      {0, -2, 1, 0, 0, 0.01, 0, 0, 0},
                                      {0, 0, -1, 0, 0, 0.02, 0, 0, 0},
                                      {0, 0, 1, 0, 0, 0.06, 0, 0, 0},
                                      {0, 0, 1, 1, 0, 0.01, 0, 0, 0},
                                      {0, 2, -1, 0, 0, 0.01, 0, 0, 0},
                                      {1, -3, 0, 0, 1, -0.06, 0, 0, 0},
                                      {1, -2, 0, -1, 0, 0.01, 0, 0, 0},
                                      {1, -2, 0, 0, 0, -1.23, -0.07, 0.06, 0.01},
                                      {1, -1, 0, 0, -1, 0.02, 0, 0, 0},
                                      {1, -1, 0, 0, 1, 0.04, 0, 0, 0},
                                      {1, 0, 0, -1, 0, -0.22, 0.01, 0.01, 0},
                                      {1, 0, 0, 0, 0, 12.00, -0.80, -0.67, -0.03},
                                      {1, 0, 0, 1, 0, 1.73, -0.12, -0.10, 0},
                                      {1, 0, 0, 2, 0, -0.04, 0, 0, 0},
                                      {1, 1, 0, 0, -1, -0.50, -0.01, 0.03, 0},
                                      {1, 1, 0, 0, 1, 0.01, 0, 0, 0},
                                      {0, 1, 0, 1, -1, -0.01, 0, 0, 0},
                                      {1, 2, -2, 0, 0, -0.01, 0, 0, 0},
                                      {1, 2, 0, 0, 0, -0.11, 0.01, 0.01, 0},
                                      {2, -2, 1, 0, 0, -0.01, 0, 0, 0},
                                      {2, 0, -1, 0, 0, -0.02, 0, 0, 0},
                                      {3, 0, 0, 0, 0, 0, 0, 0, 0},
                                      {3, 0, 0, 1, 0, 0, 0, 0, 0}};
  //  Compute the phase angles in degrees.
  double s = ((t * 1.85139e-6 - .0014663889) * t + 481267.88194) * t + 218.31664563;
  double tau =
      fhr * 15. + 280.4606184 + ((t * -2.58e-8 + 3.8793e-4) * t + 36000.7700536) * t + (-s);
  double pr = (((t * 7e-9 + 2.1e-8) * t + 3.08889e-4) * t + 1.396971278) * t;
  s += pr;
  double h__ = (((t * -6.54e-9 + 2e-8) * t + 3.0322222e-4) * t + 36000.7697489) * t + 280.46645;
  double p =
      (((t * 5.263e-8 - 1.24991e-5) * t - 0.01032172222) * t + 4069.01363525) * t + 83.35324312;
  double zns =
      (((t * 1.65e-8 - 2.13944e-6) * t - 0.00207561111) * t + 1934.13626197) * t + 234.95544499;
  double ps =
      (((t * -3.34e-9 - 1.778e-8) * t + 4.5688889e-4) * t + 1.71945766667) * t + 282.93734098;
  // Reduce angles to between the range 0 and 360.
  s = fmod(s, 360);
  tau = fmod(tau, 360);
  h__ = fmod(h__, 360);
  p = fmod(p, 360);
  zns = fmod(zns, 360);
  ps = fmod(ps, 360);
  double rsta = sqrt(SQR(xsta[0]) + SQR(xsta[1]) + SQR(xsta[2]));
  double sinphi = xsta[2] / rsta;
  double cosphi = sqrt(SQR(xsta[0]) + SQR(xsta[1])) / rsta;
  double cosla = xsta[0] / cosphi / rsta;
  double sinla = xsta[1] / cosphi / rsta;
  double zla = atan2(xsta[1], xsta[0]);
  // Initialize.
  for (int i = 0; i < 3; ++i) xcorsta[i] = 0.;
  for (int j = 0; j < 31; ++j) {
    // Convert from degrees to radians.
    double thetaf = (tau + datdi[j][0] * s + datdi[j][1] * h__ + datdi[j][2] * p +
                     datdi[j][3] * zns + datdi[j][4] * ps) *
                    D2R;
    double dr = datdi[j][5] * 2 * sinphi * cosphi * sin(thetaf + zla) +
                datdi[j][6] * 2 * sinphi * cosphi * cos(thetaf + zla);
    double dn = datdi[j][7] * (SQR(cosphi) - SQR(sinphi)) * sin(thetaf + zla) +
                datdi[j][8] * (SQR(cosphi) - SQR(sinphi)) * cos(thetaf + zla);
    double de = datdi[j][7] * sinphi * cos(thetaf + zla) - datdi[j][8] * sinphi * sin(thetaf + zla);
    xcorsta[0] += dr * cosla * cosphi - de * sinla - dn * sinphi * cosla;
    xcorsta[1] += dr * sinla * cosphi + de * cosla - dn * sinphi * sinla;
    xcorsta[2] += dr * sinphi + dn * cosphi;
  }
  for (int i = 0; i < 3; ++i) xcorsta[i] /= 1e3;
}

// This subroutine gives the out-of-phase corrections induced by mantle
// anelasticity in the diurnal band.
static void st1idiu_(const double xsta[3], const double xsun[3], const double xmon[3],
                     double fac2sun, double fac2mon, double xcorsta[3]) {
  const double dhi = -0.0025, dli = -7e-4;
  // Compute the normalized position vector of the IGS station.
  double rsta = norm(xsta, 3);
  double sinphi = xsta[2] / rsta;
  double cosphi = sqrt(SQR(xsta[0]) + SQR(xsta[1])) / rsta;
  double cos2phi = SQR(cosphi) - SQR(sinphi);
  double sinla = xsta[1] / cosphi / rsta;
  double cosla = xsta[0] / cosphi / rsta;
  // Compute the normalized position vector of the Moon.
  double rmon = norm(xmon, 3);
  // Compute the normalized position vector of the Sun.
  double rsun = norm(xsun, 3);
  double drsun = dhi * -3. * sinphi * cosphi * fac2sun * xsun[2] *
                 (xsun[0] * sinla - xsun[1] * cosla) / (rsun * rsun);
  double drmon = dhi * -3. * sinphi * cosphi * fac2mon * xmon[2] *
                 (xmon[0] * sinla - xmon[1] * cosla) / (rmon * rmon);
  double dnsun =
      dli * -3. * cos2phi * fac2sun * xsun[2] * (xsun[0] * sinla - xsun[1] * cosla) / (rsun * rsun);
  double dnmon =
      dli * -3. * cos2phi * fac2mon * xmon[2] * (xmon[0] * sinla - xmon[1] * cosla) / (rmon * rmon);
  double desun =
      dli * -3. * sinphi * fac2sun * xsun[2] * (xsun[0] * cosla + xsun[1] * sinla) / (rsun * rsun);
  double demon =
      dli * -3. * sinphi * fac2mon * xmon[2] * (xmon[0] * cosla + xmon[1] * sinla) / (rmon * rmon);
  double dr = drsun + drmon;
  double dn = dnsun + dnmon;
  double de = desun + demon;
  //  Compute the corrections for the station.
  xcorsta[0] = dr * cosla * cosphi - de * sinla - dn * sinphi * cosla;
  xcorsta[1] = dr * sinla * cosphi + de * cosla - dn * sinphi * sinla;
  xcorsta[2] = dr * sinphi + dn * cosphi;
}

// This subroutine gives the out-of-phase corrections induced by mantle
// anelasticity in the semi-diurnal band.
static void st1isem_(const double xsta[3], const double xsun[3], const double xmon[3],
                     double fac2sun, double fac2mon, double xcorsta[3]) {
  const double dhi = -0.0022, dli = -7e-4;
  // Compute the normalized position vector of the IGS station.
  double rsta = norm(xsta, 3);
  double sinphi = xsta[2] / rsta;
  double cosphi = sqrt(SQR(xsta[0]) + SQR(xsta[1])) / rsta;
  double sinla = xsta[1] / cosphi / rsta;
  double cosla = xsta[0] / cosphi / rsta;
  double costwola = SQR(cosla) - SQR(sinla);
  double sintwola = cosla * 2. * sinla;
  // Compute the normalized position vector of the Moon.
  double rmon = norm(xmon, 3);
  // Compute the normalized position vector of the Sun.
  double rsun = norm(xsun, 3);
  double drsun = -3.0 / 4.0 * dhi * SQR(cosphi) * fac2sun *
                 ((SQR(xsun[0]) - SQR(xsun[1])) * sintwola - xsun[0] * 2. * xsun[1] * costwola) /
                 SQR(rsun);
  double drmon = -3.0 / 4.0 * dhi * SQR(cosphi) * fac2mon *
                 ((SQR(xmon[0]) - SQR(xmon[1])) * sintwola - xmon[0] * 2. * xmon[1] * costwola) /
                 SQR(rmon);
  double dnsun = 3.0 / 2.0 * dli * sinphi * cosphi * fac2sun *
                 ((SQR(xsun[0]) - SQR(xsun[1])) * sintwola - xsun[0] * 2. * xsun[1] * costwola) /
                 SQR(rsun);
  double dnmon = 3.0 / 2.0 * dli * sinphi * cosphi * fac2mon *
                 ((SQR(xmon[0]) - SQR(xmon[1])) * sintwola - xmon[0] * 2. * xmon[1] * costwola) /
                 SQR(rmon);
  double desun = -3.0 / 2.0 * dli * cosphi * fac2sun *
                 ((SQR(xsun[0]) - SQR(xsun[1])) * costwola + xsun[0] * 2. * xsun[1] * sintwola) /
                 SQR(rsun);
  double demon = -3.0 / 2.0 * dli * cosphi * fac2mon *
                 ((SQR(xmon[0]) - SQR(xmon[1])) * costwola + xmon[0] * 2. * xmon[1] * sintwola) /
                 SQR(rmon);
  double dr = drsun + drmon;
  double dn = dnsun + dnmon;
  double de = desun + demon;
  xcorsta[0] = dr * cosla * cosphi - de * sinla - dn * sinphi * cosla;
  xcorsta[1] = dr * sinla * cosphi + de * cosla - dn * sinphi * sinla;
  xcorsta[2] = dr * sinphi + dn * cosphi;
}

// This subroutine gives the corrections induced by the latitude dependence
// given by L^1 in Mathews et al. 1991 (See References).
static void st1l1_(const double xsta[3], const double xsun[3], const double xmon[3], double fac2sun,
                   double fac2mon, double xcorsta[3]) {
  const double l1d = 0.0012, l1sd = 0.0024;
  // Compute the normalized position vector of the IGS station.
  double rsta = norm(xsta, 3);
  double sinphi = xsta[2] / rsta;
  double cosphi = sqrt(SQR(xsta[0]) + SQR(xsta[1])) / rsta;
  double sinla = xsta[1] / cosphi / rsta;
  double cosla = xsta[0] / cosphi / rsta;
  // Compute the normalized position vector of the Moon.
  double rmon = norm(xmon, 3);
  // Compute the normalized position vector of the Sun.
  double rsun = norm(xsun, 3);
  // Compute the station corrections for the diurnal band.
  double l1 = l1d;
  double dnsun =
      -l1 * SQR(sinphi) * fac2sun * xsun[2] * (xsun[0] * cosla + xsun[1] * sinla) / SQR(rsun);
  double dnmon =
      -l1 * SQR(sinphi) * fac2mon * xmon[2] * (xmon[0] * cosla + xmon[1] * sinla) / SQR(rmon);
  double desun = l1 * sinphi * (SQR(cosphi) - SQR(sinphi)) * fac2sun * xsun[2] *
                 (xsun[0] * sinla - xsun[1] * cosla) / SQR(rsun);
  double demon = l1 * sinphi * (SQR(cosphi) - SQR(sinphi)) * fac2mon * xmon[2] *
                 (xmon[0] * sinla - xmon[1] * cosla) / SQR(rmon);
  double de = 3 * (desun + demon);
  double dn = 3 * (dnsun + dnmon);
  xcorsta[0] = -de * sinla - dn * sinphi * cosla;
  xcorsta[1] = de * cosla - dn * sinphi * sinla;
  xcorsta[2] = dn * cosphi;
  // Compute the station corrections for the semi-diurnal band.
  l1 = l1sd;
  double costwola = SQR(cosla) - SQR(sinla);
  double sintwola = 2 * cosla * sinla;
  dnsun = -l1 / 2 * sinphi * cosphi * fac2sun *
          ((SQR(xsun[0]) - SQR(xsun[1])) * costwola + xsun[0] * 2 * xsun[1] * sintwola) / SQR(rsun);
  dnmon = -l1 / 2 * sinphi * cosphi * fac2mon *
          ((SQR(xmon[0]) - SQR(xmon[1])) * costwola + xmon[0] * 2 * xmon[1] * sintwola) / SQR(rmon);
  desun = -l1 / 2 * SQR(sinphi) * cosphi * fac2sun *
          ((SQR(xsun[0]) - SQR(xsun[1])) * sintwola - xsun[0] * 2 * xsun[1] * costwola) / SQR(rsun);
  demon = -l1 / 2 * SQR(sinphi) * cosphi * fac2mon *
          ((SQR(xmon[0]) - SQR(xmon[1])) * sintwola - xmon[0] * 2 * xmon[1] * costwola) / SQR(rmon);
  de = 3 * (desun + demon);
  dn = 3 * (dnsun + dnmon);
  xcorsta[0] += -de * sinla - dn * sinphi * cosla;
  xcorsta[1] += +de * cosla - dn * sinphi * sinla;
  xcorsta[2] += dn * cosphi;
}

// This subroutine computes the station tidal displacement caused by lunar and
// solar gravitational attraction (see References).
static void dehanttideinel(gtime_t tutc, const double xsta[3], const double xsun[3],
                           const double xmon[3], double dxtide[3]) {
  // Nominal second degree and third degree love numbers and shida numbers.
  const double h20 = 0.6078, l20 = 0.0847, h3 = 0.292, l3 = 0.015;
  // Scalar product of station vector with sun/moon vector.
  double rsta = sqrt(SQR(xsta[0]) + SQR(xsta[1]) + SQR(xsta[2]));
  double rsun = sqrt(SQR(xsun[0]) + SQR(xsun[1]) + SQR(xsun[2]));
  double rmon = sqrt(SQR(xmon[0]) + SQR(xmon[1]) + SQR(xmon[2]));
  double scs = dot(xsta, xsun, 3);
  double scm = dot(xsta, xmon, 3);
  double scsun = scs / rsta / rsun;
  double scmon = scm / rsta / rmon;
  // Computation of new h2 and l2.
  double cosphi = sqrt(SQR(xsta[0]) + SQR(xsta[1])) / rsta;
  double h2 = h20 - (1 - 3.0 / 2.0 * SQR(cosphi)) * 6e-4;
  double l2 = l20 + (1 - 3.0 / 2.0 * SQR(cosphi)) * 2e-4;
  // P2 term
  double p2sun = 3 * (h2 / 2 - l2) * SQR(scsun) - h2 / 2;
  double p2mon = 3 * (h2 / 2 - l2) * SQR(scmon) - h2 / 2;
  // P3 term
  double p3sun = 5.0 / 2.0 * (h3 - l3 * 3) * (SQR(scsun) * scsun) + 3.0 / 2.0 * (l3 - h3) * scsun;
  double p3mon = 5.0 / 2.0 * (h3 - l3 * 3) * (SQR(scmon) * scmon) + 3.0 / 2.0 * (l3 - h3) * scmon;
  // Term in direction of sun/moon vector.
  double x2sun = 3 * l2 * scsun;
  double x2mon = 3 * l2 * scmon;
  double x3sun = 3.0 / 2.0 * l3 * (SQR(scsun) * 5 - 1);
  double x3mon = 3.0 / 2.0 * l3 * (SQR(scmon) * 5 - 1);
  // Factors for sun/moon using iau current best estimates (see references).
  const double mass_ratio_sun = 332946.0482;
  const double mass_ratio_moon = 0.0123000371;
  const double re = 6378136.6;
  double resun = re / rsun;
  double fac2sun = mass_ratio_sun * re * SQR(resun) * resun;
  double remon = re / rmon;
  double fac2mon = mass_ratio_moon * re * SQR(remon) * remon;
  double fac3sun = fac2sun * resun;
  double fac3mon = fac2mon * remon;
  // Total displacement.
  for (int i = 0; i < 3; i++) {
    dxtide[i] = fac2sun * (x2sun * xsun[i] / rsun + p2sun * xsta[i] / rsta) +
                fac2mon * (x2mon * xmon[i] / rmon + p2mon * xsta[i] / rsta) +
                fac3sun * (x3sun * xsun[i] / rsun + p3sun * xsta[i] / rsta) +
                fac3mon * (x3mon * xmon[i] / rmon + p3mon * xsta[i] / rsta);
  }
  // Corrections for the out-of-phase part of love numbers (part h_2^(0)i
  // and l_2^(0)i )
  //
  // First, for the diurnal band.
  double xcorsta[3] = {0};
  st1idiu_(xsta, xsun, xmon, fac2sun, fac2mon, xcorsta);
  for (int i = 0; i < 3; ++i) dxtide[i] += xcorsta[i];
  // Second, for the semi-diurnal band.
  st1isem_(xsta, xsun, xmon, fac2sun, fac2mon, xcorsta);
  for (int i = 0; i < 3; ++i) dxtide[i] += xcorsta[i];
  // Corrections for the latitude dependence of love numbers (part l^(1) )
  st1l1_(xsta, xsun, xmon, fac2sun, fac2mon, xcorsta);
  for (int i = 0; i < 3; ++i) dxtide[i] += xcorsta[i];
  // Consider corrections for step 2.
  // Corrections for the diurnal band:
  //  First, we need to know the date converted in julian centuries
  double ep[6];
  time2epoch(tutc, ep);
  double fhr = ep[3] + ep[4] / 60.0 + ep[5] / 3600.0;

  // Terrestrial time.
  gtime_t tgps = utc2gpst(tutc);
  const double ep2000[] = {2000, 1, 1, 11, 59, 08.816};  // GPST of J2000.0
  double t = timediff(tgps, epoch2time(ep2000)) / 86400.0 / 36525.0;

  // Second, we can call the subroutine step2diu, for the diurnal band
  // corrections, (in-phase and out-of-phase frequency dependence):
  step2diu_(xsta, fhr, t, xcorsta);
  for (int i = 0; i < 3; ++i) dxtide[i] += xcorsta[i];
  // Corrections for the long-period band,
  // (in-phase and out-of-phase frequency dependence):
  step2lon_(xsta, t, xcorsta);
  for (int i = 0; i < 3; ++i) dxtide[i] += xcorsta[i];

#ifdef RTK_DISABLED
  // Consider corrections for step 3.
  // Uncorrect for the permanent tide.
  double sinphi = xsta[1] / rsta;
  cosphi = sqrt(SQR(xsta[0]) + SQR(xsta[1])) / rsta;
  double cosla = xsta[0] / cosphi / rsta;
  double sinla = xsta[1] / cosphi / rsta;
  double dr = -sqrt(5.0 / 4.0 / PI) * h2 * 0.3146 * (SQR(sinphi) * 3.0 / 2.0 - 0.5);
  double dn = -sqrt(5.0 / 4.0 / PI) * l2 * 0.3146 * 3 * cosphi * sinphi;
  dxtide[0] = dxtide[0] - dr * cosla * cosphi + dn * cosla * sinphi;
  dxtide[1] = dxtide[1] - dr * sinla * cosphi + dn * sinla * sinphi;
  dxtide[2] = dxtide[2] - dr * sinphi - dn * cosphi;
#endif
}

// The following few functions are in support of hardisp. This code is
// a translation of the respective iers fortran code to C and to RTKLIB, and
// the reference code is included in the lib/iers/src/hardisp/ directory.

// Stable Shell sort x and the index k.
static void shell_sort_stable(double *x, int *k, int n) {
  for (int i = 0; i < n; ++i) k[i] = i;

  int igap = n;
  while (igap > 1) {
    igap /= 2;
    int imax = n - igap, iex;
    do {
      iex = 0;
      for (int i = 0; i < imax; i++) {
        int ipl = i + igap;
        if (x[i] <= x[ipl]) continue;
        double sv = x[i];
        int ik = k[i];
        x[i] = x[ipl];
        k[i] = k[ipl];
        x[ipl] = sv;
        k[ipl] = ik;
        iex++;
      }
    } while (iex > 0);
  }

  // Now sort k's (for identical values of x, if any).
  for (int j = 0; j + 1 < n; j++) {
    if (x[j] != x[j + 1]) continue;
    // Have at least two x's with the same value. See how long this is true.
    int l = j + 1;
    while (l + 1 < n) {
      if (x[l] != x[l + 1]) break;
      l++;
    };
    // j and l are the indices within which x(i) does not change - sort k
    int nj = l - j + 1;
    igap = nj;
    while (igap > 1) {
      igap /= 2;
      int imax = nj - igap, iex;
      do {
        iex = 0;
        for (int i = 0; i < imax; i++) {
          int ipl = j + i + igap;
          if (k[j + i] <= k[ipl]) continue;
          int ik = k[j + i];
          k[j + i] = k[ipl];
          k[ipl] = ik;
          iex++;
        }
      } while (iex > 0);
    };
    j = l;
  }
}
// Cubic spline.
static inline double cubic_spline_q(double u1, double x1, double u2, double x2) {
  return (u1 / (x1 * x1) - u2 / (x2 * x2)) / (1 / x1 - 1 / x2);
}
// Setup cubic spline interpolation.
static void cubic_spline(int nn, const double *x, const double *u, double *s, double *a) {
  int n = abs(nn);
  if (n <= 3) {
    // Series too short for cubic spline - use straight lines.
    for (int i = 0; i < n; i++) s[i] = 0;
    return;
  }
  double q1 = cubic_spline_q(u[1] - u[0], x[1] - x[0], u[2] - u[0], x[2] - x[0]);
  double qn = cubic_spline_q(u[n - 2] - u[n - 1], x[n - 2] - x[n - 1], u[n - 3] - u[n - 1],
                             x[n - 3] - x[n - 1]);
  if (nn <= 0) {
    q1 = s[0];
    qn = s[1];
  }
  s[0] = ((u[1] - u[0]) / (x[1] - x[0]) - q1) * 6;
  int n1 = n - 1;
  for (int i = 1; i < n1; i++) {
    s[i] = (u[i - 1] / (x[i] - x[i - 1]) - u[i] * (1 / (x[i] - x[i - 1]) + 1 / (x[i + 1] - x[i])) +
            u[i + 1] / (x[i + 1] - x[i])) *
           6;
  }
  s[n - 1] = (qn + (u[n1 - 1] - u[n - 1]) / (x[n - 1] - x[n1 - 1])) * 6;
  a[0] = (x[1] - x[0]) * 2;
  a[1] = (x[1] - x[0]) * 1.5 + (x[2] - x[1]) * 2;
  s[1] -= s[0] * 0.5;
  for (int i = 2; i < n1; i++) {
    double c = (x[i] - x[i - 1]) / a[i - 1];
    a[i] = (x[i + 1] - x[i - 1]) * 2 - c * (x[i] - x[i - 1]);
    s[i] -= c * s[i - 1];
  }
  double c = (x[n - 1] - x[n1 - 1]) / a[n1 - 1];
  a[n - 1] = (2 - c) * (x[n - 1] - x[n1 - 1]);
  s[n - 1] -= c * s[n1 - 1];
  // Back substitute.
  s[n - 1] /= a[n - 1];
  for (int i = n1 - 1; i >= 0; i--) s[i] = (s[i] - (x[i + 1] - x[i]) * s[i + 1]) / a[i];
}
// Evaluate the cubic spline, as initialize by cubic_spline.
static double cubic_spline_eval(double y, int nn, const double *x, const double *u,
                                const double *s) {
  nn = abs(nn);
  if (nn <= 0) return 0;
  if (nn == 1) return u[0];
  // If y is out of range, substitute endpoint values.
  if (y <= x[0]) return u[0];
  if (y >= x[nn - 1]) return u[nn - 1];
  // Locate interval (x(k1),x(k2)) which contains y.
  int k1 = 0, k2 = 0;
  for (int k = 1; k < nn; k++) {
    if (x[k - 1] < y && x[k] >= y) {
      k1 = k - 1;
      k2 = k;
    }
  }
  // Evaluate and then interpolate.
  double dy = x[k2] - y, dy1 = y - x[k1], dk = x[k2] - x[k1];
  double f1 = (s[k1] * dy * dy * dy + s[k2] * dy1 * dy1 * dy1) / (dk * 6);
  double f2 = dy1 * (u[k2] / dk - s[k2] * dk / 6);
  double f3 = dy * (u[k1] / dk - s[k1] * dk / 6);
  return f1 + f2 + f3;
}

#define NL 8  // Processing block size, was 600 in original code but not needed here.
#define NT 342
#define NCON 20
#define NTIN 11
// This subroutine returns the frequency and phase of a tidal constituent when
// its Doodson number is given as input, and at the given time.
static void tdfrph(const gtime_t tut, const int idood[6], double *freq, double *phase) {
  // Cache with the time as the key. This cache does not depend on idood[].
  static THREADLOCAL gtime_t ctime = {0};
  static THREADLOCAL double d[6], dd[6];

  if (fabs(timediff(tut, ctime)) > 0.001) {
    // Julian centuries from J2000.0 (ET).
    double ep[6];
    time2epoch(tut, ep);
    double dayfr = ep[3] / 24.0 + ep[4] / 1440.0 + ep[5] / 86400.0;
    gtime_t tgps = utc2gpst(tut);
    const double ep2000[] = {2000, 1, 1, 11, 59, 08.816};  // GPST of J2000.0
    double t = timediff(tgps, epoch2time(ep2000)) / 86400.0 / 36525.0;
    ctime = tut;
    // IERS expressions for the Delaunay arguments, in degrees.
    double f1 =
        t * (t * (t * (t * -6.8e-8 + 1.43431e-5) + 0.0088553333) + 477198.8675605) + 134.96340251;
    double f2 =
        t * (t * (t * (t * -3.2e-9 + 3.78e-8) - 1.536667e-4) + 35999.0502911389) + 357.5291091806;
    double f3 =
        t * (t * (t * (t * 1.2e-9 - 2.881e-7) - 0.003542) + 483202.0174577222) + 93.27209062;
    double f4 = t * (t * (t * (t * -8.8e-9 + 1.8314e-6) - 0.0017696111) + 445267.1114469445) +
                297.8501954694;
    double f5 =
        t * (t * (t * (t * -1.65e-8 + 2.1394e-6) + 0.0020756111) - 1934.1362619722) + 125.04455501;
    // Convert to Doodson (Darwin) variables.
    d[0] = dayfr * 360 - f4;
    d[1] = f3 + f5;
    d[2] = d[1] - f4;
    d[3] = d[1] - f1;
    d[4] = -f5;
    d[5] = d[2] - f2;
    // Find frequencies of Delauney variables (in cycles/day), and from these
    // the same for the Doodson arguments.
    double fd1 = t * 1.3e-9 + 0.0362916471;
    double fd2 = 0.0027377786;
    double fd3 = 0.0367481951 - t * 5e-10;
    double fd4 = 0.033863192 - t * 3e-10;
    double fd5 = t * 3e-10 - 1.470938e-4;
    dd[0] = 1 - fd4;
    dd[1] = fd3 + fd5;
    dd[2] = dd[1] - fd4;
    dd[3] = dd[1] - fd1;
    dd[4] = -fd5;
    dd[5] = dd[2] - fd2;
  }
  // End of intialization (likely to be called only once)
  // Compute phase and frequency of the given tidal constituent.
  *freq = 0;
  *phase = 0;
  for (int i = 0; i < 6; i++) {
    *freq += idood[i] * dd[i];
    *phase += idood[i] * d[i];
  }
  // Adjust phases so that they fall in the positive range 0 to 360.
  *phase = fmod(*phase, 360);
  if (*phase < 0) *phase += 360;
}
// This subroutine returns the ocean loading displacement amplitude,
// frequency, and phase of a set of tidal constituents.
static int admint(const gtime_t time, const double *ampin, const int idtin[][6], const double *phin,
                  double *amp, double *f, double *p, int nin) {
  static const double tamp[NT] = {
      0.632208,  0.294107,  0.121046,  0.079915,  0.023818,  -0.023589, 0.022994,  0.019333,
      -0.017871, 0.017192,  0.016018,  0.004671,  -0.004662, -0.004519, 0.00447,   0.004467,
      0.002589,  -0.002455, -0.002172, 0.001972,  0.001947,  0.001914,  -0.001898, 0.001802,
      0.001304,  0.00117,   0.00113,   0.001061,  -0.001022, -0.001017, 0.001014,  9.01e-4,
      -8.57e-4,  8.55e-4,   8.55e-4,   7.72e-4,   7.41e-4,   7.41e-4,   -7.21e-4,  6.98e-4,
      6.58e-4,   6.54e-4,   -6.53e-4,  6.33e-4,   6.26e-4,   -5.98e-4,  5.9e-4,    5.44e-4,
      4.79e-4,   -4.64e-4,  4.13e-4,   -3.9e-4,   3.73e-4,   3.66e-4,   3.66e-4,   -3.6e-4,
      -3.55e-4,  3.54e-4,   3.29e-4,   3.28e-4,   3.19e-4,   3.02e-4,   2.79e-4,   -2.74e-4,
      -2.72e-4,  2.48e-4,   -2.25e-4,  2.24e-4,   -2.23e-4,  -2.16e-4,  2.11e-4,   2.09e-4,
      1.94e-4,   1.85e-4,   -1.74e-4,  -1.71e-4,  1.59e-4,   1.31e-4,   1.27e-4,   1.2e-4,
      1.18e-4,   1.17e-4,   1.08e-4,   1.07e-4,   1.05e-4,   -1.02e-4,  1.02e-4,   9.9e-5,
      -9.6e-5,   9.5e-5,    -8.9e-5,   -8.5e-5,   -8.4e-5,   -8.1e-5,   -7.7e-5,   -7.2e-5,
      -6.7e-5,   6.6e-5,    6.4e-5,    6.3e-5,    6.3e-5,    6.3e-5,    6.2e-5,    6.2e-5,
      -6e-5,     5.6e-5,    5.3e-5,    5.1e-5,    5e-5,      0.368645,  -0.262232, -0.121995,
      -0.050208, 0.050031,  -0.04947,  0.02062,   0.020613,  0.011279,  -0.00953,  -0.009469,
      -0.008012, 0.007414,  -0.0073,   0.007227,  -0.007131, -0.006644, 0.005249,  0.004137,
      0.004087,  0.003944,  0.003943,  0.00342,   0.003418,  0.002885,  0.002884,  0.00216,
      -0.001936, 0.001934,  -0.001798, 0.00169,   0.001689,  0.001516,  0.001514,  -0.001511,
      0.001383,  0.001372,  0.001371,  -0.001253, -0.001075, 0.00102,   9.01e-4,   8.65e-4,
      -7.94e-4,  7.88e-4,   7.82e-4,   -7.47e-4,  -7.45e-4,  6.7e-4,    -6.03e-4,  -5.97e-4,
      5.42e-4,   5.42e-4,   -5.41e-4,  -4.69e-4,  -4.4e-4,   4.38e-4,   4.22e-4,   4.1e-4,
      -3.74e-4,  -3.65e-4,  3.45e-4,   3.35e-4,   -3.21e-4,  -3.19e-4,  3.07e-4,   2.91e-4,
      2.9e-4,    -2.89e-4,  2.86e-4,   2.75e-4,   2.71e-4,   2.63e-4,   -2.45e-4,  2.25e-4,
      2.25e-4,   2.21e-4,   -2.02e-4,  -2e-4,     -1.99e-4,  1.92e-4,   1.83e-4,   1.83e-4,
      1.83e-4,   -1.7e-4,   1.69e-4,   1.68e-4,   1.62e-4,   1.49e-4,   -1.47e-4,  -1.41e-4,
      1.38e-4,   1.36e-4,   1.36e-4,   1.27e-4,   1.27e-4,   -1.26e-4,  -1.21e-4,  -1.21e-4,
      1.17e-4,   -1.16e-4,  -1.14e-4,  -1.14e-4,  -1.14e-4,  1.14e-4,   1.13e-4,   1.09e-4,
      1.08e-4,   1.06e-4,   -1.06e-4,  -1.06e-4,  1.05e-4,   1.04e-4,   -1.03e-4,  -1e-4,
      -1e-4,     -1e-4,     9.9e-5,    -9.8e-5,   9.3e-5,    9.3e-5,    9e-5,      -8.8e-5,
      8.3e-5,    -8.3e-5,   -8.2e-5,   -8.1e-5,   -7.9e-5,   -7.7e-5,   -7.5e-5,   -7.5e-5,
      -7.5e-5,   7.1e-5,    7.1e-5,    -7.1e-5,   6.8e-5,    6.8e-5,    6.5e-5,    6.5e-5,
      6.4e-5,    6.4e-5,    6.4e-5,    -6.4e-5,   -6e-5,     5.6e-5,    5.6e-5,    5.3e-5,
      5.3e-5,    5.3e-5,    -5.3e-5,   5.3e-5,    5.3e-5,    5.2e-5,    5e-5,      -0.066607,
      -0.035184, -0.030988, 0.027929,  -0.027616, -0.012753, -0.006728, -0.005837, -0.005286,
      -0.004921, -0.002884, -0.002583, -0.002422, 0.00231,   0.002283,  -0.002037, 0.001883,
      -0.001811, -0.001687, -0.001004, -9.25e-4,  -8.44e-4,  7.66e-4,   7.66e-4,   -7e-4,
      -4.95e-4,  -4.92e-4,  4.91e-4,   4.83e-4,   4.37e-4,   -4.16e-4,  -3.84e-4,  3.74e-4,
      -3.12e-4,  -2.88e-4,  -2.73e-4,  2.59e-4,   2.45e-4,   -2.32e-4,  2.29e-4,   -2.16e-4,
      2.06e-4,   -2.04e-4,  -2.02e-4,  2e-4,      1.95e-4,   -1.9e-4,   1.87e-4,   1.8e-4,
      -1.79e-4,  1.7e-4,    1.53e-4,   -1.37e-4,  -1.19e-4,  -1.19e-4,  -1.12e-4,  -1.1e-4,
      -1.1e-4,   1.07e-4,   -9.5e-5,   -9.5e-5,   -9.1e-5,   -9e-5,     -8.1e-5,   -7.9e-5,
      -7.9e-5,   7.7e-5,    -7.3e-5,   6.9e-5,    -6.7e-5,   -6.6e-5,   6.5e-5,    6.4e-5,
      -6.2e-5,   6e-5,      5.9e-5,    -5.6e-5,   5.5e-5,    -5.1e-5};
  static const int idd[NT][6] = {
      {2, 0, 0, 0, 0, 0},    {2, 2, -2, 0, 0, 0},   {2, -1, 0, 1, 0, 0},   {2, 2, 0, 0, 0, 0},
      {2, 2, 0, 0, 1, 0},    {2, 0, 0, 0, -1, 0},   {2, -1, 2, -1, 0, 0},  {2, -2, 2, 0, 0, 0},
      {2, 1, 0, -1, 0, 0},   {2, 2, -3, 0, 0, 1},   {2, -2, 0, 2, 0, 0},   {2, -3, 2, 1, 0, 0},
      {2, 1, -2, 1, 0, 0},   {2, -1, 0, 1, -1, 0},  {2, 3, 0, -1, 0, 0},   {2, 1, 0, 1, 0, 0},
      {2, 2, 0, 0, 2, 0},    {2, 2, -1, 0, 0, -1},  {2, 0, -1, 0, 0, 1},   {2, 1, 0, 1, 1, 0},
      {2, 3, 0, -1, 1, 0},   {2, 0, 1, 0, 0, -1},   {2, 0, -2, 2, 0, 0},   {2, -3, 0, 3, 0, 0},
      {2, -2, 3, 0, 0, -1},  {2, 4, 0, 0, 0, 0},    {2, -1, 1, 1, 0, -1},  {2, -1, 3, -1, 0, -1},
      {2, 2, 0, 0, -1, 0},   {2, -1, -1, 1, 0, 1},  {2, 4, 0, 0, 1, 0},    {2, -3, 4, -1, 0, 0},
      {2, -1, 2, -1, -1, 0}, {2, 3, -2, 1, 0, 0},   {2, 1, 2, -1, 0, 0},   {2, -4, 2, 2, 0, 0},
      {2, 4, -2, 0, 0, 0},   {2, 0, 2, 0, 0, 0},    {2, -2, 2, 0, -1, 0},  {2, 2, -4, 0, 0, 2},
      {2, 2, -2, 0, -1, 0},  {2, 1, 0, -1, -1, 0},  {2, -1, 1, 0, 0, 0},   {2, 2, -1, 0, 0, 1},
      {2, 2, 1, 0, 0, -1},   {2, -2, 0, 2, -1, 0},  {2, -2, 4, -2, 0, 0},  {2, 2, 2, 0, 0, 0},
      {2, -4, 4, 0, 0, 0},   {2, -1, 0, -1, -2, 0}, {2, 1, 2, -1, 1, 0},   {2, -1, -2, 3, 0, 0},
      {2, 3, -2, 1, 1, 0},   {2, 4, 0, -2, 0, 0},   {2, 0, 0, 2, 0, 0},    {2, 0, 2, -2, 0, 0},
      {2, 0, 2, 0, 1, 0},    {2, -3, 3, 1, 0, -1},  {2, 0, 0, 0, -2, 0},   {2, 4, 0, 0, 2, 0},
      {2, 4, -2, 0, 1, 0},   {2, 0, 0, 0, 0, 2},    {2, 1, 0, 1, 2, 0},    {2, 0, -2, 0, -2, 0},
      {2, -2, 1, 0, 0, 1},   {2, -2, 1, 2, 0, -1},  {2, -1, 1, -1, 0, 1},  {2, 5, 0, -1, 0, 0},
      {2, 1, -3, 1, 0, 1},   {2, -2, -1, 2, 0, 1},  {2, 3, 0, -1, 2, 0},   {2, 1, -2, 1, -1, 0},
      {2, 5, 0, -1, 1, 0},   {2, -4, 0, 4, 0, 0},   {2, -3, 2, 1, -1, 0},  {2, -2, 1, 1, 0, 0},
      {2, 4, 0, -2, 1, 0},   {2, 0, 0, 2, 1, 0},    {2, -5, 4, 1, 0, 0},   {2, 0, 2, 0, 2, 0},
      {2, -1, 2, 1, 0, 0},   {2, 5, -2, -1, 0, 0},  {2, 1, -1, 0, 0, 0},   {2, 2, -2, 0, 0, 2},
      {2, -5, 2, 3, 0, 0},   {2, -1, -2, 1, -2, 0}, {2, -3, 5, -1, 0, -1}, {2, -1, 0, 0, 0, 1},
      {2, -2, 0, 0, -2, 0},  {2, 0, -1, 1, 0, 0},   {2, -3, 1, 1, 0, 1},   {2, 3, 0, -1, -1, 0},
      {2, 1, 0, 1, -1, 0},   {2, -1, 2, 1, 1, 0},   {2, 0, -3, 2, 0, 1},   {2, 1, -1, -1, 0, 1},
      {2, -3, 0, 3, -1, 0},  {2, 0, -2, 2, -1, 0},  {2, -4, 3, 2, 0, -1},  {2, -1, 0, 1, -2, 0},
      {2, 5, 0, -1, 2, 0},   {2, -4, 5, 0, 0, -1},  {2, -2, 4, 0, 0, -2},  {2, -1, 0, 1, 0, 2},
      {2, -2, -2, 4, 0, 0},  {2, 3, -2, -1, -1, 0}, {2, -2, 5, -2, 0, -1}, {2, 0, -1, 0, -1, 1},
      {2, 5, -2, -1, 1, 0},  {1, 1, 0, 0, 0, 0},    {1, -1, 0, 0, 0, 0},   {1, 1, -2, 0, 0, 0},
      {1, -2, 0, 1, 0, 0},   {1, 1, 0, 0, 1, 0},    {1, -1, 0, 0, -1, 0},  {1, 2, 0, -1, 0, 0},
      {1, 0, 0, 1, 0, 0},    {1, 3, 0, 0, 0, 0},    {1, -2, 2, -1, 0, 0},  {1, -2, 0, 1, -1, 0},
      {1, -3, 2, 0, 0, 0},   {1, 0, 0, -1, 0, 0},   {1, 1, 0, 0, -1, 0},   {1, 3, 0, 0, 1, 0},
      {1, 1, -3, 0, 0, 1},   {1, -3, 0, 2, 0, 0},   {1, 1, 2, 0, 0, 0},    {1, 0, 0, 1, 1, 0},
      {1, 2, 0, -1, 1, 0},   {1, 0, 2, -1, 0, 0},   {1, 2, -2, 1, 0, 0},   {1, 3, -2, 0, 0, 0},
      {1, -1, 2, 0, 0, 0},   {1, 1, 1, 0, 0, -1},   {1, 1, -1, 0, 0, 1},   {1, 4, 0, -1, 0, 0},
      {1, -4, 2, 1, 0, 0},   {1, 0, -2, 1, 0, 0},   {1, -2, 2, -1, -1, 0}, {1, 3, 0, -2, 0, 0},
      {1, -1, 0, 2, 0, 0},   {1, -1, 0, 0, -2, 0},  {1, 3, 0, 0, 2, 0},    {1, -3, 2, 0, -1, 0},
      {1, 4, 0, -1, 1, 0},   {1, 0, 0, -1, -1, 0},  {1, 1, -2, 0, -1, 0},  {1, -3, 0, 2, -1, 0},
      {1, 1, 0, 0, 2, 0},    {1, 1, -1, 0, 0, -1},  {1, -1, -1, 0, 0, 1},  {1, 0, 2, -1, 1, 0},
      {1, -1, 1, 0, 0, -1},  {1, -1, -2, 2, 0, 0},  {1, 2, -2, 1, 1, 0},   {1, -4, 0, 3, 0, 0},
      {1, -1, 2, 0, 1, 0},   {1, 3, -2, 0, 1, 0},   {1, 2, 0, -1, -1, 0},  {1, 0, 0, 1, -1, 0},
      {1, -2, 2, 1, 0, 0},   {1, 4, -2, -1, 0, 0},  {1, -3, 3, 0, 0, -1},  {1, -2, 1, 1, 0, -1},
      {1, -2, 3, -1, 0, -1}, {1, 0, -2, 1, -1, 0},  {1, -2, -1, 1, 0, 1},  {1, 4, -2, 1, 0, 0},
      {1, -4, 4, -1, 0, 0},  {1, -4, 2, 1, -1, 0},  {1, 5, -2, 0, 0, 0},   {1, 3, 0, -2, 1, 0},
      {1, -5, 2, 2, 0, 0},   {1, 2, 0, 1, 0, 0},    {1, 1, 3, 0, 0, -1},   {1, -2, 0, 1, -2, 0},
      {1, 4, 0, -1, 2, 0},   {1, 1, -4, 0, 0, 2},   {1, 5, 0, -2, 0, 0},   {1, -1, 0, 2, 1, 0},
      {1, -2, 1, 0, 0, 0},   {1, 4, -2, 1, 1, 0},   {1, -3, 4, -2, 0, 0},  {1, -1, 3, 0, 0, -1},
      {1, 3, -3, 0, 0, 1},   {1, 5, -2, 0, 1, 0},   {1, 1, 2, 0, 1, 0},    {1, 2, 0, 1, 1, 0},
      {1, -5, 4, 0, 0, 0},   {1, -2, 0, -1, -2, 0}, {1, 5, 0, -2, 1, 0},   {1, 1, 2, -2, 0, 0},
      {1, 1, -2, 2, 0, 0},   {1, -2, 2, 1, 1, 0},   {1, 0, 3, -1, 0, -1},  {1, 2, -3, 1, 0, 1},
      {1, -2, -2, 3, 0, 0},  {1, -1, 2, -2, 0, 0},  {1, -4, 3, 1, 0, -1},  {1, -4, 0, 3, -1, 0},
      {1, -1, -2, 2, -1, 0}, {1, -2, 0, 3, 0, 0},   {1, 4, 0, -3, 0, 0},   {1, 0, 1, 1, 0, -1},
      {1, 2, -1, -1, 0, 1},  {1, 2, -2, 1, -1, 0},  {1, 0, 0, -1, -2, 0},  {1, 2, 0, 1, 2, 0},
      {1, 2, -2, -1, -1, 0}, {1, 0, 0, 1, 2, 0},    {1, 0, 1, 0, 0, 0},    {1, 2, -1, 0, 0, 0},
      {1, 0, 2, -1, -1, 0},  {1, -1, -2, 0, -2, 0}, {1, -3, 1, 0, 0, 1},   {1, 3, -2, 0, -1, 0},
      {1, -1, -1, 0, -1, 1}, {1, 4, -2, -1, 1, 0},  {1, 2, 1, -1, 0, -1},  {1, 0, -1, 1, 0, 1},
      {1, -2, 4, -1, 0, 0},  {1, 4, -4, 1, 0, 0},   {1, -3, 1, 2, 0, -1},  {1, -3, 3, 0, -1, -1},
      {1, 1, 2, 0, 2, 0},    {1, 1, -2, 0, -2, 0},  {1, 3, 0, 0, 3, 0},    {1, -1, 2, 0, -1, 0},
      {1, -2, 1, -1, 0, 1},  {1, 0, -3, 1, 0, 1},   {1, -3, -1, 2, 0, 1},  {1, 2, 0, -1, 2, 0},
      {1, 6, -2, -1, 0, 0},  {1, 2, 2, -1, 0, 0},   {1, -1, 1, 0, -1, -1}, {1, -2, 3, -1, -1, -1},
      {1, -1, 0, 0, 0, 2},   {1, -5, 0, 4, 0, 0},   {1, 1, 0, 0, 0, -2},   {1, -2, 1, 1, -1, -1},
      {1, 1, -1, 0, 1, 1},   {1, 1, 2, 0, 0, -2},   {1, -3, 1, 1, 0, 0},   {1, -4, 4, -1, -1, 0},
      {1, 1, 0, -2, -1, 0},  {1, -2, -1, 1, -1, 1}, {1, -3, 2, 2, 0, 0},   {1, 5, -2, -2, 0, 0},
      {1, 3, -4, 2, 0, 0},   {1, 1, -2, 0, 0, 2},   {1, -1, 4, -2, 0, 0},  {1, 2, 2, -1, 1, 0},
      {1, -5, 2, 2, -1, 0},  {1, 1, -3, 0, -1, 1},  {1, 1, 1, 0, 1, -1},   {1, 6, -2, -1, 1, 0},
      {1, -2, 2, -1, -2, 0}, {1, 4, -2, 1, 2, 0},   {1, -6, 4, 1, 0, 0},   {1, 5, -4, 0, 0, 0},
      {1, -3, 4, 0, 0, 0},   {1, 1, 2, -2, 1, 0},   {1, -2, 1, 0, -1, 0},  {0, 2, 0, 0, 0, 0},
      {0, 1, 0, -1, 0, 0},   {0, 0, 2, 0, 0, 0},    {0, 0, 0, 0, 1, 0},    {0, 2, 0, 0, 1, 0},
      {0, 3, 0, -1, 0, 0},   {0, 1, -2, 1, 0, 0},   {0, 2, -2, 0, 0, 0},   {0, 3, 0, -1, 1, 0},
      {0, 0, 1, 0, 0, -1},   {0, 2, 0, -2, 0, 0},   {0, 2, 0, 0, 2, 0},    {0, 3, -2, 1, 0, 0},
      {0, 1, 0, -1, -1, 0},  {0, 1, 0, -1, 1, 0},   {0, 4, -2, 0, 0, 0},   {0, 1, 0, 1, 0, 0},
      {0, 0, 3, 0, 0, -1},   {0, 4, 0, -2, 0, 0},   {0, 3, -2, 1, 1, 0},   {0, 3, -2, -1, 0, 0},
      {0, 4, -2, 0, 1, 0},   {0, 0, 2, 0, 1, 0},    {0, 1, 0, 1, 1, 0},    {0, 4, 0, -2, 1, 0},
      {0, 3, 0, -1, 2, 0},   {0, 5, -2, -1, 0, 0},  {0, 1, 2, -1, 0, 0},   {0, 1, -2, 1, -1, 0},
      {0, 1, -2, 1, 1, 0},   {0, 2, -2, 0, -1, 0},  {0, 2, -3, 0, 0, 1},   {0, 2, -2, 0, 1, 0},
      {0, 0, 2, -2, 0, 0},   {0, 1, -3, 1, 0, 1},   {0, 0, 0, 0, 2, 0},    {0, 0, 1, 0, 0, 1},
      {0, 1, 2, -1, 1, 0},   {0, 3, 0, -3, 0, 0},   {0, 2, 1, 0, 0, -1},   {0, 1, -1, -1, 0, 1},
      {0, 1, 0, 1, 2, 0},    {0, 5, -2, -1, 1, 0},  {0, 2, -1, 0, 0, 1},   {0, 2, 2, -2, 0, 0},
      {0, 1, -1, 0, 0, 0},   {0, 5, 0, -3, 0, 0},   {0, 2, 0, -2, 1, 0},   {0, 1, 1, -1, 0, -1},
      {0, 3, -4, 1, 0, 0},   {0, 0, 2, 0, 2, 0},    {0, 2, 0, -2, -1, 0},  {0, 4, -3, 0, 0, 1},
      {0, 3, -1, -1, 0, 1},  {0, 0, 2, 0, 0, -2},   {0, 3, -3, 1, 0, 1},   {0, 2, -4, 2, 0, 0},
      {0, 4, -2, -2, 0, 0},  {0, 3, 1, -1, 0, -1},  {0, 5, -4, 1, 0, 0},   {0, 3, -2, -1, -1, 0},
      {0, 3, -2, 1, 2, 0},   {0, 4, -4, 0, 0, 0},   {0, 6, -2, -2, 0, 0},  {0, 5, 0, -3, 1, 0},
      {0, 4, -2, 0, 2, 0},   {0, 2, 2, -2, 1, 0},   {0, 0, 4, 0, 0, -2},   {0, 3, -1, 0, 0, 0},
      {0, 3, -3, -1, 0, 1},  {0, 4, 0, -2, 2, 0},   {0, 1, -2, -1, -1, 0}, {0, 2, -1, 0, 0, -1},
      {0, 4, -4, 2, 0, 0},   {0, 2, 1, 0, 1, -1},   {0, 3, -2, -1, 1, 0},  {0, 4, -3, 0, 1, 1},
      {0, 2, 0, 0, 3, 0},    {0, 6, -4, 0, 0, 0}};
  double rl[NCON] = {0}, aim[NCON] = {0}, rf[NCON] = {0};
  int k = 0;
  for (int ll = 0; ll < nin; ll++) {
    // See if Doodson numbers match.
    int kk, ii = 0;
    for (kk = 0; kk < NT; kk++) {
      ii = 0;
      for (int i = 0; i < 6; i++) ii += abs(idd[kk][i] - idtin[ll][i]);
      if (ii == 0) break;
    }
    // If you have a match, put line into array.
    if (ii == 0 && k < NCON) {
      rl[k] = ampin[ll] * cos(phin[ll] * D2R) / fabs(tamp[kk]);
      aim[k] = ampin[ll] * sin(phin[ll] * D2R) / fabs(tamp[kk]);
      // Now have double and imaginary parts of admittance, scaled by
      // Cartwright- Edden amplitude. Admittance phase is whatever was used in
      // the original expression. (Usually phase is given relative to some
      // reference, but amplitude is in absolute units). Next get frequency.
      double fr, pr;
      tdfrph(time, idd[kk], &fr, &pr);
      rf[k] = fr;
      k++;
    }
  }
  // Done going through constituents; there are k of them.
  // Have specified admittance at a number of points. Sort these by frequency
  // and separate diurnal and semidiurnal, recopying admittances to get them
  // in order using Shell Sort.
  int key[NCON];
  shell_sort_stable(rf, key, k);
  int nlp = 0, ndi = 0, nsd = 0;
  double scr[NCON];  // Scratch working space.
  for (int i = 0; i < k; i++) {
    if (rf[i] < 0.5) nlp++;
    if (rf[i] < 1.5 && rf[i] > 0.5) ndi++;
    if (rf[i] < 2.5 && rf[i] > 1.5) nsd++;
    scr[i] = rl[key[i]];
  }
  for (int i = 0; i < k; i++) {
    rl[i] = scr[i];
    scr[i] = aim[key[i]];
  }
  for (int i = 0; i < k; i++) aim[i] = scr[i];
  // Now set up splines (8 cases - four species, each double and imaginary)
  // We have to allow for the case when there are no constituent amplitudes
  // for the long-period tides.
  double zdr[NCON] = {0}, zdi[NCON] = {0};
  if (nlp != 0) {
    cubic_spline(nlp, rf, rl, zdr, scr);
    cubic_spline(nlp, rf, aim, zdi, scr);
  }
  double dr[NCON] = {0}, di[NCON] = {0}, sdr[NCON] = {0}, sdi[NCON] = {0};
  cubic_spline(ndi, &rf[nlp], &rl[nlp], dr, scr);
  cubic_spline(ndi, &rf[nlp], &aim[nlp], di, scr);
  cubic_spline(nsd, &rf[nlp + ndi], &rl[nlp + ndi], sdr, scr);
  cubic_spline(nsd, &rf[nlp + ndi], &aim[nlp + ndi], sdi, scr);
  // Evaluate all harmonics using the interpolated admittance.
  int nout = 0;
  for (int i = 0; i < NT; i++) {
    if (idd[i][0] == 0 && nlp == 0) continue;
    tdfrph(time, idd[i], &f[nout], &p[nout]);
    int c = idd[i][0];
    // Compute phase corrections to equilibrium tide using function eval().
    double sf = f[nout], re, am;
    if (c == 0) {
      p[nout] += 180;
      re = cubic_spline_eval(sf, nlp, rf, rl, zdr);
      am = cubic_spline_eval(sf, nlp, rf, aim, zdi);
    } else if (c == 1) {
      p[nout] += 90;
      re = cubic_spline_eval(sf, ndi, &rf[nlp], &rl[nlp], dr);
      am = cubic_spline_eval(sf, ndi, &rf[nlp], &aim[nlp], di);
    } else if (c == 2) {
      re = cubic_spline_eval(sf, nsd, &rf[nlp + ndi], &rl[nlp + ndi], sdr);
      am = cubic_spline_eval(sf, nsd, &rf[nlp + ndi], &aim[nlp + ndi], sdi);
    } else {
      trace(1, "admint: unexpected state c=%d\n", c);
      continue;
    }
    amp[nout] = tamp[i] * sqrt(re * re + am * am);
    p[nout] += atan2(am, re) * R2D;
    if (p[nout] > 180) p[nout] += -360;
    nout++;
  }
  return nout;
}
// The purpose of the subroutine is to perform sine and cosine recursion to
// fill in data x, of length n, for nf sines and cosines with frequencies om.
static void recurs(double *x, int n, const double *hc, int nf, const double *om, double *scr) {
  for (int i = 0; i < nf; i++) {
    scr[i * 3] = hc[i * 2];
    scr[i * 3 + 1] = hc[i * 2] * cos(om[i]) - hc[i * 2 + 1] * sin(om[i]);
    scr[i * 3 + 2] = cos(om[i]) * 2;
  }
  // Do recursion over data.
  for (int i = 0; i < n; i++) {
    x[i] = 0;
    // Then do recursive computation for each harmonic.
    for (int j = 0; j < nf; j++) {
      double sc = scr[j * 3];
      x[i] += sc;
      scr[j * 3] = scr[j * 3 + 2] * sc - scr[j * 3 + 1];
      scr[j * 3 + 1] = sc;
    }
  }
}
// This function applies station displacements read in the BLQ format used by
// Scherneck and Bos for ocean loading, and outputs a time series of computed
// tidal displacements, using an expanded set of tidal constituents, whose
// amplitudes and phases are found by spline interpolation of the tidal
// admittance.  A total of 342 constituent tides are included, which gives a
// precision of about 0.1%.
//
// time - the epoch, in GPST.
// samp - the sampling interval in seconds.
// irnt - the number of samples to return.
// tamp[NTIN][3], tph[NTIN][3] - amplitudes and phases, in standard "Scherneck" form.
// dz[irnt], ds[irnt], dw[irnt] - the offsets, up, south, and west respectively.
static void hardisp(const gtime_t time, double samp, int irnt, const double tamp[NTIN][3],
                    const double tph[NTIN][3], double *dz, double *ds, double *dw) {
  static const int idt[NTIN][6] = {{2, 0, 0, 0, 0, 0},  {2, 2, -2, 0, 0, 0}, {2, -1, 0, 1, 0, 0},
                                   {2, 2, 0, 0, 0, 0},  {1, 1, 0, 0, 0, 0},  {1, -1, 0, 0, 0, 0},
                                   {1, 1, -2, 0, 0, 0}, {1, -2, 0, 1, 0, 0}, {0, 2, 0, 0, 0, 0},
                                   {0, 1, 0, -1, 0, 0}, {0, 0, 2, 0, 0, 0}};
  // Find amplitudes and phases for all constituents, for each of the three
  // displacements. Note that the same frequencies are returned each time.
  //
  // BLQ format order is vertical, horizontal EW, horizontal NS.
  double amp[NTIN], phase[NTIN];
  for (int i = 0; i < NTIN; i++) {
    amp[i] = tamp[i][0];
    phase[i] = tph[i][0];
  }
  double az[NT], f[NT], pz[NT];
  admint(time, amp, idt, phase, az, f, pz, NTIN);

  for (int i = 0; i < NTIN; i++) {
    amp[i] = tamp[i][1];
    phase[i] = tph[i][1];
  }
  double aw[NT], pw[NT];
  admint(time, amp, idt, phase, aw, f, pw, NTIN);

  for (int i = 0; i < NTIN; i++) {
    amp[i] = tamp[i][2];
    phase[i] = tph[i][2];
  }
  double as[NT], ps[NT];
  int ntout = admint(time, amp, idt, phase, as, f, ps, NTIN);
  // Set up for recursion, by normalizing frequencies, and converting phases
  // to radians.
  double wf[NT];
  for (int i = 0; i < ntout; i++) {
    pz[i] = D2R * pz[i];
    ps[i] = D2R * ps[i];
    pw[i] = D2R * pw[i];
    f[i] = samp * PI * f[i] / 43200;
    wf[i] = f[i];
  }
  // Loop over times, NL output points at a time. At the start of each such
  // block, convert from amp and phase to sin and cos (hc array) at the start
  // of the block. The computation of values within each block is done
  // recursively, since the times are equi-spaced.
  int irli = 1;
  for (int j = 0;; j++) {
    int irhi = irli + NL - 1;
    if (irhi > irnt) irhi = irnt;
    int np = irhi - irli + 1;
    // Set up harmonic coefficients, compute tide.
    double hcz[NT * 2], hcs[NT * 2], hcw[NT * 2];
    for (int i = 0; i < NT; i++) {
      hcz[i * 2] = az[i] * cos(pz[i]);
      hcz[i * 2 + 1] = -az[i] * sin(pz[i]);
      hcs[i * 2] = as[i] * cos(ps[i]);
      hcs[i * 2 + 1] = -as[i] * sin(ps[i]);
      hcw[i * 2] = aw[i] * cos(pw[i]);
      hcw[i * 2 + 1] = -aw[i] * sin(pw[i]);
    }
    double scr[NT * 3];  // Scratch.
    recurs(dz + j * NL, np, hcz, ntout, wf, scr);
    recurs(ds + j * NL, np, hcs, ntout, wf, scr);
    recurs(dw + j * NL, np, hcw, ntout, wf, scr);
    if (irhi >= irnt) break;
    irli = irhi + 1;
    // Reset phases to the start of the new section.
    for (int i = 0; i < NT; i++) {
      pz[i] = fmod(pz[i] + np * f[i], 2 * PI);
      ps[i] = fmod(ps[i] + np * f[i], 2 * PI);
      pw[i] = fmod(pw[i] + np * f[i], 2 * PI);
    }
  }
}


// IERS mean pole -------------------------------------------------------------
// Ref: https://iers-conventions.obspm.fr/conventions_material.php TN.36
// 7.1.4 Rotational deformation due to polar motion: Secular polar motion and the pole tide.
// Eq. 21.
static void iers_secular_pole(gtime_t tutc, double *xp_bar, double *yp_bar) {
  // Need 365.25 day years since J2000.0. Midday less TT to GPST (19.0 + 32.184) is:
  const double ep2000[] = {2000, 1, 1, 11, 59, 08.816};  // GPST of J2000.0
  gtime_t tgps = utc2gpst(tutc);
  double y = timediff(tgps, epoch2time(ep2000)) / 86400.0 / 365.25;
  *xp_bar = 55.0 + 1.677 * y;
  *yp_bar = 320.5 + 3.460 * y;
}
// Displacement by pole tide (ref [7] eq.7.26) --------------------------------
static void tide_pole(gtime_t tutc, const double *pos, const double *erpv, double *denu) {
  trace(3, "tide_pole: pos=%.3f %.3f\n", pos[0] * R2D, pos[1] * R2D);

  // IERS secular pole (mas)
  double xp_bar, yp_bar;
  iers_secular_pole(tutc, &xp_bar, &yp_bar);

  // Ref [7] eq.7.24
  double m1 = erpv[0] / AS2R - xp_bar * 1E-3;  // (as)
  double m2 = -erpv[1] / AS2R + yp_bar * 1E-3;

  // sin(2*theta) = sin(2*phi), cos(2*theta)=-cos(2*phi)
  double cosl = cos(pos[1]);
  double sinl = sin(pos[1]);
  denu[0] = 9E-3 * sin(pos[0]) * (m1 * sinl - m2 * cosl);          // de= Slambda (m)
  denu[1] = -9E-3 * cos(2.0 * pos[0]) * (m1 * cosl + m2 * sinl);   // dn=-Stheta  (m)
  denu[2] = -33E-3 * sin(2.0 * pos[0]) * (m1 * cosl + m2 * sinl);  // du= Sr      (m)

  trace(5, "tide_pole : denu=%.3f %.3f %.3f\n", denu[0], denu[1], denu[2]);
}

/* Tidal displacement ----------------------------------------------------------
* Displacements by earth tides
* Args   : gtime_t tutc     I   time in UTC
*          double *rr       I   site position (ECEF) (m)
*          int    opt       I   options (or of the followings)
*                                 1: solid earth tide
*                                 2: ocean tide loading
*                                 4: pole tide
*                                 8: eliminate permanent deformation
*          double *erp      I   earth rotation parameters (NULL: not used)
*          double *odisp    I   ocean loading parameters  (NULL: not used)
*                                 odisp[0][i][0]: constituent i amplitude radial(m)
*                                 odisp[0][i][1]: constituent i amplitude west  (m)
*                                 odisp[0][i][2]: constituent i amplitude south (m)
*                                 odisp[1][i][0]: constituent i phase radial  (deg)
*                                 odisp[1][i][1]: constituent i phase west    (deg)
*                                 odisp[1][i][2]: constituent i phase south   (deg)
*                                (i=0:M2,1:S2,2:N2,3:K2,4:K1,5:O1,6:P1,7:Q1,
*                                   8:Mf,9:Mm,10:Ssa)
*          double *dr       O   displacement by earth tides (ECEF) (m)
* Return : none
* Notes  : see ref [1], [2] chap 7
*          see ref [4] 5.2.1, 5.2.2, 5.2.3
*-----------------------------------------------------------------------------*/
extern void tidedisp(gtime_t tutc, const double *rr, int opt, const erp_t *erp,
                     const double odisp[2][11][3], double *dr) {
  char tstr[40];
  trace(3, "tidedisp: tutc=%s\n", time2str(tutc, tstr, 0));

  double erpv[5] = {0};
  if (erp) geterp(erp, utc2gpst(tutc), erpv);

  dr[0] = dr[1] = dr[2] = 0.0;

  if (norm(rr, 3) <= 0.0) return;

  if (opt & 1) {  // Solid earth tides.
    // Sun and moon position in ECEF.
    double rs[3], rm[3];
    sunmoonpos(tutc, erpv, rs, rm, NULL);
    double drt[3];
    dehanttideinel(tutc, (double *)rr, rs, rm, drt);
    for (int i = 0; i < 3; i++) dr[i] += drt[i];
    trace(5, "tidedisp solid: dr=%.3f %.3f %.3f\n", drt[0], drt[1], drt[2]);
  }

  double pos[3], E[9];
  if (((opt & 2) && odisp) || ((opt & 4) && erp)) {
    ecef2pos(rr, pos);
    xyz2enu(pos, E);
  }
  if ((opt & 2) && odisp) {  // Ocean tide loading.
    gtime_t tut = timeadd(tutc, erpv[2]);
    double dz, ds, dw;
    hardisp(tut, 1, 1, odisp[0], odisp[1], &dz, &ds, &dw);
    double denu[3];
    denu[0] = -dw;
    denu[1] = -ds;
    denu[2] =  dz;
    double drt[3];
    matmul("TN", 3, 1, 3, E, denu, drt);
    for (int i = 0; i < 3; i++) dr[i] += drt[i];
    trace(5, "tidedisp otide: dr=%.3f %.3f %.3f\n", drt[0], drt[1], drt[2]);
  }
  if ((opt & 4) && erp) {  // Pole tide.
    double denu[3];
    tide_pole(tutc, pos, erpv, denu);
    double drt[3];
    matmul("TN", 3, 1, 3, E, denu, drt);
    for (int i = 0; i < 3; i++) dr[i] += drt[i];
    trace(5, "tidedisp spole: dr=%.3f %.3f %.3f\n", drt[0], drt[1], drt[2]);
  }
  trace(5, "tidedisp: dr=%.3f %.3f %.3f\n", dr[0], dr[1], dr[2]);
}
