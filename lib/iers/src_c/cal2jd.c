/* cal2jd.f -- translated by f2c (version 20200916).
   You must link the resulting object file with libf2c:
	on Microsoft Windows system, link with libf2c.lib;
	on Linux or Unix systems, link with .../path/to/libf2c.a -lm
	or, if you install libf2c.a in a standard place, with -lf2c -lm
	-- in that order, at the end of the command line, as in
		cc *.o -lf2c -lm
	Source for libf2c is in /netlib/f2c/libf2c.zip, e.g.,

		http://www.netlib.org/f2c/libf2c.zip
*/

#include "f2c.h"

/* Subroutine */ int iau_cal2jd__(integer *iy, integer *im, integer *id, 
	doublereal *djm0, doublereal *djm, integer *j)
{
    /* Initialized data */

    static integer mtab[12] = { 31,28,31,30,31,30,31,31,30,31,30,31 };

    integer my, ndays, iypmy;

/* + */
/*  - - - - - - - - - - - */
/*   i a u _ C A L 2 J D */
/*  - - - - - - - - - - - */

/*  Gregorian Calendar to Julian Date. */

/*  This routine is part of the International Astronomical Union's */
/*  SOFA (Standards of Fundamental Astronomy) software collection. */

/*  Status:  support routine. */

/*  Given: */
/*     IY,IM,ID    i     year, month, day in Gregorian calendar (Note 1) */

/*  Returned: */
/*     DJM0        d     MJD zero-point: always 2400000.5D0 */
/*     DJM         d     Modified Julian Date for 0 hrs */
/*     J           i     status: */
/*                           0 = OK */
/*                          -1 = bad year   (Note 3: JD not computed) */
/*                          -2 = bad month  (JD not computed) */
/*                          -3 = bad day    (JD computed) */

/*  Notes: */

/*  1) The algorithm used is valid from -4800 March 1, but this */
/*     implementation rejects dates before -4799 January 1. */

/*  2) The Julian Date is returned in two pieces, in the usual SOFA */
/*     manner, which is designed to preserve time resolution.  The */
/*     Julian Date is available as a single number by adding DJM0 and */
/*     DJM. */

/*  3) In early eras the conversion is from the "Proleptic Gregorian */
/*     Calendar";  no account is taken of the date(s) of adoption of */
/*     the Gregorian Calendar, nor is the AD/BC numbering convention */
/*     observed. */

/*  Reference: */

/*     Explanatory Supplement to the Astronomical Almanac, */
/*     P. Kenneth Seidelmann (ed), University Science Books (1992), */
/*     Section 12.92 (p604). */

/*  This revision:  2020 October 21 */

/*  SOFA release 2023-10-11 */

/*  Copyright (C) 2023 IAU SOFA Board.  See notes at end. */

/* ----------------------------------------------------------------------- */
/*  Earliest year allowed (4800BC) */
/*  Month lengths in days */
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/*  Preset status. */
    *j = 0;
/*  Validate year. */
    if (*iy < -4799) {
	*j = -1;
    } else {
/*     Validate month. */
	if (*im >= 1 && *im <= 12) {
/*        Days in current month. */
	    ndays = mtab[*im - 1];
/*        Allow for leap year. */
	    if (*im == 2) {
		if (*iy % 4 == 0) {
		    ndays = 29;
		}
		if (*iy % 100 == 0 && *iy % 400 != 0) {
		    ndays = 28;
		}
	    }
/*        Validate day. */
	    if (*id < 1 || *id > ndays) {
		*j = -3;
	    }
/*        Result. */
	    my = (*im - 14) / 12;
	    iypmy = *iy + my;
	    *djm0 = 2400000.5;
	    *djm = (doublereal) ((iypmy + 4800) * 1461 / 4 + (*im - 2 - my * 
		    12) * 367 / 12 - (iypmy + 4900) / 100 * 3 / 4 + *id - 
		    2432076);
/*        Bad month */
	} else {
	    *j = -2;
	}
    }
/*  Finished. */
/* +---------------------------------------------------------------------- */

/*  Copyright (C) 2023 */
/*  Standards of Fundamental Astronomy Board */
/*  of the International Astronomical Union. */

/*  ===================== */
/*  SOFA Software License */
/*  ===================== */

/*  NOTICE TO USER: */

/*  BY USING THIS SOFTWARE YOU ACCEPT THE FOLLOWING SIX TERMS AND */
/*  CONDITIONS WHICH APPLY TO ITS USE. */

/*  1. The Software is owned by the IAU SOFA Board ("SOFA"). */

/*  2. Permission is granted to anyone to use the SOFA software for any */
/*     purpose, including commercial applications, free of charge and */
/*     without payment of royalties, subject to the conditions and */
/*     restrictions listed below. */

/*  3. You (the user) may copy and distribute SOFA source code to others, */
/*     and use and adapt its code and algorithms in your own software, */
/*     on a world-wide, royalty-free basis.  That portion of your */
/*     distribution that does not consist of intact and unchanged copies */
/*     of SOFA source code files is a "derived work" that must comply */
/*     with the following requirements: */

/*     a) Your work shall be marked or carry a statement that it */
/*        (i) uses routines and computations derived by you from */
/*        software provided by SOFA under license to you; and */
/*        (ii) does not itself constitute software provided by and/or */
/*        endorsed by SOFA. */

/*     b) The source code of your derived work must contain descriptions */
/*        of how the derived work is based upon, contains and/or differs */
/*        from the original SOFA software. */

/*     c) The names of all routines in your derived work shall not */
/*        include the prefix "iau" or "sofa" or trivial modifications */
/*        thereof such as changes of case. */

/*     d) The origin of the SOFA components of your derived work must */
/*        not be misrepresented;  you must not claim that you wrote the */
/*        original software, nor file a patent application for SOFA */
/*        software or algorithms embedded in the SOFA software. */

/*     e) These requirements must be reproduced intact in any source */
/*        distribution and shall apply to anyone to whom you have */
/*        granted a further right to modify the source code of your */
/*        derived work. */

/*     Note that, as originally distributed, the SOFA software is */
/*     intended to be a definitive implementation of the IAU standards, */
/*     and consequently third-party modifications are discouraged.  All */
/*     variations, no matter how minor, must be explicitly marked as */
/*     such, as explained above. */

/*  4. You shall not cause the SOFA software to be brought into */
/*     disrepute, either by misuse, or use for inappropriate tasks, or */
/*     by inappropriate modification. */

/*  5. The SOFA software is provided "as is" and SOFA makes no warranty */
/*     as to its use or performance.   SOFA does not and cannot warrant */
/*     the performance or results which the user may obtain by using the */
/*     SOFA software.  SOFA makes no warranties, express or implied, as */
/*     to non-infringement of third party rights, merchantability, or */
/*     fitness for any particular purpose.  In no event will SOFA be */
/*     liable to the user for any consequential, incidental, or special */
/*     damages, including any lost profits or lost savings, even if a */
/*     SOFA representative has been advised of such damages, or for any */
/*     claim by any third party. */

/*  6. The provision of any version of the SOFA software under the terms */
/*     and conditions specified herein does not imply that future */
/*     versions will also be made available under the same terms and */
/*     conditions. */

/*  In any published work or commercial product which uses the SOFA */
/*  software directly, acknowledgement (see www.iausofa.org) is */
/*  appreciated. */

/*  Correspondence concerning SOFA software should be addressed as */
/*  follows: */

/*      By email:  sofa@ukho.gov.uk */
/*      By post:   IAU SOFA Center */
/*                 HM Nautical Almanac Office */
/*                 UK Hydrographic Office */
/*                 Admiralty Way, Taunton */
/*                 Somerset, TA1 2DN */
/*                 United Kingdom */

/* ----------------------------------------------------------------------- */
    return 0;
} /* iau_cal2jd__ */

