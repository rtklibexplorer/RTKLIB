      PROGRAM HARDISP
*+
*  - - - - - - - - - - -
*   H A R D I S P
*  - - - - - - - - - - -
*
*  This program is part of the International Earth Rotation and
*  Reference Systems Service (IERS) Conventions software collection.
*
*  This program reads in a file of station displacements in the BLQ
*  format used by Scherneck and Bos for ocean loading, and outputs a
*  time series of computed tidal displacements, using an expanded set
*  of tidal constituents, whose amplitudes and phases are found by
*  spline interpolation of the tidal admittance.  A total of 342
*  constituent tides are included, which gives a precision of about
*  0.1%.
*
*  In general, Class 1, 2, and 3 models represent physical effects that
*  act on geodetic parameters while canonical models provide lower-level
*  representations or basic computations that are used by Class 1, 2, or
*  3 models.
* 
*  Status:  Class 1 model
*
*     Class 1 models are those recommended to be used a priori in the
*     reduction of raw space geodetic data in order to determine
*     geodetic parameter estimates.
*     Class 2 models are those that eliminate an observational
*     singularity and are purely conventional in nature.
*     Class 3 models are those that are not required as either Class
*     1 or 2.
*     Canonical models are accepted as is and cannot be classified as
*     a Class 1, 2, or 3 model.
*
*  Given:
*     User provided input ocean loading coefficients (Note 1)
*
*  Returned:
*     DU         d      Radial tidal ocean loading displacement (Note 2)
*     DW         d      West tidal ocean loading displacement (Note 2)
*     DS         d      South tidal ocean loading displacement (Note 2)
*
*     :------------------------------------------:
*     :                                          :
*     :                 IMPORTANT                :
*     :                                          :
*     :  A new version of the ETUTC routine must :
*     :  be produced whenever a new leap second  :
*     :  is announced.  There are three items to :
*     :  change on each such occasion:           :
*     :                                          :
*     :  1) Update the nstep variable            :
*     :  2) Update the arrays st and si          :                              
*     :  3) Change date of latest leap second    :
*     :                                          :
*     :  Latest leap second:  2016 December 31   :
*     :                                          :
*     :__________________________________________:
*
*
*  Notes:
*
*  1) The input ocean loading coefficients were generated by the ocean loading
*     service on 25 June 2009 using http://www.oso.chalmers.se/~loading/ for
*     IGS stations Onsala and Reykjavik using the CSR4.0 model and "NO"
*     geocenter correction.
*
*  2) The site displacement output is written to standard output with the 
*     format 3F14.6.  All units are expressed in meters.
*
*  Called:
*     MDAY              Finds the day number of days before start of a
*                       month and year in Gregorian intercalation
*     ADMINT            Returns the ocean loading displacement amplitude,
*                       frequency, and phase of a set of tidal constituents
*     RECURS            Performs sine and cosine recursion
*
*  Test cases:
*     given input:      The six lines of coefficients listed below entered 
*                       from standard input (e.g. epoch 2009 06 25 01h
*                       10m 45s and for each of the following 23 hours)
*                       with command 'HARDISP 2009 6 25 1 10 45 24 3600 < file'
*
*  ONSALA    
*$$ CSR4.0_f_PP ID: 2009-06-25 17:43:24
*$$ Computed by OLMPP by H G Scherneck, Onsala Space Observatory, 2009
*$$ Onsala,                              lon/lat:   11.9264   57.3958    0.00
*  .00352 .00123 .00080 .00032 .00187 .00112 .00063 .00003 .00082 .00044 .00037
*  .00144 .00035 .00035 .00008 .00053 .00049 .00018 .00009 .00012 .00005 .00006
*  .00086 .00023 .00023 .00006 .00029 .00028 .00010 .00007 .00004 .00002 .00001
*   -64.7  -52.0  -96.2  -55.2  -58.8 -151.4  -65.6 -138.1    8.4    5.2    2.1
*    85.5  114.5   56.5  113.6   99.4   19.1   94.1  -10.4 -167.4 -170.0 -177.7
*   109.5  147.0   92.7  148.8   50.5  -55.1   36.4 -170.4  -15.0    2.3    5.2
*
*  REYKJAVIK 
*$$ CSR4.0_f_PP ID: 2009-06-25 20:02:03
*$$ Computed by OLMPP by H G Scherneck, Onsala Space Observatory, 2009
*$$ Reykjavik,                           lon/lat:   64.1388  -21.9555    0.00
*  .02359 .01481 .00448 .00419 .00273 .00033 .00088 .00005 .00081 .00034 .00034
*  .00514 .00280 .00089 .00078 .00106 .00074 .00035 .00018 .00004 .00001 .00003
*  .00209 .00077 .00051 .00021 .00151 .00066 .00047 .00019 .00014 .00008 .00006
*    78.5  102.3   76.3  104.1  -52.0 -160.4  -52.6 -128.0 -174.8 -175.7 -178.1
*    54.2   93.8   38.9   96.9    2.3  -12.5    3.1  -31.6 -144.4 -122.6 -173.5
*   156.2 -167.1  141.9 -164.9  155.9  178.7  155.6 -168.5  177.7  178.7  179.7
*
*     expected output:
*        Onsala:
*         dU            dS            dW
*      0.003094     -0.001538     -0.000895
*      0.001812     -0.000950     -0.000193
*      0.000218     -0.000248      0.000421
*     -0.001104      0.000404      0.000741
*     -0.001668      0.000863      0.000646
*     -0.001209      0.001042      0.000137
*      0.000235      0.000926     -0.000667
*      0.002337      0.000580     -0.001555
*      0.004554      0.000125     -0.002278
*      0.006271     -0.000291     -0.002615
*      0.006955     -0.000537     -0.002430
*      0.006299     -0.000526     -0.001706
*      0.004305     -0.000244     -0.000559
*      0.001294      0.000245      0.000793
*     -0.002163      0.000819      0.002075
*     -0.005375      0.001326      0.003024
*     -0.007695      0.001622      0.003448
*     -0.008669      0.001610      0.003272
*     -0.008143      0.001262      0.002557
*     -0.006290      0.000633      0.001477
*     -0.003566     -0.000155      0.000282
*     -0.000593     -0.000941     -0.000766
*      0.001992     -0.001561     -0.001457
*      0.003689     -0.001889     -0.001680
*
*        Reykjavik:
*         dU            dS            dW
*     -0.005940     -0.001245     -0.000278
*      0.013516     -0.001086      0.003212
*      0.029599     -0.000353      0.005483
*      0.038468      0.000699      0.005997
*      0.038098      0.001721      0.004690
*      0.028780      0.002363      0.001974
*      0.013016      0.002371     -0.001369
*     -0.005124      0.001653     -0.004390
*     -0.021047      0.000310     -0.006225
*     -0.030799     -0.001383     -0.006313
*     -0.032056     -0.003048     -0.004549
*     -0.024698     -0.004288     -0.001314
*     -0.010814     -0.004794      0.002623
*      0.005849     -0.004416      0.006291
*      0.020857     -0.003208      0.008766
*      0.030226     -0.001413      0.009402
*      0.031437      0.000594      0.007996
*      0.024079      0.002389      0.004844
*      0.009945      0.003606      0.000663
*     -0.007426      0.004022     -0.003581
*     -0.023652      0.003601     -0.006911
*     -0.034618      0.002505     -0.008585
*     -0.037515      0.001044     -0.008270
*     -0.031544     -0.000402     -0.006125
*           
*  References:
*
*     Petit, G. and Luzum, B. (eds.), IERS Conventions (2010),
*     IERS Technical Note No. 36, BKG (2010)
*
*  Revisions:  
*  2005 August  Duncan Agnew     Original code, which is based on the 
*                                program hartid distributed with the 
*                                SPOTL loading package
*  2005 November D. Agnew        Corrected error in reading displacements
*  2005 December D. Agnew        Hartmann-Wenzel harmonics in admint
*                                subroutine
*  2007 December 17 G. Petit     Corrected 'intial' to 'initial' 
*                                (noted by T. Springer)
*  2008 June     D. Agnew        Corrected long-period tides, modernized
*                                control flow, added explicit typing, increased
*                                number of harmonics and added one decimal to
*                                their amplitudes
*  2009 February 16 G. Petit     Updated etutc subroutine for 2009.0 leap
*                                second
*  2009 June     25 B. Stetzler  Initial standardization of code
*  2009 June     26 B. Stetzler  Provided two test cases
*  2009 July     02 B. Stetzler  Capitalization for backwards compatibility
*  2009 August   19 B. Stetzler  Updated test cases
*  2012 March    13 B. Stetzler  Updated etutc subroutine for 2012.5 leap
*                                second
*  2015 April    29 M.A. Davis   Updated etutc subroutine for 2015.5 leap
*                                second
*  2015 May      21 M.A. Davis   Changed 'delta' and 'year' in etutc 
*                                subroutine from real to double
*                                precision. Modified test case output.
*  2015 June     09 M.A. Davis   Modified hardisp test case to test 
*                                non-zero hhmmss.
*  2016 December 19 M.A. Davis   Updated etutc subroutine for 2017.0
*                                leap second
*-----------------------------------------------------------------------

      IMPLICIT NONE
      INTEGER I,IDAY,IDT,IMONTH,IRNT,IRHI,IRLI,IT,LUO,NB,NL,NP,NT,NTIN,
     .        KK,NTOUT,MDAY
*+---------------------------------------------------------------------
*
*  Parameters below set the buffer size for computing the tides
*  recursively (nl), the number of harmonics used in the prediction
*  (nt; this must also be set in the subroutine admint) and the number
*  of harmonics read in (ntin)
*
*----------------------------------------------------------------------
      PARAMETER (NL=600)
      PARAMETER (NT=342)
      PARAMETER (NTIN=11)

      CHARACTER*40 DUMM
      REAL AMP,AS,AW,AZ,DS,DW,DZ,HCS,HCW,HCZ,PHASE,TAMP,TPH,WF,SAMP
      REAL IARGC
      DOUBLE PRECISION F,PZ,PS,PW,SCR
      DOUBLE PRECISION DR,PI

      DIMENSION TAMP(3,NTIN),TPH(3,NTIN)
      DIMENSION IDT(6,NTIN),AMP(NTIN),PHASE(NTIN)
      DIMENSION AZ(NT),PZ(NT),HCZ(2*NT)
      DIMENSION AS(NT),PS(NT),HCS(2*NT)
      DIMENSION AW(NT),PW(NT),HCW(2*NT)
      DIMENSION DZ(NL),DS(NL),DW(NL)
      DIMENSION F(NT),SCR(3*NT),WF(NT)
      COMMON/DATE/IT(5)
      PARAMETER ( PI = 3.1415926535897932384626433D0 ) 
      DATA DR/0.01745329252D0/,IRLI/1/
      DATA LUO/6/

*  Cartwright-Tayler numbers of tides used in Scherneck lists:
*      M2, S2, N2, K2, K1, O1, P1, Q1, Mf, Mm, Ssa

      DATA IDT/
     .  2, 0, 0, 0, 0, 0,   2, 2,-2, 0, 0, 0,   2,-1, 0, 1, 0, 0,
     .  2, 2, 0, 0, 0, 0,   1, 1, 0, 0, 0, 0,   1,-1, 0, 0, 0, 0,
     .  1, 1,-2, 0, 0, 0,   1,-2, 0, 1, 0, 0,   0, 2, 0, 0, 0, 0,
     .  0, 1, 0,-1, 0, 0,   0, 0, 2, 0, 0, 0/
*+----------------------------------------------------------------------
*
*  Check number of arguments from command line, then read them in
*
*-----------------------------------------------------------------------

      IF(IARGC().LT.7.OR.IARGC().GT.8) THEN
         WRITE(LUO,100)
 100     FORMAT(/,'Usage:',/,
     .  '   hardisp yr [d-of-yr | month day] hr min sec num samp',//,
     .  ' Where ',/,
     .  '   the UTC date given is the time of the first term output',/,
     .  '   num is the number of output epochs to be written out',/,
     .  '   samp is the sample interval (seconds)',//,
     .  '  The harmonics file (amp and phase of displacement) is ',/,
     .  '    read from standard input in the BLQ format used by  ',/,
     .  '    Scherneck and Bos                                   ',//,
     .  '  Results are written to standard output (units = m):',/,
     .  '      dU    dS    dW   ',/,
     .  '    using format: 3F14.6 ',/)
         STOP
      ENDIF

      CALL GETARG(1,DUMM)
      READ(DUMM,102) IT(1)
 102  FORMAT(I4)
      IF(IARGC().EQ.7) THEN
        CALL GETARG(2,DUMM)
        READ(DUMM,102) IT(2)
        NB=0
      ENDIF
      IF(IARGC().EQ.8) THEN
        CALL GETARG(2,DUMM)
        READ(DUMM,102) IMONTH
        CALL GETARG(3,DUMM)
        READ(DUMM,102) IDAY
        NB=1
        IT(2) = IDAY + MDAY(IT(1),IMONTH)
      ENDIF
      CALL GETARG(NB+3,DUMM)
      READ(DUMM,102) IT(3)
      CALL GETARG(NB+4,DUMM)
      READ(DUMM,102) IT(4)
      CALL GETARG(NB+5,DUMM)
      READ(DUMM,102) IT(5)
      CALL GETARG(NB+6,DUMM)
      READ(DUMM,104) IRNT
 104  FORMAT(I6)
      CALL GETARG(NB+7,DUMM)
      READ(DUMM,106) SAMP
 106  FORMAT(F7.0)

*+---------------------------------------------------------------------
*  Read in amplitudes and phases, in standard "Scherneck" form, from
*  standard input
*----------------------------------------------------------------------
      DO I=1,3
        READ(5,108) (TAMP(I,KK),KK=1,NTIN)
 108    FORMAT(1X,11F7.5)
      ENDDO
      DO I=1,3
        READ(5,110) (TPH(I,KK),KK=1,NTIN)
 110    FORMAT(1X,11F7.1)

* Change sign for phase, to be negative for lags

        DO KK=1,NTIN
          TPH(I,KK)=-TPH(I,KK)
        ENDDO
      ENDDO
*+---------------------------------------------------------------------
*
*  Find amplitudes and phases for all constituents, for each of the
*  three displacements. Note that the same frequencies are returned 
*  each time.
*
*  BLQ format order is vertical, horizontal EW, horizontal NS
*
*----------------------------------------------------------------------
      DO I=1,NTIN
        AMP(I)=TAMP(1,I)
        PHASE(I)=TPH(1,I)
      ENDDO
      CALL ADMINT(AMP,IDT,PHASE,AZ,F,PZ,NTIN,NTOUT)
      DO I=1,NTIN
        AMP(I)=TAMP(2,I)
        PHASE(I)=TPH(2,I)
      ENDDO
      CALL ADMINT(AMP,IDT,PHASE,AW,F,PW,NTIN,NTOUT)
      DO I=1,NTIN
        AMP(I)=TAMP(3,I)
        PHASE(I)=TPH(3,I)
      ENDDO
      CALL ADMINT(AMP,IDT,PHASE,AS,F,PS,NTIN,NTOUT)

*  set up for recursion, by normalizing frequencies, and converting
*  phases to radians

      DO I=1,NTOUT
        PZ(I) = DR*PZ(I)
        PS(I) = DR*PS(I)
        PW(I) = DR*PW(I)
        F(I) = SAMP*PI*F(I)/43200.D0
        WF(I) = F(I)
      ENDDO

*+---------------------------------------------------------------------
*
*  Loop over times, nl output points at a time. At the start of each
*  such block, convert from amp and phase to sin and cos (hc array) at
*  the start of the block. The computation of values within each
*  block is done recursively, since the times are equi-spaced.
*
*----------------------------------------------------------------------

 11   IRHI = MIN(IRLI+NL-1,IRNT)
      NP = IRHI - IRLI + 1

* Set up harmonic coefficients, compute tide, and write out
      DO I=1,NT
        HCZ(2*I-1) = AZ(I)*DCOS(PZ(I))
        HCZ(2*I)  = -AZ(I)*DSIN(PZ(I))
        HCS(2*I-1) = AS(I)*DCOS(PS(I))
        HCS(2*I)  = -AS(I)*DSIN(PS(I))
        HCW(2*I-1) = AW(I)*DCOS(PW(I))
        HCW(2*I)  = -AW(I)*DSIN(PW(I))
      ENDDO
      CALL RECURS(DZ,NP,HCZ,NTOUT,WF,SCR)
      CALL RECURS(DS,NP,HCS,NTOUT,WF,SCR)
      CALL RECURS(DW,NP,HCW,NTOUT,WF,SCR)
      WRITE(LUO,120) (DZ(I),DS(I),DW(I),I=1,NP)
 120  FORMAT(3F14.6)
      IF(IRHI.EQ.IRNT) STOP
      IRLI = IRHI + 1

*  Reset phases to the start of the new section
      DO I=1,NT
        PZ(I) = DMOD(PZ(I) + NP*F(I),2.D0*PI)
        PS(I) = DMOD(PS(I) + NP*F(I),2.D0*PI)
        PW(I) = DMOD(PW(I) + NP*F(I),2.D0*PI)
      ENDDO
      GO TO 11

*  Finished.

*+----------------------------------------------------------------------
*
*  Copyright (C) 2008
*  IERS Conventions Center
*
*  ==================================
*  IERS Conventions Software License
*  ==================================
*
*  NOTICE TO USER:
*
*  BY USING THIS SOFTWARE YOU ACCEPT THE FOLLOWING TERMS AND CONDITIONS
*  WHICH APPLY TO ITS USE.
*
*  1. The Software is provided by the IERS Conventions Center ("the
*     Center").
*
*  2. Permission is granted to anyone to use the Software for any
*     purpose, including commercial applications, free of charge,
*     subject to the conditions and restrictions listed below.
*
*  3. You (the user) may adapt the Software and its algorithms for your
*     own purposes and you may distribute the resulting "derived work"
*     to others, provided that the derived work complies with the
*     following requirements:
*
*     a) Your work shall be clearly identified so that it cannot be
*        mistaken for IERS Conventions software and that it has been
*        neither distributed by nor endorsed by the Center.
*
*     b) Your work (including source code) must contain descriptions of
*        how the derived work is based upon and/or differs from the
*        original Software.
*
*     c) The name(s) of all modified routine(s) that you distribute
*        shall be changed.
* 
*     d) The origin of the IERS Conventions components of your derived
*        work must not be misrepresented; you must not claim that you
*        wrote the original Software.
*
*     e) The source code must be included for all routine(s) that you
*        distribute.  This notice must be reproduced intact in any
*        source distribution. 
*
*  4. In any published work produced by the user and which includes
*     results achieved by using the Software, you shall acknowledge
*     that the Software was used in obtaining those results.
*
*  5. The Software is provided to the user "as is" and the Center makes
*     no warranty as to its use or performance.   The Center does not
*     and cannot warrant the performance or results which the user may
*     obtain by using the Software.  The Center makes no warranties,
*     express or implied, as to non-infringement of third party rights,
*     merchantability, or fitness for any particular purpose.  In no
*     event will the Center be liable to the user for any consequential,
*     incidental, or special damages, including any lost profits or lost
*     savings, even if a Center representative has been advised of such
*     damages, or for any claim by any third party.
*
*  Correspondence concerning IERS Conventions software should be
*  addressed as follows:
*
*                     Gerard Petit
*     Internet email: gpetit[at]bipm.org
*     Postal address: IERS Conventions Center
*                     Time, frequency and gravimetry section, BIPM
*                     Pavillon de Breteuil
*                     92312 Sevres  FRANCE
*
*     or
*
*                     Brian Luzum
*     Internet email: brian.luzum[at]usno.navy.mil
*     Postal address: IERS Conventions Center
*                     Earth Orientation Department
*                     3450 Massachusetts Ave, NW
*                     Washington, DC 20392
*
*
*-----------------------------------------------------------------------
      END
