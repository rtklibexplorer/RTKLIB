/*------------------------------------------------------------------------------
* septentrio.c : Septentrio Binary Format decoder (All Septentrio receivers)
*
*          Copyright (C) 2013 by Fabrizio Tappero.
*          Copyright (C) 2015 by Jens Reimann
*
* reference :
*     [1] Septentrio, SBF Reference Guide, Version 130722r38600, 07/2013
*
* note: - IRN is deactivated. The code is not tested. Use -DTESTING to activate.
*
*
* version : $Revision: 1.4 $ $Date: 2016/01/29 15:05:00 $
*
* history : 2013/07/17  1.0  begin writing
*           2013/10/24  1.1  GPS L1 working
*           2013/11/02  1.2  modified by TTAKASU
*           2015/01/26  1.3  fix some problems by Jens Reimann
*           2016/02/04  1.4  by Jens Reimann
*                           - added more sanity checks
*                           - added galileon raw decoding
*                           - added usage of decoded SBAS messages for testing
*                           - add QZSS and Compass/Beidou navigation messages
*                           - fixed code and Doppler for 2nd and following frequency
*                           - fixed bug in glonass ephemeris
*                           - fixed decoding of galileo ephemeris
*                           - fixed lost lock indicator
*                           - fixed sbas decoding
*                           - cleanups
*           2016/03/03  1.5 - fixed TOW in SBAS messages
*           2016/03/12  1.6 - respect code priorities
*                           - fixed bug in carrier phase calculation of type2 data
*                           - unify frequency determination
*                           - improve lock handling
*                           - various bug fixes
*           2016/05/25  1.7  rtk_crc24q() -> crc24q() by T.T
*           2016/07/29  1.8  crc24q() -> rtk_crc24q() by T.T
*           2017/04/11  1.9  (char *) -> (signed char *) by T.T
*           2017/09/01  1.10 suppress warnings
*           2024/01/12  1.11 update with new code from Tomoji TAKASU
*           2024/06/16  1.12 restructed code, tested with Mosaic and PolarRx receivers
*           2024/06/26  1.13 implemented reading new Meas3 records
*-----------------------------------------------------------------------------*/
#include "rtklib.h"

#include <math.h>
#include <stdint.h>

extern const sbsigpband_t igpband1[][8]; /* SBAS IGP band 0-8 */
extern const sbsigpband_t igpband2[][5]; /* SBAS IGP band 9-10 */

#define MEAS3_SYS_MAX 7
#define MEAS3_SAT_MAX 64
#define MEAS3_SIG_MAX 16

static uint32_t const Meas3_EpochIntervals[] = {1, 500, 1000, 2000, 5000, 10000, 15000, 30000, 60000, 120000, 1, 1, 1, 1, 1, 1};  /* epoche interval index to epoche time in [ms] */
static uint8_t const Meas3_NavSys[] = {SYS_GPS, SYS_GLO, SYS_GAL, SYS_CMP, SYS_SBS, SYS_QZS, SYS_IRN};  /* meas3 navsys to rtklib navsys conversion */
static uint8_t const Meas3_SVIDBase[] = {MINPRNGPS, MINPRNGLO, MINPRNGAL, MINPRNCMP, MINPRNSBS, MINPRNQZS, MINPRNIRN}; /* rtklib satelite number start for rtklib navigation systems */
/* base pseudorange for the different constellations */
static const double PRBase[]  /* base distance for navigation systems in [m] */
    = { 19e6,    /* GPS        */
        19e6,    /* GLO        */
        22e6,    /* GAL        */
        20e6,    /* BDS  !!redefined to 34000km for GEO/IGSO    */
        34e6,    /* SBAS       */
        34e6,    /* QZS        */
        34e6     /* IRN        */
};
/* mapping of the Meas3 lock time indicator into actual lock time in milliseconds */
static const uint32_t Meas3_LTItoPLLTime[16] = { 0, 60000, 30000, 15000, 10000, 5000, 2000, 1000, 500, 200, 100, 50, 40, 20, 10, 0};

// Data from the last reference epoch when decoding Meas3 blocks.
typedef struct {
  uint32_t TOW;                                                    // Time-of-week in milliseconds
  obsd_t obsData[MEAS3_SYS_MAX][MEAS3_SAT_MAX];                    // Reference observation data
  uint8_t signalIdx[MEAS3_SYS_MAX][MEAS3_SAT_MAX][MEAS3_SIG_MAX];  // Reference signal indeces
  uint32_t slaveSignalMask[MEAS3_SYS_MAX][MEAS3_SAT_MAX];  // Mask of available slave signals
  int16_t prRate[MEAS3_SYS_MAX][MEAS3_SAT_MAX];  // Pseudo-range change rate in 64 mm steps
  uint32_t lockt[MEAS3_SYS_MAX][MEAS3_SAT_MAX][NFREQ + NEXOBS];  // Lock time of the PLL, in ms
  uint8_t halfc[MEAS3_SYS_MAX][MEAS3_SAT_MAX][NFREQ + NEXOBS];   // Half cycle ambiguity

  uint8_t constellationHeader[MEAS3_SYS_MAX][32];  // Copy of constellation header
} Meas3_RefEpoch_t;

typedef struct {
  gtime_t current_time;
  int meas2_channelAssignment[2048];
  Meas3_RefEpoch_t meas3_refEpoch;
  // Assignment for RTKLIB frequency indices to meas3 signal indices
  int8_t meas3_freqAssignment[MEAS3_SYS_MAX][MEAS3_SAT_MAX][MEAS3_SIG_MAX];
} sbf_t;

extern void free_sbf(raw_t *raw) {
  if (raw->format != STRFMT_SEPT) return;
  sbf_t *sbf = (sbf_t *)raw->rcv_data;
  if (sbf) {
    free(sbf);
    raw->rcv_data = NULL;
  }
}

extern int init_sbf(raw_t *raw) {
  if (raw->format != STRFMT_SEPT) return 0;
  sbf_t *sbf = (sbf_t *)calloc(1, sizeof(sbf_t));
  if (!sbf) {
    trace(0, "SBF: memory allocation error for SBF data\n");
    return 0;
  }
  raw->rcv_data = (void *)sbf;
  return 1;
}

/* SBF definitions  2020 */
#define SBF_SYNC1          0x24 /* SBF message header sync field 1 (correspond to $) */
#define SBF_SYNC2          0x40 /* SBF message header sync field 2 (correspont to @)*/
#define SBF_MAXSIG         39   /* SBF max signal number */

/* Measurement Blocks */

#define ID_GENMEASEPOCH    5944 /* SBF message id: Measurement set of one epoch */
#define ID_MEASEPOCH       4027 /* SBF message id: Measurement set of one epoch  */
#define ID_MEASEPOCHEXTRA  4000 /* SBF message id: Additional info such as observable variance */
#define ID_MEASFULLRANGE   4098 /* SBF message id: Extended-range code and phase measurements */
#define ID_MEASE3RNG       4109 /* SBF message id: Code, phase and CN0 measurements */
#define ID_MEASE3CN        4110 /* SBF message id: Extension of Meas3Ranges containing fractional C/N0 values */
#define ID_MEASE3DOPPLER   4111 /* SBF message id: Extension of Meas3Ranges containing Doppler values */
#define ID_MEASE3PP        4112 /* SBF message id: Extension of Meas3Ranges containing proprietary flags for data post-processing. (undocumented) */
#define ID_MEASE3MP        4113 /* SBF message id: Extension of Meas3Ranges containing multipath corrections applied by the receiver. (undocumented */
#define ID_IQCORR          4046 /* SBF message id: Real and imaginary post-correlation values */
#define ID_ISMR            4086 /* SBF message id: Ionospheric scintillation monitor (ISMR) data  */
#define ID_SQMSAMPLES      4087 /* SBF message id: Correlation samples for signal quality monitoring */
#define ID_MEASEPOCH_END   5922 /* SBF message id: Measurement epoch marker */

/* Navigation Page Blocks */
//#define ID_GPSRaw       5895    /* SBF message id: GPS CA navigation frame  */
//#define ID_CNAVRaw      5947    /* SBF message id: GPS L2C navigation frame */
//#define ID_GEORaw       5898    /* SBF message id: SBAS L1 navigation frame */
#define ID_GPSRAWCA     4017    /* SBF message id: GPS CA navigation subframe  */
#define ID_GPSRAWL2C    4018    /* SBF message id: GPS L2C navigation frame */
#define ID_GPSRAWL5     4019    /* SBF message id: GPS L5 navigation frame */
//#define ID_GPSRAWL1C    4221    /* SBF message id: GPS L1C navigation frame */
#define ID_GLORAWCA     4026    /* SBF message id: GLONASS CA navigation string */
#define ID_GALRAWFNAV   4022    /* SBF message id: Galileo F/NAV navigation page */
#define ID_GALRAWINAV   4023    /* SBF message id: Galileo I/NAV navigation page */
#define ID_GALRAWCNAV   4024    /* SBF message id: Galileo C/NAV navigation page */
//#define ID_GALRAWGNAV   4025    /* SBF message id: Galileo G/NAV navigation page */
//#define ID_GALRAWGNAVE  4029    /* SBF message id: Galileo G/NAVe navigation page */
#define ID_GEORAWL1     4020    /* SBF message id: SBAS L1 navigation message */
#define ID_GEORAWL5     4021    /* SBF message id: SBAS L5 navigation message */
#define ID_BDSRAW       4047    /* SBF message id: BeiDou navigation page */
#define ID_BDSRAWB1C    4218    /* SBF message id: BeiDou B1C navigation frame */
#define ID_BDSRAWB2A    4219    /* SBF message id: BeiDou B2A navigation frame */
#define ID_BDSRAWB2B    4242    /* SBF message id: BeiDou B2B navigation frame */
#define ID_QZSRAWL1CA   4066    /* SBF message id: QZSS L1C/A or L1C/B navigation frame */
#define ID_QZSRAWL2C    4067    /* SBF message id: QZSS L2C navigation frame  */
#define ID_QZSRAWL5     4068    /* SBF message id: QZSS L5 navigation frame */
#define ID_QZSSL6       4069    /* SBF message id: QZSS L6 navigation frame */
#define ID_QZSSL1C      4227    /* SBF message id: QZSS L1C navigation message */
#define ID_QZSSL1S      4228    /* SBF message id: QZSS L1S navigation message */
#define ID_QZSSL5S      4246    /* SBF message id: QZSS L5S navigation message */
#define ID_IRNSSRAW     4093    /* SBF message id: IRNSS raw navigation page or frame */
//#define ID_IRNSSRAWL1   4262    /* SBF message id: IRNSS raw navigation page or frame */
#define ID_GNSSNAVBITS  4088  /* SBF message id: Raw navigation bits during last second */
#define ID_GNSSSYMBOLS  4099  /* SBF message id: Raw navigation symbols */

/* GPS Decoded Message Blocks */
#define ID_GPSNAV       5891    /* SBF message id: GPS ephemeris and clock */
#define ID_GPSALM       5892    /* SBF message id: Almanac data for a GPS satellite */
#define ID_GPSION       5893    /* SBF message id: Ionosphere data from the GPS subframe 5 */
#define ID_GPSUTC       5894    /* SBF message id: GPS-UTC data from GPS subframe 5 */
#define ID_GPSCNAV      4042    /* SBF message id: CNAV Ephemeris data for one satellite.  */
#define ID_GPSCNAV2     4258    /* SBF message id: CNAV Ephemeris data for one satellite.  */

/* GLONASS Decoded Message Blocks */
#define ID_GLONAV       4004    /* SBF message id: GLONASS ephemeris and clock */
#define ID_GLOALM       4005    /* SBF message id: Almanac data for a GLONASS satellite  */
#define ID_GLOTIME      4036    /* SBF message id: GLO-UTC, GLO-GPS and GLO-UT1 data  */

/* Galileo Decoded Message Blocks */
#define ID_GALNAV       4002    /* SBF message id: Galileo ephemeris, clock, health and BGD */
#define ID_GALALM       4003    /* SBF message id: Almanac data for a Galileo satellite */
#define ID_GALION       4030    /* SBF message id: NeQuick Ionosphere model parameters*/
#define ID_GALUTC       4031    /* SBF message id: GST-UTC data */
#define ID_GALGSTGPS    4032    /* SBF message id: GST-GPS data */
#define ID_GALARRLM     4034    /* SBF message id: Search-and-rescue return link message */

/* BeiDou Decoded Message Blocks */
#define ID_BDSNAV       4081    /* SBF message id: BeiDou ephemeris and clock */
#define ID_BDSCNAV1     4251    /* SBF message id: BeiDou B-CNAV1 ephemeris data for one satellite.  */
#define ID_BDSCNAV2     4252    /* SBF message id: BeiDou B-CNAV2 ephemeris data for one satellite.  */
#define ID_BDSCNAV3     4253    /* SBF message id: BeiDou B-CNAV3 ephemeris data for one satellite.  */
#define ID_BDSALM       4119    /* SBF message id: Almanac data for a BeiDou satellite  */
#define ID_BDSION       4120    /* SBF message id: BeiDou Ionospheric delay model parameters */
#define ID_BDSUTC       4121    /* SBF message id: BDT-UTC data */

/* QZSS Decoded Message Blocks */
#define ID_QZSSNAV      4095    /* SBF message id: QZSS ephemeris and clock */
//#define ID_QZSSCNAV     4259    /* SBF message id: QZSS ephemeris and clock */
//#define ID_QZSSCNAV2    4260    /* SBF message id: QZSS ephemeris and clock */
#define ID_QZSSALM      4116    /* SBF message id: Almanac data for a QZSS satellite */

/* NavIC/IRNSS Decoded Message Blocks */
#define ID_NAVICLNAV    4254    /* SBF message id: NavIC/IRNSS ephemeris and clock  */

/* SBAS L1 Decoded Message Blocks */
#define ID_GEOMT00                  5925 /* SBF message id:  SBAS: Don't use for safety application */
#define ID_GEOPRNMASK               5926 /* SBF message id:  PRN Mask assignments */
#define ID_GEOFASTCORR              5927 /* SBF message id:  Fast Corrections */
#define ID_GEOINTEGRITY             5928 /* SBF message id:  Integrity information */
#define ID_GEOFASTCORRDEGR          5929 /* SBF message id:  Fast correction degradation factor */
#define ID_GEONAV                   5896 /* SBF message id:  SBAS navigation message */
#define ID_GEODEGRFACTORS           5930 /* SBF message id:  Degration factors */
#define ID_GEONETWORKTIME           5918 /* SBF message id:  SBAS Network Time/UTC offset parameters */
#define ID_GEOALM                   5897 /* SBF message id:  SBAS satellite almanac */
#define ID_GEOIGPMASK               5931 /* SBF message id:  Ionospheric grid point mask */
#define ID_GEOLONGTERMCOR           5932 /* SBF message id:  Long term satellite error corrections */
#define ID_GEOIONODELAY             5933 /* SBF message id:  Inospheric delay correction */
#define ID_GEOSERVICELEVEL          5917 /* SBF message id:  SBAS Service Message */
#define ID_GEOCLOCKEPHCOVMATRIX     5934 /* SBF message id:  Clock-Ephemeris Covariance Matrix l*/

/* SBAS L5 Decoded Message Blocks */
#define ID_SBASL5NAV    5958     /* SBF message id: DFMC SBAS ephemeris and clock data */
#define ID_SBASL5ALM    5959     /* SBF message id: DFMC SBAS almanac data */

/* GNSS Position, Velocity and Time Blocks */
#define ID_PVTCART1      5903    /* SBF message id: GNSS position, velocity, and time in Cartesian coordinates */
#define ID_PVTGEOD1      5904    /* SBF message id: GNSS position, velocity, and time in Geodetic coordinates */
#define ID_DOP1          5909    /* SBF message id: Dilution of precision */
#define ID_PVTRESIDUALS1 5910    /* SBF message id: Measurement residuals */
#define ID_RAIMSTATS1    5915    /* SBF message id: Integrity statistics */
#define ID_PVTCART2      4006    /* SBF message id: GNSS position, velocity, and time in Cartesian coordinates */
#define ID_PVTGEOD2      4007    /* SBF message id: GNSS position, velocity, and time in Geodetic coordinates */
#define ID_PVTGEODAUTH   4232    /* SBF message id: OSNMA-Authenticated Position, velocity, and time in geodetic coordinate */
#define ID_COVCART       5905    /* SBF message id: Position covariance matrix (X,Y, Z) */
#define ID_COVGEOD       5906    /* SBF message id: Position covariance matrix (Lat, Lon, Alt) */
#define ID_VELCOVCART    5907    /* SBF message id: Velocity covariance matrix (X, Y, Z)  */
#define ID_VELCOVGEOD    5908    /* SBF message id: Velocity covariance matrix (North, East, Up)  */
#define ID_DOP2          4001    /* SBF message id: Dilution of precision */
#define ID_DOPAUTH       4247    /* SBF message id: Dilution of Precision Authenticated Data */
#define ID_POSCART       4044    /* SBF message id: Position, variance and baseline in Cartesian coordinates */
#define ID_PVTLOCAL      4052    /* SBF message id: Position in a local datum */
#define ID_POSPROJ       4094    /* SBF message id: Plane grid coordinates */
#define ID_PVTSATCART    4008    /* SBF message id: Satellite positions */
#define ID_PVTRESIDUALS2 4009    /* SBF message id: Measurement residuals */
#define ID_RAIMSTATS2    4011    /* SBF message id: Integrity statistics */
#define ID_GEOCORR       5935    /* SBF message id: Orbit, Clock and pseudoranges SBAS corrections  */
#define ID_BASEVECCART   4043    /* SBF message id: XYZ relative position and velocity with respect to base(s) */
#define ID_BASEVECGEOD   4028    /* SBF message id: ENU relative position and velocity with respect to base(s)  */
#define ID_AMBIGUITIES   4240    /* SBF message id: Carrier phase ambiguity states */
#define ID_PVTSUPPORT    4076    /* SBF message id: Internal parameters for maintenance and support */
#define ID_PVTSUPPORTA   4079    /* SBF message id: Internal parameters for maintenance and support */
#define ID_PVTEND        5921    /* SBF message id: PVT epoch marke */
#define ID_BASELINE      5950    /* SBF message id:  Base-rover vector (deprecated block - not to be used) */

/* INS/GNSS Integrated Blocks */
#define ID_INTPVCART      4060   /* SBF message id: Integrated PV in Cartesian coordinates */
#define ID_INTPVGEOD      4061   /* SBF message id: Integrated PV in Geodetic coordinates */
#define ID_INTPOSCOVCART  4062   /* SBF message id: Integrated position covariance matrix (X, Y, Z) */
#define ID_INTVELCOVCART  4063   /* SBF message id: Integrated velocity covariance matrix (X, Y, Z) */
#define ID_INTPOSCOVGEOD  4064   /* SBF message id: Integrated position covariance matrix (Lat, Lon, Alt) */
#define ID_INTVELCOVGEOD  4065   /* SBF message id: Integrated velocity covariance matrix (North, East, Up) */
#define ID_INTATTEULER    4070   /* SBF message id: Integrated attitude in Euler angles */
#define ID_INTATTCCOEULER 4072   /* SBF message id: Integrated attitude covariance matrix of Euler angles */
#define ID_INTPVAAGEOD    4045   /* SBF message id: Integrated position, velocity, acceleration and attitude */
#define ID_INSNAVCART     4225   /* SBF message id: INS solution in Cartesian coordinates */
#define ID_INSNAVGEOD     4226   /* SBF message id: INS solution in Geodetic coordinates */
#define ID_IMUBIAS        4241   /* SBF message id: Estimated parameters of the IMU, such as the IMU biases and their standard deviation */

/* GNSS Attitude Blocks */
#define ID_ATTEULER     5938    /* SBF message id: GNSS attitude expressed as Euler angles */
#define ID_ATTCOVEULER  5939    /* SBF message id: Covariance matrix of attitude */
#define ID_AUXPOS       5942    /* SBF message id: Relative position and velocity estimates of auxiliary antennas  */
#define ID_ENDATT       5943    /* SBF message id: GNSS attitude epoch marker */
#define ID_ATTQUAD      5940    /* SBF message id: GNSS attitude expressed as Quaternions */
#define ID_ATTCOVQUAD   5941    /* SBF message id: Covariance matrix of attitude as Quaternions */

/* Receiver Time Blocks */
#define ID_RXTIME       5914    /* SBF message id: Current receiver and UTC time */
#define ID_PPSOFFSET    5911    /* SBF message id: Offset of the xPPS pulse with respect to GNSS time */
#define ID_SYSTIMEOFF   4039    /* SBF message id: Time offset between different constellations */
#define ID_FUGROTIMEOFF 4255    /* SBF message id: Fugro clock biases */

/* External Event Blocks */
#define ID_EXTEVENT     5924    /* SBF message id: Time at the instant of an external event */
#define ID_EXTEVENTCART 4937    /* SBF message id: Cartesian position at the instant of an event */
#define ID_EXTEVENTGEO  4938    /* SBF message id: Geodetic position at the instant of an event */
#define ID_EXTEVENTBASEVECCART  4216 /* SBF message id: XYZ relative position with respect to base(s) at the instant of an event */
#define ID_EXTEVENTBASEVECGEOD  4217 /* SBF message id: ENU relative position with respect to base(s) at the instant of an event */
#define ID_EXTEVENTINSNAVCART   4229 /* SBF message id: INS solution in Cartesian coordinates at the instant of an event */
#define ID_EXTEVENTINSNAVGEOD   4230 /* SBF message id: INS solution in Geodetic coordinates at the instant of an event */
#define ID_EXTEEVENTAATTEULER   4237 /* GNSS attitude expressed as Euler angles at the instant of an event */

/* Differential Correction Blocks */
#define ID_DIFFCORRIN   5919    /* SBF message id: Incoming RTCM or CMR message */
#define ID_BASESTATION  5949    /* SBF message id: Base station coordinates  */
#define ID_RTCMDATUM    4049    /* SBF message id: Datum information from the RTK service provider  */
#define ID_BASELINK     5948    /* SBF message id: Number of received and transmitted byte */

/* L-Band Demodulator Blocks */
#define ID_LRECEIVER    4200    /* SBF message id: L-Band Receiver Status */
#define ID_LTRACK       4201    /* SBF message id: Status of the L-band signal tracking */
#define ID_LDECODE      4202    /* SBF message id: Status of the LBAS1 L-band service */
//#define ID_LMESSAGE     4203    /* SBF message id: LBAS1 over-the-air message */
#define ID_LBEAMS       4204    /* SBF message id: L-band satellite/beam information */
#define ID_FUGRODSS     4211    /* SBF message id: DDS (Debug Data Stream) from Fugro */
//#define ID_LRAW         4212  /* SBF message id: L-Band raw user data  */
#define ID_FUGROSTAT    4214    /* SBF message id: Fugro Status Information  */

/* External Sensor Blocks */
#define ID_EXTSENSORMEAS     4050  /* SBF message id: Measurement set of external sensors of one epoch */
#define ID_EXTSENSORSTATUS   4056  /* SBF message id: Overall status of external sensors */
#define ID_EXTSENSORSETUP    4057  /* SBF message id: General information about the setup of external sensors */
#define ID_EXTSENSORSTATUS2  4223  /* SBF message id: Overall status of external sensors */
#define ID_EXTSENSORINFO     4222  /* SBF message id: Configuration information of external sensors */
#define ID_IMUSETUP          4224  /* SBF message id: General information about the setup of the IMU */
#define ID_VELSENSORSETUP    4244  /* SBF message id: General information about the setup of the velocity sensor */
#define ID_LEVERARMSUPPORT1  4248  /* SBF message id: LeverArm estimation */

/* Status Blocks */
#define ID_RXSTATUS1       5913    /* SBF message id: Overall status information of the receiver */
#define ID_TRACKSTATUS     5912    /* SBF message id: Status of the tracking for all receiver channels */
#define ID_CHNSTATUS       4013    /* SBF message id: Status of the tracking for all receiver channels */
#define ID_RXSTATUS2       4014    /* SBF message id: Overall status information of the receiver */
#define ID_SATVISIBILITY   4012    /* SBF message id: Azimuth/elevation of visible satellites  */
#define ID_INPUTLINK       4090    /* SBF message id: Statistics on input streams */
#define ID_OUTPUTLINK      4091    /* SBF message id: Statistics on in´output streams */
#define ID_NTRIPCLTSTAT    4053    /* SBF message id: NTRIP client connection status */
#define ID_NTRIPSVRSTAT    4122    /* SBF message id: NTRIP server connection status */
#define ID_IPSTATUS        4058    /* SBF message id: IP address, gateway and MAC address of Ethernet interface */
#define ID_WIFIAPSTATUS    4054    /* SBF message id: WiFi status in access point mode */
#define ID_WIFICLTSTATUS   4096    /* SBF message id: WiFi status in client mode */
#define ID_CELLULARSTATUS  4055    /* SBF message id: Cellular status */
#define ID_BLUETOOTHSTATUS 4051    /* SBF message id: Bluethooth status */
#define ID_DYNDNSSTATUS    4105    /* SBF message id: DynDNS status */
#define ID_BATTERYSTATUS   4083    /* SBF message id: Battery status */
#define ID_POWERSTATUS     4101    /* SBF message id: Power supply source and voltage */
#define ID_QUALIND         4082    /* SBF message id: Quality indicators */
#define ID_DISKSTATUS      4059    /* SBF message id: Internal logging status */
#define ID_LOGSTATUS       4102    /* SBF message id: Log sessions status  */
#define ID_UHFSTATUS       4085    /* SBF message id: UHF status  */
#define ID_RFSTATUS        4092    /* SBF message id: Radio-frequency interference mitigation status  */
#define ID_RIMSHEALTH      4089    /* SBF message id: Health status of the receiver */
#define ID_OSNMASTATUS     4231    /* SBF message id: OSNMA status information */
#define ID_GALNAVMONITOR   4108    /* SBF message id: Monitoring navigation data per Galileo satellite. */
#define ID_GALNAVRCEDMONITOR 4249  /* SBF message id: Monitoring Reduced CED per Galileo satellite. */
#define ID_INAVMONITOR     4233    /* SBF message id: Reed-Solomon and SSP status information */
#define ID_P2PPSTATUS      4238    /* SBF message id: P2PP client/server status */
//#define ID_AUTHSTATUS     4239
#define ID_COSMOSTATUS     4243    /* SBF message id: Cosmos receiver service status */
#define ID_GALAUTHSTATUS   4245    /* SBF message id: Galileo OSNMA authentication status */
#define ID_FUGROAUTHSTATUS 4256    /* SBF message id: Fugro authentication status */

/* Miscellaneous Blocks */
#define ID_RXSETUP      5902     /* SBF message id: General information about the receiver installation*/
#define ID_RXCOMPS      4084     /* SBF message id: Information on various receiver components  */
#define ID_RXMESSAGE    4103     /* SBF message id: Receiver Messages */
#define ID_COMMANDS     4015     /* SBF message id: Commands entered by the user */
#define ID_COMMENT      5936     /* SBF message id: Comment entered by the user */
#define ID_BBSMPS       4040     /* SBF message id: Baseband samples */
#define ID_ASCIIIN      4075     /* SBF message id: ASCII input from external sensor  */
#define ID_ENCAPSOUT    4097     /* SBF message id: SBF encapsulation of non-SBF messages */
#define ID_RAWDATAIN    4236     /* SBF message id: Incoming raw data message */
#define ID_IMURAWSAMPLES  4250   /* SBF message id: IMU Raw samples */
#define ID_INTERFACESTATS 4261   /* SBF message id: Statistics (data traffic and Internet availability) of every interface. */

/* Advanced Blocks */
#define ID_SYSINFO      6000     /* SBF message id: System parameters for maintenance and support  */


/* get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((const uint8_t *)(p)))
#define I1(p) (*((const int8_t *)(p)))
static uint16_t U2(const uint8_t *p) {uint16_t u; memcpy(&u, p, 2); return u;}
static uint32_t U4(const uint8_t *p) {uint32_t u; memcpy(&u, p, 4); return u;}
static float    R4(const uint8_t *p) {float    r; memcpy(&r, p, 4); return r;}
static double   R8(const uint8_t *p) {double   r; memcpy(&r, p, 8); return r;}
static int32_t  I4(const uint8_t *p) {int32_t  u; memcpy(&u, p, 4); return u;}
static int16_t  I2(const uint8_t *p) {int16_t  i; memcpy(&i, p, 2); return i;}

/* checksum lookup table -----------------------------------------------------*/
static const unsigned short CRC_16CCIT_LookUp[256] = {
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
  0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
  0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
  0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
  0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
  0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
  0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
  0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
  0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
  0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
  0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
  0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
  0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
  0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
  0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
  0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
  0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
  0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
  0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
  0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
  0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
  0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
  0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
  0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
  0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
  0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
  0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};

/* SBF checksum calculation --------------------------------------------------*/
static unsigned short sbf_checksum(unsigned char *buff, int len)
{
    int i;
    unsigned short crc = 0;
    for (i=0; i<len; i++) {
        crc = (crc << 8) ^ CRC_16CCIT_LookUp[ (crc >> 8) ^ buff[i] ];
    }
    return crc;
}

/* count number of bits set in byte ------------------------------------------*/
static uint8_t bitcnt(uint8_t b)
{
    uint8_t n = 0;

    for (uint8_t i = 0; i < 8; i++)
        n += ((b >> i) & 1);

    return n;
}

/* svid to satellite number ([1] 4.1.9) --------------------------------------*/
static int svid2sat(int svid)
{
    if (svid == 0) return 0;
    if (svid <=  37) return satno(SYS_GPS, svid);
    if (svid <=  61) return satno(SYS_GLO, svid-37);
    if (svid <=  62) return 0; /* GLONASS unknown slot */
    if (svid <=  68) return satno(SYS_GLO, svid-38);
    if (svid <=  70) return 0;
    if (svid <= 106) return satno(SYS_GAL, svid-70);
    if (svid <= 119) return 0;
    if (svid <= 140) return satno(SYS_SBS, svid);
    if (svid <= 180) return satno(SYS_CMP, svid-140);
    if (svid <= 187) return satno(SYS_QZS, svid-180+192);
    if (svid <= 190) return 0; /* L-Band (MMS) Satellite */
    if (svid <= 197) return satno(SYS_IRN, svid-190);
    if (svid <= 215) return satno(SYS_SBS, svid-57);
    if (svid <= 222) return satno(SYS_IRN, svid-208);
    if (svid <= 245) return satno(SYS_CMP, svid-182);
    return 0; /* error */
}

/* signal number table ([1] 4.1.10) ------------------------------------------*/
static uint8_t sig_tbl[SBF_MAXSIG+1][2] = { /* system, obs-code */
    {SYS_GPS, CODE_L1C}, /*  0: GPS L1C/A */
    {SYS_GPS, CODE_L1W}, /*  1: GPS L1P */
    {SYS_GPS, CODE_L2W}, /*  2: GPS L2P */
    {SYS_GPS, CODE_L2L}, /*  3: GPS L2C */
    {SYS_GPS, CODE_L5Q}, /*  4: GPS L5 */
    {SYS_GPS, CODE_L1L}, /*  5: GPS L1C */
    {SYS_QZS, CODE_L1C}, /*  6: QZS L1C/A */
    {SYS_QZS, CODE_L2L}, /*  7: QZS L2C */
    {SYS_GLO, CODE_L1C}, /*  8: GLO L1C/A */
    {SYS_GLO, CODE_L1P}, /*  9: GLO L1P */
    {SYS_GLO, CODE_L2P}, /* 10: GLO L2P */
    {SYS_GLO, CODE_L2C}, /* 11: GLO L2C/A */
    {SYS_GLO, CODE_L3Q}, /* 12: GLO L3 */
    {SYS_CMP, CODE_L1P}, /* 13: BDS B1C */
    {SYS_CMP, CODE_L5P}, /* 14: BDS B2a */
    {SYS_IRN, CODE_L5A}, /* 15: IRN L5 */
    {      0,        0}, /* 16: reserved */
    {SYS_GAL, CODE_L1C}, /* 17: GAL E1(L1BC) */
    {      0,        0}, /* 18: reserved */
    {SYS_GAL, CODE_L6C}, /* 19: GAL E6(E6BC) */
    {SYS_GAL, CODE_L5Q}, /* 20: GAL E5a */
    {SYS_GAL, CODE_L7Q}, /* 21: GAL E5b */
    {SYS_GAL, CODE_L8Q}, /* 22: GAL E5 AltBoc */
    {      0,        0}, /* 23: LBand */
    {SYS_SBS, CODE_L1C}, /* 24: SBS L1C/A */
    {SYS_SBS, CODE_L5I}, /* 25: SBS L5 */
    {SYS_QZS, CODE_L5Q}, /* 26: QZS L5 */
    {SYS_QZS, CODE_L6L}, /* 27: QZS L6 */
    {SYS_CMP, CODE_L2I}, /* 28: BDS B1I */
    {SYS_CMP, CODE_L7I}, /* 29: BDS B2I */
    {SYS_CMP, CODE_L6I}, /* 30: BDS B3I */
    {      0,        0}, /* 31: reserved */
    {SYS_QZS, CODE_L1L}, /* 32: QZS L1C */
    {SYS_QZS, CODE_L1Z}, /* 33: QZS L1S */
    {SYS_CMP, CODE_L7D}, /* 34: BDS B2b */
    {      0,        0}, /* 35: reserved */
    {SYS_IRN, CODE_L9A}, /* 36: IRN S */
    {SYS_IRN,        0}, /* 37: IRN L1 */
    {SYS_QZS, CODE_L1E}, /* 38: (tentative) QZSS L1CB */
    {SYS_QZS, CODE_L5P}, /* 39: QZSS L5S */
};

static const int Meas3SigIdx2SignalType_Default[7][16] =
    {
        /* Idx:        0         1          2          3           4         5           6          7                 8          9         10         11         12         13         14         15       */
        /* GPS */ {CODE_L1C, CODE_L2L,   CODE_L5Q,  CODE_L1W,  CODE_L2W,  CODE_L1L, CODE_NONE, CODE_NONE,        CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE},
        /* GLO */ {CODE_L1C, CODE_L2C,   CODE_L1P,  CODE_L2P,  CODE_L3Q, CODE_NONE, CODE_NONE, CODE_NONE,        CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE},
        /* GAL */ {CODE_L1C, CODE_L5Q,   CODE_L7Q,  CODE_L6C,  CODE_L8Q, CODE_NONE, CODE_NONE, CODE_NONE,        CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE},
        /* BDS */ {CODE_L2I, CODE_L7I,   CODE_L6I,  CODE_L1P,  CODE_L5P,  CODE_L7D, CODE_NONE, CODE_NONE,        CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE},
        /* SBA */ {CODE_L1C, CODE_L5I,  CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE,        CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE},
        /* QZS */ {CODE_L1C, CODE_L2L,   CODE_L5Q,  CODE_L6L,  CODE_L1L,  CODE_L1Z,  CODE_L5P, CODE_NONE/*L1CB*/,CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE},
        /* IRN */ {CODE_L5A, CODE_L1E,   CODE_L9A, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE,        CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE, CODE_NONE}
};

static int sigPriority(int sys, int idx, const char *opt, uint8_t *code)
{
    int nex = NEXOBS;
    /* resolve code priority in a freq-index */
    if (sys == SYS_GPS) {
        if (strstr(opt, "-GL1W") && idx==0) return (*code == CODE_L1W) ? 0 : -1;
        if (strstr(opt, "-GL1L") && idx==0) return (*code == CODE_L1L) ? 0 : -1;
        if (strstr(opt, "-GL2L") && idx==1) return (*code == CODE_L2L) ? 1 : -1;
        if (*code == CODE_L1W) return (nex<1) ? -1 : NFREQ;
        if (*code == CODE_L2L) return (nex<2) ? -1 : NFREQ + 1;
        if (*code == CODE_L1L) return (nex<3) ? -1 : NFREQ + 2;
    }
    else if (sys == SYS_GLO) {
        if (strstr(opt, "-RL1P") && idx==0) return (*code == CODE_L1P) ? 0 : -1;
        if (strstr(opt, "-RL2C") && idx==1) return (*code == CODE_L2C) ? 1 : -1;
        if (*code == CODE_L1P) return (nex<1) ? -1 : NFREQ;
        if (*code == CODE_L2C) return (nex<2) ? -1 : NFREQ + 1;
    }
    else if (sys == SYS_QZS) {
        if (strstr(opt, "-JL1L") && idx==0) return (*code == CODE_L1L) ? 0 : -1;
        if (strstr(opt, "-JL1Z") && idx==0) return (*code == CODE_L1Z) ? 0 : -1;
        if (*code == CODE_L1L) return (nex<1) ? -1 : NFREQ;
        if (*code == CODE_L1Z) return (nex<2) ? -1 : NFREQ + 1;
    }
    else if (sys == SYS_CMP) {
        if (strstr(opt, "-CL1P") && idx==0) return (*code == CODE_L1P) ? 0 : -1;
        if (*code == CODE_L1P) return (nex<1) ? -1 : NFREQ;
    }
    return (idx < NFREQ + nex) ? idx : -1;
}

/* signal number to freq-index and code --------------------------------------*/
static int meas2_sig2idx(int sat, int sig, const char *opt, uint8_t *code)
{
    int idx, sys = satsys(sat, NULL);

    if (sig<0 || sig>SBF_MAXSIG || sig_tbl[sig][0]!=sys) return -1;
    *code = sig_tbl[sig][1];
    idx = code2idx(sys, *code);

    return sigPriority(sys, idx, opt, code);
}

/* signal number to freq-index and code for meas3 data -----------------------*/
static int meas3_sig2idx(int sbf_navsys, int sig, const char *opt, uint8_t *code, int sigTable[7][16])
{
    int idx;

    if (sig<0 || sig>=MEAS3_SIG_MAX) return -1;
    if (sbf_navsys<0 || sbf_navsys>=MEAS3_SYS_MAX) return -1;
    *code = sigTable[sbf_navsys][sig];
    idx = code2idx(Meas3_NavSys[sbf_navsys], *code);

    return sigPriority(Meas3_NavSys[sbf_navsys], idx, opt, code);
}

/* initialize obs data fields ------------------------------------------------*/
static void init_obsd(gtime_t time, int sat, obsd_t *data)
{
    int i;

    data->time = time;
    data->sat = (uint8_t)sat;

    for (i = 0; i < NFREQ+NEXOBS; i++) {
        data->L[i] = data->P[i] = 0.0;
        data->D[i] = 0.0f;
        data->Lstd[i] = data->Pstd[i] = 0.0;
        data->SNR[i] = (uint16_t)0;
        data->LLI[i] = (uint8_t)0;
        data->code[i] = CODE_NONE;
    }
}

/* 8-bit week -> full week ---------------------------------------------------*/
static void adj_utcweek(gtime_t time, double *utc)
{
    int week;

    if (*utc >= 256.0) return;
    time2gpst(time, &week);
    *utc += (week / 256) * 256;
    if      (*utc < week-128) *utc += 256.0;
    else if (*utc > week+128) *utc -= 256.0;
}

/* convert 8-bit week -> full week --------------------------------------------*/
static uint16_t adjust_WN8(uint16_t ref_WN, uint8_t WN)
{
    int16_t offset = (ref_WN % 256) - WN;
    if (offset > 128) offset -= 256;
    if (offset < -127) offset += 256;
    return ref_WN + offset;
}
/* convert 10-bit week -> full week --------------------------------------------*/
static uint16_t adjust_WN10(uint16_t ref_WN, uint16_t WN)
{
    int16_t offset = (ref_WN % 1024) - WN;
    if (offset > 512) offset -= 1024;
    if (offset < -511) offset += 1024;
    return ref_WN + offset;
}
/* convert 12-bit week -> full week --------------------------------------------*/
static uint16_t adjust_WN12(uint16_t ref_WN, uint16_t WN)
{
    int16_t offset = (ref_WN % 4096) - WN;
    if (offset > 2048) offset -= 4096;
    if (offset < -2047) offset += 4096;
    return ref_WN + offset;
}
/* convert 14-bit week -> full week --------------------------------------------*/
static uint16_t adjust_WN14(uint16_t ref_WN, uint16_t WN)
{
    int16_t offset = (ref_WN % 8192) - WN;
    if (offset > 4096) offset -= 8192;
    if (offset < -4095) offset += 8192;
    return ref_WN + offset;
}/* adjust daily rollover of time ---------------------------------------------*/
static gtime_t adjday(gtime_t time, double tod)
{
    double ep[6], tod_p;
    time2epoch(time, ep);
    tod_p = ep[3]*3600.0 + ep[4]*60.0 + ep[5];
    if      (tod < tod_p-43200.0) tod += 86400.0;
    else if (tod > tod_p+43200.0) tod -= 86400.0;
    ep[3] = ep[4] = ep[5] = 0.0;
    return timeadd(epoch2time(ep), tod);
}

/* Measurement Blocks */

/* flush observation data buffer ---------------------------------------------*/
static int flushobuf(raw_t *raw) {
    int i, j, n = 0;

    trace(3, "flushobuf: n=%d\n", raw->obuf.n);

    /* copy observation data buffer */
    for (i = 0; i < raw->obuf.n && i < MAXOBS; i++) {
        if (!satsys(raw->obuf.data[i].sat, NULL)) continue;
        if (raw->obuf.data[i].time.time == 0) continue;
        raw->obs.data[n++] = raw->obuf.data[i];
    }
    raw->obs.n = n;

    /* clear observation data buffer */
    for (i = 0; i < MAXOBS; i++) {
        raw->obuf.data[i].time.time = 0;
        raw->obuf.data[i].time.sec = 0;
        for (j = 0; j < NFREQ + NEXOBS; j++) {
            raw->obuf.data[i].L[j] = raw->obuf.data[i].P[j] = 0.0;
            raw->obuf.data[i].Lstd[j] = raw->obuf.data[i].Pstd[j] = 0.0;
            raw->obuf.data[i].D[j] = 0.0;
            raw->obuf.data[i].SNR[j] = raw->obuf.data[i].LLI[j] = 0;
            raw->obuf.data[i].code[j] = CODE_NONE;
        }
    }
    for (i = 0; i < MAXSAT; i++) {
        raw->prCA[i] = raw->dpCA[i] = 0.0;
    }
    return n > 0 ? 1 : 0;
}

/* decode SBF measurements message (observables) -----------------------------*/
/*
 * This is the most important block in the SBF format. It it contains all code
 * pseudoranges and carrier phase measurements of all received satellites.
 * This block is made of one Type1 sub-block per satellite followed by, if any,
 * a certain number of Type2 sub-blocks. SB2Num defines how many Type2
 * sub-blocks there are inside its Type1 sub-block.
 * Type1 subblock contains code pseudorange and carrier phase range of the first
 * decoded signal defined by signType1, this is typically L1 signal.
 * Any following Type2 sub-block (if there are any) contains signType2 signal
 * information, typically L2 signal. Inside Type2 sub-blocks, information is
 * expressed as difference from the data in signType1 sub-block. This makes the
 * format a little more compact.
 *
*/
static int decode_measepoch(raw_t *raw)
{
    sbf_t *sbf = (sbf_t *)raw->rcv_data;
    uint8_t *p = raw->buff+14, code;
    double P1, P2, L1, L2, D1, D2, S1, S2, freq1, freq2;
    int i, j, idx, n, n1, n2, len1, len2, sig, ant, svid, info, sat, sys, lock, fcn, LLI, chn, ret=0;
    int ant_sel = 0; /* antenna selection (0:main) */

    if (timediff(raw->time, sbf->current_time) != 0) {
        sbf->current_time = raw->time;
        ret = flushobuf(raw);
    }

    if (strstr(raw->opt, "-NO_MEAS2")) return ret;

    if      (strstr(raw->opt, "-AUX1")) ant_sel = 1;
    else if (strstr(raw->opt, "-AUX2")) ant_sel = 2;

    if (raw->len < 20) {
        trace(2, "sbf measepoch length error: len=%d\n", raw->len);
        return ret ? ret : -1;
    }
    n1   = U1(p);
    len1 = U1(p+1);   /* size of measurement block type 1 */
    len2 = U1(p+2);   /* size of measurement block type 2 */

    if (U1(p+3) & 0x80) {
        trace(2, "sbf measepoch scrambled\n");
        return ret ? ret : -1;
    }

    // reset channel assignment
    memset(&sbf->meas2_channelAssignment, -1, 2048*sizeof(int));

    if (raw->outtype) {
        sprintf(raw->msgtype+strlen(raw->msgtype), " nsat=%d", n1);
    }

    for (i=n=0, p+=6; i<n1 && n<MAXOBS && p+20 <= raw->buff+raw->len; i++) {
        /* byte 0: receiver channel */
        chn  = U1(p);
        ant  = U1(p+1) >> 5;
        sig  = U1(p+1) & 0x1f;
        svid = U1(p+2);
        info = U1(p+18);
        n2   = U1(p+19); /* number of type 2 measurement blocks */
        fcn  = 0;
        if (sig == 31) sig = (info>>3)+32;
        else if (sig>=8 && sig<=11) fcn = (info>>3)-8;
        raw->obuf.data[n].freq = fcn+7;

        if (ant != ant_sel) {
            trace(3, "sbf measepoch ant error: svid=%d ant=%d\n", svid, ant);
            p += len1 + len2*n2; /* skip block (and its sub-blocks)*/
            continue;
        }
        sat = svid2sat(svid);
        if (!sat) {
            trace(3, "sbf measepoch svid error: svid=%d\n", svid);
            p += len1 + len2*n2; /* skip block (and its sub-blocks)*/
            continue;
        }
        idx = meas2_sig2idx(sat, sig, raw->opt, &code);
        if (idx < 0) {
            trace(2, "sbf measepoch sig error: sat=%d sig=%d\n", sat, sig);
            p+= len1 + len2*n2;  /* skip block (and its sub-blocks)*/
            continue;
        }
        init_obsd(raw->time, sat, raw->obuf.data+n);
        sbf->meas2_channelAssignment[chn] = n;
        P1 = D1 = 0.0;
        sys = satsys(sat, NULL);
        freq1 = code2freq(sys, code, fcn);

        if ((U1(p+3) & 0x0f) !=0 || U4(p+4) != 0) {
            P1 = (U1(p+3) & 0x0f)*4294967.296 + U4(p+4)*0.001;
            raw->obuf.data[n].P[idx] = P1;
        }
        if (I4(p+8) != -2147483648) {
            D1 = I4(p+8)*0.0001;
            raw->obuf.data[n].D[idx] = (float)D1;
        }

        lock = U2(p+16);
        if (P1!=0.0 && freq1>0.0 && lock!=65535 && (I1(p+14) != -128 || U2(p+12) != 0)) {
            L1 = I1(p+14)*65.536 + U2(p+12)*0.001;
            raw->obuf.data[n].L[idx] = P1*freq1/CLIGHT + L1;
            LLI = (lock<raw->lockt[sat-1][idx] ? 1 : 0) + ((info & (1<<2)) ? 2 : 0);
            raw->obuf.data[n].LLI[idx] = (uint8_t)LLI;
            raw->lockt[sat-1][idx] = lock;
        }
        if (U1(p+15) != 255) {
            S1 = U1(p+15)*0.25 + ((sig==1 || sig==2) ? 0.0 : 10.0);
            raw->obuf.data[n].SNR[idx] = (uint16_t)(S1/SNR_UNIT+0.5);
        }
        raw->obuf.data[n].code[idx] = code;

        for (j=0, p+=len1; j<n2 && p+12<=(raw->buff+raw->len); j++, p+=len2) {
            sig  = U1(p) & 0x1f;
            ant  = U1(p) >> 5;
            lock = U1(p+1);
            info = U1(p+5);
            if (sig == 31) sig = (info>>3)+32;

            if (ant != ant_sel) {
                trace(3, "sbf measepoch ant error: sat=%d ant=%d\n", sat, ant);
                continue;
            }
            if ((idx = meas2_sig2idx(sat, sig, raw->opt, &code)) < 0) {
                trace(3, "sbf measepoch sig error: sat=%d sig=%d\n", sat, sig);
                continue;
            }
            P2 = 0.0;
            freq2 = code2freq(sys, code, fcn);
            if (lock != 255) {
                LLI = (lock<raw->lockt[sat-1][idx] ? 1 : 0) + ((info&(1<<2)) ? 2 : 0);
                raw->obuf.data[n].LLI[idx] = (uint8_t)LLI;
                raw->lockt[sat-1][idx] = lock;
            }
            if (U1(p+2) != 255) {
                S2 = U1(p+2)*0.25 + ((sig==1 || sig==2) ? 0.0 : 10.0);
                raw->obuf.data[n].SNR[idx] = (uint16_t)(S2/SNR_UNIT+0.5);
            }
            if (P1!=0.0 && (getbits(p+3, 5, 3)!=-4 || U2(p+6)!=0)) {
                P2 = P1+getbits(p+3, 5, 3)*65.536 + U2(p+6)*0.001;
                raw->obuf.data[n].P[idx] = P2;
            }
            if (P2!=0.0 && freq2>0.0 && (I1(p+4)!=-128 || U2(p+8)!=0)) {
                L2 = I1(p+4)*65.536 + U2(p+8)*0.001;
                raw->obuf.data[n].L[idx] = P2*freq2/CLIGHT+L2;
            }
            if (D1!=0.0 && freq1>0.0 && freq2>0.0 && (getbits(p+3, 0, 5)!=-16 || U2(p+10)!=0)) {
                D2 = getbits(p+3, 0, 5)*6.5536 + U2(p+10)*0.0001;
                raw->obuf.data[n].D[idx] = (float)(D1*freq2/freq1)+D2;
            }

            raw->obuf.data[n].code[idx] = code;
        }
        n++;
    }
    raw->obuf.n = n;

    return ret;
}

static int decode_measepochextra(raw_t *raw)
{
    sbf_t *sbf = (sbf_t *)raw->rcv_data;
    uint8_t *p = raw->buff+8;
    uint8_t n_chn, sbLen, i, misc;
    uint8_t code, type, chn;
    uint16_t revision;
    int sig, n, idx, sat, ret = 0;
    int ant_sel = 0; /* antenna selection (0:main) */

    if (timediff(raw->time, sbf->current_time) != 0) {
        sbf->current_time = raw->time;
        ret = flushobuf(raw);
    }

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF Measurement Data Extra");
    }

    if (strstr(raw->opt, "-NO_MEAS2")) return ret;

    if      (strstr(raw->opt, "-AUX1")) ant_sel = 1;
    else if (strstr(raw->opt, "-AUX2")) ant_sel = 2;

    revision = U2(raw->buff+4) >> 13;
    n_chn = U1 (p+6);
    sbLen = U1(p+7);

    int rcvstds = 0;
    if (strstr(raw->opt, "-RCVSTDS")) rcvstds = 1;

    for (i = 0; i < n_chn; i++)
    {
        uint8_t *p_chan = p+12+i*sbLen;

        chn = U1(p_chan);
        type = U1(p_chan+1);
        if ((type >> 5) != ant_sel)   /* check selected antenna */
            continue;
        sig = type & 0x1f;
        if (sig == 31)
            sig = (U1(p_chan + 15) >> 3) + 32;

        n = sbf->meas2_channelAssignment[chn];
        if (n < 0) {
            continue;  /* channel is acquired but not tracked */
        }
        sat = raw->obuf.data[n].sat;
        idx = meas2_sig2idx(sat, sig, raw->opt, &code);
        if (idx < 0) {
            trace(2, "sbf measepochextra sig error: sat=%d sig=%d\n", sat, sig);
            continue;
        }

        /* Write rcvr stdevs to unused RINEX fields */
        if (rcvstds) {
            uint16_t codeVarU = U2(p_chan + 6);
            if (codeVarU != 65535) {
                double codeVar = codeVarU / 10000.;   /* meters^2 */
                // To RTKlib encoding
                // TODO starting from 2 rather than 5 would better suit here
                double pstd = log2(sqrt(codeVar) * 100) - 5;
                pstd = pstd > 0 ? pstd : 0;
                /* Further limited to 9 in RINEX output */
                pstd = pstd <= 254 ? pstd : 254;
                raw->obuf.data[n].Pstd[idx] = pstd + 0.5;
            }

            uint16_t carrierVarU = U2(p_chan + 8);
            if (carrierVarU != 65535) {
                double carrierVar = carrierVarU / 1000000.; /* cycles^2 */
                // To RTKlib encoding
                double lstd = sqrt(carrierVar) / 0.004;
                lstd = lstd > 0 ? lstd : 0;
                /* Further limited to 9 in RINEX output */
                lstd = lstd <= 254 ? lstd : 254;
                raw->obuf.data[n].Lstd[idx] = lstd + 0.5;
            }
        }

        if ((revision >= 3) && (sbLen >= 16)) {  /* later revision contains high-resolution extension for CN0 values*/
            misc = U1(p_chan+15);
            raw->obuf.data[n].SNR[idx] = (uint16_t)((raw->obuf.data[n].SNR[idx] * SNR_UNIT + (misc & 0x7) * 0.03125) / SNR_UNIT + 0.5);
        }
    }

    return ret;
}

/* decode meas3 bloack -------------------------------------------------*/
static int decode_meas3ranges(raw_t *raw) {
    sbf_t *sbf = (sbf_t *)raw->rcv_data;
    uint8_t *p = raw->buff+8;
    int n = 0, idx = 0, ret = 0;
    int ant_sel = 0; /* antenna selection (0:main) */

    if (timediff(raw->time, sbf->current_time) != 0) {
        sbf->current_time = raw->time;
        ret = flushobuf(raw);
    }

    if (strstr(raw->opt, "-NO_MEAS3")) return ret;

    if      (strstr(raw->opt, "-AUX1")) ant_sel = 1;
    else if (strstr(raw->opt, "-AUX2")) ant_sel = 2;

    if (raw->len < 12) {
        trace(2, "sbf meas3ranges length error: len=%d\n", raw->len);
        return ret ? ret : -1;
    }

    uint32_t TOW = U4(p + 0);
    /* bit 0: multipath mitigation, bit 1: at least one smoothing, bit 2: reserved, bit 3: clock steering active
       bit 4: measurement from data component, bit 5: high-dynamic mode, bit 6: E6B used, bit 7: scrambled data */
    /* uint8_t commonFlags = U1(p+6); */
    // int16_t clkJumps = U1(p+7);
    // if (clkJumps >= 128) clkJumps -= 256; /* cummulated clock jumps in ms */
    uint8_t constellations = U2(p+8);
    uint8_t misc = U1(p+10); /* bit 3: PPR available */
    uint8_t reserved = U1(p+11); /* is actually a version indicator */
    uint8_t prrAvailable = (misc & 8) != 0; /* pseudo-range change rate available in data */

    if (reserved > 31) {
        trace(2, "sbf meas3ranges invalid data version: len=%d\n", raw->len);
        return ret ? ret : -1;
    }

    int antennaIdx = misc & 7;
    if (ant_sel != antennaIdx) return ret;
    uint32_t refEpochInterval = Meas3_EpochIntervals[misc >> 4]; /* interval for full epoche data */

    // if this is a reference epoch?
    if ((TOW % refEpochInterval) == 0) // clean-up old data
    {
        memset(&sbf->meas3_refEpoch, 0, sizeof(Meas3_RefEpoch_t));
        sbf->meas3_refEpoch.TOW = TOW;
    }

    p += 12; // jump to start of data

    /* invalidate frequency assignments */
    memset(sbf->meas3_freqAssignment, -1, MEAS3_SYS_MAX*MEAS3_SAT_MAX*MEAS3_SIG_MAX*sizeof(uint8_t));

    if (((TOW % refEpochInterval) != 0) &&  // check reference epoch
        (sbf->meas3_refEpoch.TOW != (uint32_t)(TOW / refEpochInterval)*refEpochInterval)) { // or reference epoch is available
        raw->obuf.n = 0;
        return ret;
    }

        for (int navsys = 0; navsys < MEAS3_SYS_MAX; navsys++) {
            if ((constellations & (1 << navsys)) == 0)
                continue;  /* no data for this navigation system */

            uint8_t *p_navsys = p+idx, idx_navsys = 0;
            uint8_t nSats = 0, satCnt = 0, sigExcluded;
            uint16_t bdsLongRange = 0;
            uint8_t gloFncs[8] = {0, 0, 0, 0, 0, 0, 0, 0};
            uint64_t satMask = 0;
            int sigTable[MEAS3_SYS_MAX][MEAS3_SIG_MAX];

            // read satellite data
            if (U1(p+idx) == 0)
                p_navsys = sbf->meas3_refEpoch.constellationHeader[navsys];

            uint8_t BF1 = U1(p_navsys+idx_navsys);
            uint8_t nB = BF1 & 0x7;
            uint8_t signalIndexMasterShort = (BF1 >> 3) & 0xf;
            uint8_t signalExcludedPresent = (BF1 >> 7) != 0;
            idx_navsys++;
            if (nB == 7) nB = 8;

            // read satellite mask
            for (int i = 0; i < nB; i++) {
                satMask |= (uint64_t)U1(p_navsys+idx_navsys+i) << (i*8);
                nSats += bitcnt(U1(p_navsys+idx_navsys+i));
            }
            idx_navsys += nB;

            // Read GLONASS fcn list
            if (navsys == 1) {  // GLONASS
                memcpy(gloFncs, p_navsys+idx_navsys,  (nSats+1) / 2);
                idx_navsys += (nSats+1) / 2;
            } else if (navsys  == 3) {  // BDS
                bdsLongRange = U2(p_navsys+idx_navsys);
                idx_navsys += 2;
            }

            if (signalExcludedPresent) {
                sigExcluded = U1(p_navsys+idx_navsys);
                idx_navsys++;
            } else {
                sigExcluded = 0;
            }

            if ((TOW % refEpochInterval) == 0) { // reference epoche
                if (idx_navsys > 23)
                    trace(2, "sbf meas3ranges idx_navsys too large\n");
                memcpy(sbf->meas3_refEpoch.constellationHeader[navsys], p+idx, idx_navsys);
            }

            if (U1(p+idx) == 0)  // if data were from the reference block
                idx += 1;
            else
                idx += idx_navsys;

            // prepare signal table
            uint8_t j = 0;
            for (uint8_t i = 0; i < MEAS3_SIG_MAX; i++)
                /* signals that correspond to the zero bits must be included */
                if (((uint32_t)sigExcluded & (1 << i)) == 0) {
                    sigTable[navsys][j] = Meas3SigIdx2SignalType_Default[navsys][i];
                    j++;
                }
            /* remaining signals do not exist */
            for (;j < MEAS3_SIG_MAX; j++)
                sigTable[navsys][j] = CODE_NONE;

            for (int svid = 0; svid < MEAS3_SAT_MAX && satCnt < nSats; svid++)
            {
                if ((satMask & (1ULL << svid)) == 0)
                    continue;

                int8_t glofnc = 0;
                if (navsys == 1) {  // GLONASS
                    glofnc = (int)((gloFncs[satCnt / 2] >> (4 * (satCnt % 2))) & 0xf) - 8;
                }
                int satNo;
                int masterFreqIndex;
                uint8_t codeMaster;
                double freqMaster = 0;
                uint32_t slaveSignalMask = 0, masterSignalIndex;
                int16_t prRate;

                satNo = satno(Meas3_NavSys[navsys], Meas3_SVIDBase[navsys]+svid);

                // decode master measurement
                double prbase = (bdsLongRange & (1 << satCnt)) != 0 ? 34e6 : PRBase[navsys];
                int blockTypeMaster = U1(p+idx);
                init_obsd(raw->time, satNo, raw->obuf.data+n);

                if ((blockTypeMaster & 1) == 1) {
                    // Master short
                    uint32_t BF1    = U4(p+idx);
                    uint32_t pr_lsb  = U4(p+idx+4);

                    uint32_t cmc    = (BF1 >> 1) & 0x3ffff;
                    uint32_t prMsb  = (BF1 >> 19) & 1;
                    uint32_t lti3   = (BF1 >> 20) & 0x7;
                    uint32_t CN0    = (BF1 >> 23) & 0x1f;
                    uint32_t signalList = (BF1 >> 28) & 0xf;
                    uint32_t lockTime = Meas3_LTItoPLLTime[lti3];

                    masterSignalIndex = signalIndexMasterShort;
                    slaveSignalMask = signalList << (masterSignalIndex + 1);

                    if ((masterFreqIndex = meas3_sig2idx(navsys, masterSignalIndex, raw->opt, &codeMaster, sigTable)) >= 0 && satNo > 0) {
                        freqMaster = code2freq(Meas3_NavSys[navsys], codeMaster, glofnc);
                        double pr = prbase + ((double)pr_lsb + 4294967296.0 * (double)prMsb) * .001;
                        raw->obuf.data[n].P[masterFreqIndex] = pr;
                        raw->obuf.data[n].SNR[masterFreqIndex] = (uint16_t)((CN0 + 24.0)/SNR_UNIT+0.5);
                        raw->obuf.data[n].code[masterFreqIndex] = codeMaster;
                        if (cmc != 0)
                            raw->obuf.data[n].L[masterFreqIndex] = pr / (CLIGHT/freqMaster) - 131.072 + (double)cmc * .001;

                        raw->obuf.data[n].LLI[masterFreqIndex] = (lockTime < raw->lockt[satNo-1][masterFreqIndex] ? LLI_SLIP : 0) | (lti3 == 0 ? LLI_HALFC : 0);
                        raw->lockt[satNo-1][masterFreqIndex] = lockTime;
                        raw->obuf.data[n].freq = glofnc+7;
                        sbf->meas3_freqAssignment[navsys][svid][0] = masterFreqIndex;
                    };

                    if (prrAvailable)
                        prRate = I2(p+idx+8);
                    else
                        prRate = 0;

                    idx += prrAvailable ? 10 : 8;
                } else if ((blockTypeMaster & 3) == 0) {
                    // Master long
                    uint32_t BF1    = U4(p+idx);
                    uint32_t prLsb  = U4(p+idx+4);
                    uint16_t BF2    = U2(p+idx+8);
                    uint8_t  BF3    = U1(p+idx+10);

                    uint32_t prMsb  = (BF1 >> 2) & 0xf;
                    uint32_t cmc    = (BF1 >> 6) & 0x3fffff;
                    uint32_t lti4   = (BF1 >> 28) & 0xf;
                    uint32_t CN0    = (BF2 >> 0) & 0x3f;
                    uint32_t signalMask = (BF2 >> 6) & 0x1ff;
                    uint32_t cont   = (BF2 >> 15) & 0x1;
                    uint32_t lockTime = Meas3_LTItoPLLTime[lti4];

                    if (cont != 0)
                        signalMask |= (uint32_t)(BF3 & 0x7f) << 9;

                    /* masterSignalIdx is the index of the right-most bit set to 1 */
                    for (masterSignalIndex = 0; masterSignalIndex < 32; masterSignalIndex++)
                        if (((signalMask >> masterSignalIndex) & 1) != 0)
                            break;
                    slaveSignalMask = signalMask ^ 1UL << (masterSignalIndex);

                    if ((masterFreqIndex = meas3_sig2idx(navsys, masterSignalIndex, raw->opt, &codeMaster, sigTable)) >= 0 && satNo > 0) {
                        freqMaster = code2freq(Meas3_NavSys[navsys], codeMaster, glofnc);
                        uint8_t isGPSPCode = (navsys == 0) && (codeMaster == CODE_L1W || codeMaster == CODE_L2W);

                        raw->obuf.data[n].P[masterFreqIndex] = ((double)prLsb + 4294967296.0 * (double)prMsb) * .001;
                        raw->obuf.data[n].SNR[masterFreqIndex] = (uint16_t)((isGPSPCode ? CN0 : CN0 + 10.0) / SNR_UNIT + 0.5);
                        raw->obuf.data[n].code[masterFreqIndex] = codeMaster;
                        if (cmc != 0)
                            raw->obuf.data[n].L[masterFreqIndex] = raw->obuf.data[n].P[masterFreqIndex] / (CLIGHT/freqMaster) - 2097.152 + (double)cmc * .001;

                        raw->obuf.data[n].LLI[masterFreqIndex] = (lockTime < raw->lockt[satNo-1][masterFreqIndex] ? LLI_SLIP : 0) | (lti4 == 0 ? LLI_HALFC : 0);
                        raw->lockt[satNo-1][masterFreqIndex] = lockTime;
                        raw->obuf.data[n].freq = glofnc+7;
                        sbf->meas3_freqAssignment[navsys][svid][0] = masterFreqIndex;
                    };

                    if (prrAvailable)
                        prRate = I2(p+idx+10);
                    else
                        prRate = 0;

                    idx += prrAvailable ? 12 + cont : 10 + cont;
                } else if ((blockTypeMaster & 0xc) == 0xc) {
                    // Master long delta
                    masterSignalIndex = sbf->meas3_refEpoch.signalIdx[navsys][svid][0];
                    slaveSignalMask = sbf->meas3_refEpoch.slaveSignalMask[navsys][svid];
                    if ((masterFreqIndex = meas3_sig2idx(navsys, masterSignalIndex, raw->opt, &codeMaster, sigTable)) >= 0 && satNo > 0) {
                        uint8_t   BF1   = U1(p+idx);
                        uint32_t  BF2   = U4(p+idx+1);

                        uint32_t  pr    = (((uint32_t)(BF1 >> 4) << 13) | (BF2 & 0x1fff));
                        uint32_t  CN0   = (BF2 >> 13) & 0x7;
                        uint32_t  cmc   = BF2 >> 16;

                        obsd_t * master_reference = &(sbf->meas3_refEpoch.obsData[navsys][svid]);
                        freqMaster = code2freq(Meas3_NavSys[navsys], codeMaster, glofnc);

                        raw->obuf.data[n].P[masterFreqIndex] = master_reference->P[masterFreqIndex] +
                                                       ((int64_t)sbf->meas3_refEpoch.prRate[navsys][svid] * 64 * (int32_t)(TOW % refEpochInterval) / 1000) * .001 +
                                                       (double)pr * .001 - 65.536;
                        raw->obuf.data[n].SNR[masterFreqIndex] = (uint16_t)((master_reference->SNR[masterFreqIndex] * SNR_UNIT - 4.0 + CN0) / SNR_UNIT + 0.5);
                        raw->obuf.data[n].code[masterFreqIndex] = codeMaster;
                        if (cmc != 0)
                            raw->obuf.data[n].L[masterFreqIndex] = (raw->obuf.data[n].P[masterFreqIndex] - master_reference->P[masterFreqIndex]) / (CLIGHT/freqMaster) +
                                                           master_reference->L[masterFreqIndex] - 32.768 + (double)cmc * .001;

                        raw->obuf.data[n].LLI[masterFreqIndex] = master_reference->LLI[masterFreqIndex];
                        raw->lockt[satNo-1][masterFreqIndex] = sbf->meas3_refEpoch.lockt[navsys][svid][masterFreqIndex];
                        raw->obuf.data[n].freq = glofnc+7;
                        sbf->meas3_freqAssignment[navsys][svid][0] = masterFreqIndex;
                    }

                    prRate = 0;

                    idx += 5;
                } else {
                    // Master short delta
                    uint32_t  BF1   = U4(p+idx);

                    uint32_t  pr    = (BF1 >> 4) & 0x3fff;
                    uint32_t  cmc   = (BF1 >> 18) & 0x3fff;
                    uint32_t  CN0   = (BF1 >> 2) & 0x3;

                    masterSignalIndex = sbf->meas3_refEpoch.signalIdx[navsys][svid][0];
                    if ((masterFreqIndex = meas3_sig2idx(navsys, masterSignalIndex, raw->opt, &codeMaster, sigTable)) >= 0 && satNo > 0) {
                        obsd_t * masterReference = &(sbf->meas3_refEpoch.obsData[navsys][svid]);
                        freqMaster = code2freq(Meas3_NavSys[navsys], codeMaster, glofnc);

                        raw->obuf.data[n].P[masterFreqIndex] = masterReference->P[masterFreqIndex] + ((int64_t)sbf->meas3_refEpoch.prRate[navsys][svid] * 64 * (int32_t)(TOW % refEpochInterval) / 1000) * .001 + (double)pr * .001 - 8.192;
                        if (cmc != 0)
                            raw->obuf.data[n].L[masterFreqIndex] = (raw->obuf.data[n].P[masterFreqIndex] - masterReference->P[masterFreqIndex]) / (CLIGHT/freqMaster) + masterReference->L[masterFreqIndex] - 8.192 + (double)cmc * .001;
                        raw->obuf.data[n].SNR[masterFreqIndex] = (uint16_t)((masterReference->SNR[masterFreqIndex] * SNR_UNIT - 1.0 + CN0) / SNR_UNIT + 0.5);
                        raw->obuf.data[n].LLI[masterFreqIndex] = masterReference->LLI[masterFreqIndex];
                        raw->lockt[satNo-1][masterFreqIndex] = sbf->meas3_refEpoch.lockt[navsys][svid][masterFreqIndex];
                        raw->obuf.data[n].code[masterFreqIndex] = codeMaster;
                        raw->obuf.data[n].freq = glofnc+8;
                        sbf->meas3_freqAssignment[navsys][svid][0] = masterFreqIndex;
                    }

                    prRate = 0;
                    slaveSignalMask = sbf->meas3_refEpoch.slaveSignalMask[navsys][svid];

                    idx += 4;
                }

                /* keep reference measurement to decode the delta measurements */
                if (TOW % refEpochInterval == 0  && satNo > 0)
                {
                    if ((masterFreqIndex > NFREQ+NEXOBS) || (masterFreqIndex < 0))
                        trace(2, "sbf meas3ranges index out of bounds: %d\n", masterFreqIndex);

                    sbf->meas3_refEpoch.signalIdx[navsys][svid][0] = (uint8_t)masterSignalIndex;
                    sbf->meas3_refEpoch.slaveSignalMask[navsys][svid] = slaveSignalMask;
                    sbf->meas3_refEpoch.prRate[navsys][svid] = prRate;
                    sbf->meas3_refEpoch.obsData[navsys][svid].P[masterFreqIndex] = raw->obuf.data[n].P[masterFreqIndex];
                    sbf->meas3_refEpoch.obsData[navsys][svid].L[masterFreqIndex] = raw->obuf.data[n].L[masterFreqIndex];
                    sbf->meas3_refEpoch.obsData[navsys][svid].SNR[masterFreqIndex] = raw->obuf.data[n].SNR[masterFreqIndex];
                    sbf->meas3_refEpoch.obsData[navsys][svid].LLI[masterFreqIndex] = raw->obuf.data[n].LLI[masterFreqIndex];
                    sbf->meas3_refEpoch.lockt[navsys][svid][masterFreqIndex] = raw->lockt[satNo-1][masterFreqIndex];
                }

                // update PLL lock time
                if (satNo > 0  && raw->lockt[satNo-1][masterFreqIndex] > sbf->meas3_refEpoch.lockt[navsys][svid][masterFreqIndex])
                    sbf->meas3_refEpoch.lockt[navsys][svid][masterFreqIndex] = raw->lockt[satNo-1][masterFreqIndex];

                /* decode slave data */
                int slaveCnt = 0;
                for (int slaveSignalIndex = 1; slaveSignalIndex < MEAS3_SIG_MAX && slaveSignalMask != 0; slaveSignalIndex++)
                {
                    if ((slaveSignalMask & (1 << slaveSignalIndex)) != 0)
                    {
                        int slaveFreqIndex;
                        double freqSlave;
                        uint8_t codeSlave;
                        int blockTypeSlave = U1(p+idx);

                        if ((blockTypeSlave & 1) == 1) {
                            /* Slave Short */
                            uint32_t BF1    = U4(p+idx);
                            uint8_t  BF2    = U1(p+idx+4);

                            uint32_t cmcRes = (BF1 >> 1) & 0xffff;
                            uint32_t prRel  = BF1 >> 17;
                            uint32_t lti3   = BF2 & 0x7;
                            uint32_t CN0    = BF2 >> 3;
                            uint32_t lockTime = Meas3_LTItoPLLTime[lti3];

                            if ((slaveFreqIndex = meas3_sig2idx(navsys, slaveSignalIndex, raw->opt, &codeSlave, sigTable)) >= 0 && satNo > 0) {
                                freqSlave = code2freq(Meas3_NavSys[navsys], codeSlave, glofnc);

                                if (freqMaster > freqSlave)
                                    raw->obuf.data[n].P[slaveFreqIndex] = raw->obuf.data[n].P[masterFreqIndex] + prRel * .001 - 10;
                                else
                                    raw->obuf.data[n].P[slaveFreqIndex] = raw->obuf.data[n].P[masterFreqIndex] - prRel * .001 + 10;

                                if (cmcRes != 0)
                                    raw->obuf.data[n].L[slaveFreqIndex] =
                                                         raw->obuf.data[n].P[slaveFreqIndex] / (CLIGHT/freqSlave) +
                                                          (raw->obuf.data[n].L[masterFreqIndex] - raw->obuf.data[n].P[masterFreqIndex] / (CLIGHT/freqMaster)) * (freqMaster / freqSlave)
                                                          - 32.768 + cmcRes * .001;

                                if ((navsys == 0) && (codeSlave == CODE_L1W || codeSlave == CODE_L2W))
                                    raw->obuf.data[n].SNR[slaveFreqIndex] = (uint16_t)((raw->obuf.data[n].SNR[masterFreqIndex] * SNR_UNIT - 3.0 - CN0) / SNR_UNIT + 0.5);
                                else
                                    raw->obuf.data[n].SNR[slaveFreqIndex] = (uint16_t)((CN0 + 24.0) / SNR_UNIT + 0.5);

                                raw->obuf.data[n].code[slaveFreqIndex] = codeSlave;
                                raw->obuf.data[n].LLI[slaveFreqIndex] = (lockTime < raw->lockt[satNo-1][slaveFreqIndex] ? LLI_SLIP : 0) | (lti3 == 0 ? LLI_HALFC : 0);
                                raw->lockt[satNo-1][slaveFreqIndex] = lockTime;
                                sbf->meas3_freqAssignment[navsys][svid][slaveCnt+1] = slaveFreqIndex;
                            }

                            idx += 5;
                        } else if ((blockTypeSlave & 3) == 0) {
                            /* Slave Long */
                            uint32_t BF1      = U4(p+idx);
                            uint16_t prLsbRel = U2(p+idx + 4);
                            uint8_t  BF3      = U1(p+idx + 6);

                            uint32_t cmc      = (BF1 >> 2) & 0x3fffff;
                            uint32_t lti4     = (BF1 >> 24) & 0xf;
                            uint32_t prMsbRel = (BF1 >> 28) & 0x7;
                            uint32_t CN0      = BF3 & 0x3f;
                            uint32_t lockTime = Meas3_LTItoPLLTime[lti4];

                            if ((slaveFreqIndex = meas3_sig2idx(navsys, slaveSignalIndex, raw->opt, &codeSlave, sigTable)) >= 0 && satNo > 0) {
                                freqSlave = code2freq(Meas3_NavSys[navsys], codeSlave, glofnc);
                                raw->obuf.data[n].P[slaveFreqIndex] = raw->obuf.data[n].P[masterFreqIndex] + (prMsbRel * 65536 + prLsbRel) * .001 - 262.144;

                                if (cmc != 0)
                                    raw->obuf.data[n].L[slaveFreqIndex] = raw->obuf.data[n].P[slaveFreqIndex] / (CLIGHT/freqSlave) - 2097.152 + cmc * 0.001;

                                if ((navsys == 0) && (codeSlave == CODE_L1W || codeSlave == CODE_L2W))
                                    raw->obuf.data[n].SNR[slaveFreqIndex] = (uint16_t)(CN0 / SNR_UNIT + 0.5);
                                else
                                    raw->obuf.data[n].SNR[slaveFreqIndex] = (uint16_t)((CN0 + 10.0) / SNR_UNIT + 0.5);

                                raw->obuf.data[n].code[slaveFreqIndex] = codeSlave;
                                raw->obuf.data[n].LLI[slaveFreqIndex] = (lockTime < raw->lockt[satNo-1][slaveFreqIndex] ? LLI_SLIP : 0) | (lti4 == 0 ? LLI_HALFC : 0);
                                raw->lockt[satNo-1][slaveFreqIndex] = lockTime;
                                sbf->meas3_freqAssignment[navsys][svid][slaveCnt+1] = slaveFreqIndex;
                            }

                            idx += 7;
                        } else {
                            /* Slave Delta */
                            uint16_t BF1      = U2(p+idx);
                            uint8_t  dC      = U1(p+idx + 2);

                            uint32_t dPr     = (BF1 >> 2) & 0xfff;
                            uint32_t CN0      = BF1 >> 14;

                            if ((slaveFreqIndex = meas3_sig2idx(navsys, slaveSignalIndex, raw->opt, &codeSlave, sigTable)) >= 0 && satNo > 0) {
                                freqSlave = code2freq(Meas3_NavSys[navsys], codeSlave, glofnc);

                                obsd_t * masterReference = &(sbf->meas3_refEpoch.obsData[navsys][svid]);
                                obsd_t * slaveReference = &(sbf->meas3_refEpoch.obsData[navsys][svid]);
                                int masterRefFreqIdx = meas3_sig2idx(navsys, sbf->meas3_refEpoch.signalIdx[navsys][svid][0], raw->opt, &codeSlave, sigTable);
                                int slaveRefFreqIdx = meas3_sig2idx(navsys, sbf->meas3_refEpoch.signalIdx[navsys][svid][slaveCnt+1], raw->opt, &codeSlave, sigTable);

                                raw->obuf.data[n].L[slaveFreqIndex] = (slaveReference->L[slaveRefFreqIdx]
                                                                      + (raw->obuf.data[n].L[masterFreqIndex] - masterReference->L[masterRefFreqIdx]) * freqSlave / freqMaster - 0.128 + dC * 0.001);

                                raw->obuf.data[n].P[slaveFreqIndex] = (slaveReference->P[slaveRefFreqIdx] +
                                                                      (raw->obuf.data[n].L[slaveFreqIndex] - slaveReference->L[slaveRefFreqIdx]) * (CLIGHT/freqSlave)
                                                                      - 2.048 + dPr * 0.001);

                                raw->obuf.data[n].SNR[slaveFreqIndex] = (uint16_t)(slaveReference->SNR[slaveRefFreqIdx] + (-2.0 + CN0)/SNR_UNIT+0.5);

                                raw->obuf.data[n].code[slaveFreqIndex] = codeSlave;
                                raw->obuf.data[n].LLI[slaveFreqIndex] = slaveReference->LLI[slaveRefFreqIdx];
                                raw->lockt[satNo-1][slaveFreqIndex] = sbf->meas3_refEpoch.lockt[navsys][svid][slaveCnt+1];
                                sbf->meas3_freqAssignment[navsys][svid][slaveCnt+1] = slaveFreqIndex;
                            }

                            idx += 3;
                        };

                        /* keep reference measurement to decode delta measurements */
                        if (TOW % refEpochInterval == 0 && satNo > 0)
                        {
                            if ((slaveFreqIndex > NFREQ+NEXOBS) || (slaveFreqIndex < 0))
                                trace(2, "sbf meas3ranges index out of bounds: %d\n", slaveFreqIndex);
                            sbf->meas3_refEpoch.signalIdx[navsys][svid][slaveCnt + 1] = (uint8_t)slaveSignalIndex;

                            sbf->meas3_refEpoch.obsData[navsys][svid].P[slaveFreqIndex] = raw->obuf.data[n].P[slaveFreqIndex];  //TODO: save all parameters
                            sbf->meas3_refEpoch.obsData[navsys][svid].L[slaveFreqIndex] = raw->obuf.data[n].L[slaveFreqIndex];
                            sbf->meas3_refEpoch.obsData[navsys][svid].SNR[slaveFreqIndex] = raw->obuf.data[n].SNR[slaveFreqIndex];
                            sbf->meas3_refEpoch.obsData[navsys][svid].LLI[slaveFreqIndex] = raw->obuf.data[n].LLI[slaveFreqIndex];
                            sbf->meas3_refEpoch.lockt[navsys][svid][slaveFreqIndex] = raw->lockt[satNo-1][slaveFreqIndex];
                        }

                        if (raw->lockt[satNo-1][slaveFreqIndex] > sbf->meas3_refEpoch.lockt[navsys][svid][slaveFreqIndex])
                            sbf->meas3_refEpoch.lockt[navsys][svid][slaveFreqIndex] = raw->lockt[satNo-1][slaveFreqIndex];

                        slaveCnt++;
                        /* delete this bit of the mask */
                        slaveSignalMask ^= (1 << slaveSignalIndex);
                    }
                }
                n++;
                satCnt++;
            }
        }

    raw->obuf.n = n;

    return ret;
}

int32_t meas3_DopplerPrRate(raw_t* raw, uint32_t *offset)
{
    int32_t prRate;

    uint32_t value = U4(raw->buff + 16 + *offset);

    if ((value & 2) == 0)
    {
        /* 1 byte */
        prRate = (int32_t)((value & 0xff) >> 2);
        *offset += 1;
    }
    else if ((value & 6) == 2)
    {
        /* 2 bytes */
        prRate = (int32_t)((value & 0xffff) >> 3);
        *offset += 2;
    }
    else if ((value & 0xe) == 6) {
        /* 3 bytes */
        prRate = (int32_t)((value & 0xffffff) >> 4);
        *offset += 3;
    }
    else {
        /* 4 bytes */
        prRate = (int32_t)(value >> 4);
        *offset += 4;
    }

    if ((value & 1) == 1)
        prRate = -prRate;

    return prRate;
}

int decode_meas3Doppler(raw_t* raw)
{
    sbf_t *sbf = (sbf_t *)raw->rcv_data;
    int n;
    uint32_t offset = 0, ret = 0;
    int ant_sel = 0; /* antenna selection (0:main) */

    if (timediff(raw->time, sbf->current_time) != 0) {
        sbf->current_time = raw->time;
        ret = flushobuf(raw);
    }

    if (strstr(raw->opt, "-NO_MEAS3")) return ret;

    uint16_t flags = U2(raw->buff + 14);
    if      (strstr(raw->opt, "-AUX1")) ant_sel = 1;
    else if (strstr(raw->opt, "-AUX2")) ant_sel = 2;
    if ((flags & 0x7) != ant_sel) return ret;

    for (n = 0; n < raw->obuf.n && offset+16 < (uint32_t)raw->len; n++) {
        int navsys, sys, prn;
        int8_t masterFreqIndex, slaveFreqIndex;
        int32_t prRate = meas3_DopplerPrRate(raw, &offset);

        if (prRate == (int32_t)0x80000000 || prRate == (int32_t) -268435455)
            continue;

        sys = satsys(raw->obuf.data[n].sat, &prn);

        for (navsys = 0; navsys < 7; navsys++)
            if (Meas3_NavSys[navsys] == sys)
                break;
        if (navsys == 7)
            continue;

        int svid = prn - Meas3_SVIDBase[navsys];
        if (svid >= MEAS3_SAT_MAX) {
            // Give up on the remainder, sync with the buffer offset would be lost.
            trace(1, "SBF decode_meas3Doppler: svid=%d out of bounds %d\n", svid, MEAS3_SAT_MAX);
            return ret;
        }

        masterFreqIndex = sbf->meas3_freqAssignment[navsys][svid][0];

        double freqMaster = code2freq(sys, raw->obuf.data[n].code[masterFreqIndex], raw->obuf.data[n].freq - 7);

        raw->obuf.data[n].D[masterFreqIndex] = (float)(-(prRate + (int32_t)sbf->meas3_refEpoch.prRate[navsys][svid] * 64) * 0.001 / (CLIGHT/freqMaster));
        for (int i = 1; i<MEAS3_SIG_MAX; i++) {
            slaveFreqIndex = sbf->meas3_freqAssignment[navsys][svid][i];
            if (slaveFreqIndex < 0)
                break;
            prRate = meas3_DopplerPrRate(raw, &offset);
            double freqSlave = code2freq(sys, raw->obuf.data[n].code[slaveFreqIndex], raw->obuf.data[n].freq - 7);
            raw->obuf.data[n].D[slaveFreqIndex] = (float)((raw->obuf.data[n].D[masterFreqIndex] * (CLIGHT/freqMaster) * 1000 - prRate) * 0.001 / (CLIGHT/freqSlave));
        }
    }

    return ret;
}

int decode_meas3CN(raw_t* raw)
{
    sbf_t *sbf = (sbf_t *)raw->rcv_data;
    int n, ret = 0;
    uint32_t offset = 0;
    int ant_sel = 0; /* antenna selection (0:main) */

    if (timediff(raw->time, sbf->current_time) != 0) {
        sbf->current_time = raw->time;
        ret = flushobuf(raw);
    }

    if (strstr(raw->opt, "-NO_MEAS3")) return ret;

    uint16_t flags = U2(raw->buff + 14);
    if      (strstr(raw->opt, "-AUX1")) ant_sel = 1;
    else if (strstr(raw->opt, "-AUX2")) ant_sel = 2;
    if ((flags & 0x7) != ant_sel) return ret;

    for (n = 0; n < raw->obuf.n && offset/2+16 < (uint32_t)raw->len; n++) {
        int navsys, sys, prn;
        int8_t masterFreqIndex, slaveFreqIndex;
        sys = satsys(raw->obuf.data[n].sat, &prn);

        for (navsys = 0; navsys < 7; navsys++)
            if (Meas3_NavSys[navsys] == sys)
                break;
        if (navsys == 7)
            continue;

        int svid = prn - Meas3_SVIDBase[navsys];
        if (svid >= MEAS3_SAT_MAX) {
            // Give up correcting the remainder.
            trace(1, "SBF decode_meas3CN: svid=%d out of bounds %d\n", svid, MEAS3_SAT_MAX);
            return ret;
        }

        masterFreqIndex = sbf->meas3_freqAssignment[navsys][svid][0];

        uint8_t mc = (U1(raw->buff + 16 + offset / 2) >> ((offset % 2) * 4)) & 0xf;
        raw->obuf.data[n].SNR[masterFreqIndex] += (mc * 0.0625 - 0.5) / SNR_UNIT + 0.5;
        offset++;
        for (int i = 1; i<MEAS3_SIG_MAX; i++) {
            slaveFreqIndex = sbf->meas3_freqAssignment[navsys][svid][i];
            if (slaveFreqIndex < 0)
                break;
            raw->obuf.data[n].SNR[slaveFreqIndex] += (((U1(raw->buff + 16 + offset / 2) >> ((offset % 2) * 4)) & 0xf) * .0625F - 0.5F)/SNR_UNIT + 0.5;
            offset++;
        }
    }

    return ret;
}

/* Navigation Page Blocks */

/* decode ION/UTC parameters -------------------------------------------------*/
static int decode_gpsionutc(raw_t *raw, int sat)
{
    double ion[8], utc[8];

    if (!decode_frame(raw->subfrm[sat-1], NULL, NULL, ion, utc)) return 0;

    adj_utcweek(raw->time, &utc[3]);
    adj_utcweek(raw->time, &utc[5]);
    int sys = satsys(sat, NULL);
    if (sys == SYS_QZS) {
        matcpy(raw->nav.ion_qzs, ion, 8, 1);
        matcpy(raw->nav.utc_qzs, utc, 8, 1);
    } else {
        matcpy(raw->nav.ion_gps, ion, 8, 1);
        matcpy(raw->nav.utc_gps, utc, 8, 1);
    }
    return 9;
}


/* Decode SBF raw nav message (raw navigation data) --------------------------*/
static int decode_gpsrawcanav(raw_t *raw, int sys){

    /* NOTE. This function works quite well but it somestimes fails in line:
     * if (resp>5 || resp<=0){
     * To debug the problem an understanding of the whole RTK code is needed
     */

    trace(3, "SBF decode_gpsrawcanav: len=%d\n", raw->len);

    if (raw->len < 60) {
        trace(2, "SBF decode_gpsrawcanav block length error: len=%d\n", raw->len);
        return -1;
    }

    /* Get GPS satellite number */
    int svid = U1(raw->buff+14);
    int sat = svid2sat(svid);
    int prn;
    if (!sat || satsys(sat, &prn) != sys) {
        trace(2, "sbf rawca svid error: sys=%d svid=%d\n", sys, svid);
        return -1;
    }

    if (!U1(raw->buff+15)) {
        trace(3, "sbf rawca parity/crc error: sys=%d prn=%d\n", sys, prn);
        return 0;
    }

    if (raw->outtype) {
        if (sys == SYS_GPS)
            sprintf(raw->msgtype, "SBF GPS Raw Navigation Data (PRN=%d)", prn);
        if (sys == SYS_QZS)
            sprintf(raw->msgtype, "SBF QZSS Raw Navigation Data (PRN=%d)", prn);
    }

    /* Clean up subframe from Septentrio. This is a little bit of work because
     * Septentrio Rx add some parity bits to this message.
     * We have to throw away the reserved bits as well as the parity bits.
     */

    /*   | 2bits |         24bits        |  6bits  |       <- SBF 32-bit word
         ------------------------------------------
                  | byte1 | bite2 | byte3 |                 <- sat nav message
     */
    uint8_t _buf[30] = {0};
    for (int i=0; i<10; i++) { /* 24 x 10 bits w/o parity */
        setbitu(_buf, 24*i, 24, U4(raw->buff+20+4*i)>>6);
    }

    /* Now that we have a classic subframe we call the generic function */
    int id = getbitu(_buf, 43, 3); /* get subframe id */
    if (id<1 || id>5) {
        trace(2, "sbf rawca subframe id error: sys=%d prn=%d id=%d\n", sys, prn, id);
        return -1;
    }

    memcpy(raw->subfrm[sat-1]+(id-1)*30, _buf, 30);

    if (id == 3) {
        eph_t eph = {0};
        if (!decode_frame(raw->subfrm[sat-1], &eph, NULL, NULL, NULL))
            return 0;

        if (!strstr(raw->opt, "-EPHALL")) {
            if (eph.iode == raw->nav.eph[sat-1].iode &&
                eph.iodc == raw->nav.eph[sat-1].iodc &&
                timediff(eph.toe, raw->nav.eph[sat-1].toe) == 0.0 &&
                timediff(eph.toc, raw->nav.eph[sat-1].toc) == 0.0) return 0;
        }
        eph.sat = sat;
        raw->nav.eph[sat-1] = eph;
        raw->ephsat = sat;
        raw->ephset = 0;

        return 2;
    }
    if (id==4 || id==5) {
        int ret = decode_gpsionutc(raw, sat);
        memset(raw->subfrm[sat-1]+(id-1)*30, 0, 30);

        return ret;
    }

    trace(4, "SBF, decode_gpsrawcanav: sat=%2d\n", sat);
    return 0;
}

/* Decode SBF GPS raw cnav message (raw navigation data) ----------------------*/
static int decode_gpsrawcnav(raw_t *raw, int sys) {
  trace(3, "SBF decode_gpsrawcnav: len=%d\n", raw->len);

  if (raw->len < 60) {
    trace(2, "SBF decode_gpsrawcnav block length error: len=%d\n", raw->len);
    return -1;
  }

  /* get GPS satellite number */
  int svid = U1(raw->buff + 14);
  int sat = svid2sat(svid);
  int prn;
  if (!sat || satsys(sat, &prn) != sys) {
    trace(2, "sbf rawcnav svid error: sys=%d svid=%d\n", sys, svid);
    return -1;
  }

  if (!U1(raw->buff + 15)) {
    trace(3, "sbf rawcnav parity/crc error: sys=%d svid=%d\n", sys, svid);
    return 0;
  }

  uint8_t buff[40];
  for (int i = 0; i < 10; i++) {
    setbitu(buff, 32 * i, 32, U4(raw->buff + 20 + 4 * i)); /* 300 bits */
  }

  /* Check the preamble */
  int offset = 0;
  uint8_t preamble = getbitu(buff, offset, 8);
  if (preamble != 0x8b) {
    trace(2, "sbf rawcnav unexpected preamble %02x\n", preamble);
    return -1;
  }
  offset += 8;
  uint8_t prn2 = getbitu(buff, offset, 6);
  offset += 6;
  uint8_t type = getbitu(buff, offset, 6);
  offset += 8;
  uint32_t tow = getbitu(buff, offset, 17);
  offset += 17;
  uint8_t alert = getbitu(buff, offset, 1);
  offset += 1;

  switch (type) {
    case 10: // Ephemeris 1
    case 11: // Ephemeris 2
    case 12: // Reduced Almanac
    case 13: // Clock Differential Correction
    case 14: // Ephemeris Differential Correction
    case 15: // Text
    case 30: // Clock, IONO & Group Delay
    case 31: // Clock & Reduced Almanac
    case 32: // Clock & EOP
    case 33: // Clock & UTC
    case 34: // Clock & Differential Correction
    case 35: // Clock & GGTO
    case 36: // Clock & Text
    case 37: // Clock & Midi Almanac
    case 40: // Integrity Support Message
      trace(3, "sbf rawcnav unsupported message type %d\n", type);
      break;
    default:
      trace(3, "sbf rawcnav unexpected message type %d\n", type);
      break;
  }

  // TODO

  return 0;
}

/* Decode SBF raw nav message (raw navigation data) for GLONASS---------------*/
static int decode_glorawcanav(raw_t *raw){
    if (raw->len < 32) {
        trace(2, "sbf glorawca length error: len=%d\n", raw->len);
        return -1;
    }
    int svid = U1(raw->buff+14);
    int sat=svid2sat(svid);
    int prn;
    if (!sat || satsys(sat,&prn)!=SYS_GLO) {
        trace(3, "sbf glorawca svid error: svid=%d\n", svid);
        return (svid==62) ? 0 : -1; /* svid=62: slot unknown */
    }
    if (!U1(raw->buff+15)) {
        trace(3, "sbf glorawca parity/crc error: prn=%d\n", prn);
        return 0;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype+strlen(raw->msgtype), " prn=%d", prn);
    }
    uint8_t buff[12];
    for (int i=0; i<3; i++) {
        setbitu(buff, 32*i, 32, U4(raw->buff+20+4*i)); /* 85 bits */
    }
    int m = getbitu(buff, 1, 4);
    if (m<1 || m>15) {
        trace(2, "sbf glorawca string number error: prn=%d m=%d\n", prn, m);
        return -1;
    }
    int32_t time = raw->time.time;
    int32_t dt = time - I4(raw->subfrm[sat - 1] + 152);
    if (dt > 30 || dt < -30) {
        memset(raw->subfrm[sat-1], 0, 40);
        memcpy(raw->subfrm[sat-1]+152, &time, sizeof(time));
    }
    memcpy(raw->subfrm[sat-1]+(m-1)*10, buff, 10);
    if (m != 4) return 0;

    geph_t geph = {0};
    geph.tof = raw->time;
    double utc[8] = {0};
    if (!decode_glostr(raw->subfrm[sat-1], &geph, utc)) return 0;

    matcpy(raw->nav.utc_glo, utc, 8, 1);

    if (geph.sat != sat) {
        trace(2, "sbf glorawca satellite error: sat=%d %d\n", sat, geph.sat);
        return -1;
    }
    geph.frq = (int)U1(raw->buff+18)-8;

    if (!strstr(raw->opt, "-EPHALL")) {
        if (geph.iode == raw->nav.geph[prn-1].iode &&
            timediff(geph.toe, raw->nav.geph[prn-1].toe) == 0.0) return 0;
    }
    raw->nav.geph[prn-1] = geph;
    raw->ephsat = sat;
    raw->ephset = 0;

    return 2;
}

/* decode SBF Galileo F/NAV navigation page ----------------------------------*/
static int decode_galrawfnav(raw_t *raw)
{
    if (strstr(raw->opt, "-GALINAV")) return 0;

    if (raw->len < 52) {
        trace(2, "sbf galrawfnav length error: len=%d\n", raw->len);
        return -1;
    }
    int svid = U1(raw->buff+14);
    int src  = U1(raw->buff+17) & 0x1f;

    int sat = svid2sat(svid);
    int prn;
    if (!sat || satsys(sat, &prn) != SYS_GAL) {
        trace(2, "sbf galrawfnav svid error: svid=%d src=%d\n", svid, src);
        return -1;
    }
    if (!U1(raw->buff+15)) {
        trace(3, "sbf galrawfnav parity/crc error: prn=%d src=%d\n", prn, src);
        return 0;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype+strlen(raw->msgtype), " prn=%d src=%d", prn, src);
    }
    if (src!=20 && src!=22) { /* E5a or E5 AltBOC */
        trace(2, "sbf galrawfnav source error: prn=%d src=%d\n", prn, src);
        return -1;
    }
    uint8_t buff[32];
    for (int i=0; i<8; i++) {
        setbitu(buff, 32*i, 32, U4(raw->buff+20+4*i)); /* 244 bits page */
    }
    int type = getbitu(buff, 0, 6); /* page type */

    if (type == 63) return 0; /* dummy page */
    if (type<1 || type>6) {
        trace(2, "sbf galrawfnav page type error: prn=%d type=%d\n", prn, type);
        return -1;
    }
    /* save 244 bits page (31 bytes * 6 page) */
    memcpy(raw->subfrm[sat-1]+128+(type-1)*31, buff, 31);

    if (type != 4) return 0;
    eph_t eph = {0};
    double ion[4] = {0}, utc[8] = {0};
    if (!decode_gal_fnav(raw->subfrm[sat-1]+128, &eph, ion, utc)) return 0;

    if (eph.sat != sat) {
        trace(2, "sbf galrawfnav satellite error: sat=%d %d\n", sat, eph.sat);
        return -1;
    }
    /* Data source: E5a */
    eph.code |= (1<<1)|(1<<8);

    adj_utcweek(raw->time, utc);
    matcpy(raw->nav.ion_gal, ion, 4, 1);
    matcpy(raw->nav.utc_gal, utc, 8, 1);

    if (!strstr(raw->opt, "-EPHALL")) {
        if (eph.iode == raw->nav.eph[sat-1+MAXSAT].iode &&
            timediff(eph.toe, raw->nav.eph[sat-1+MAXSAT].toe) == 0.0 &&
            timediff(eph.toc, raw->nav.eph[sat-1+MAXSAT].toc) == 0.0) return 0;
    }
    raw->nav.eph[sat-1+MAXSAT] = eph;
    raw->ephsat = sat;
    raw->ephset = 1; /* 1:F/NAV */

    return 2;
}

/* decode SBF raw nav message (raw navigation data) for galileo I/NAV---------*/
static int decode_galrawinav(raw_t *raw){
    eph_t eph = {0};
    double ion[4] = {0}, utc[8] = {0};
    uint8_t *p = raw->buff+14, buff[32], type, part1, part2, page1, page2;
    int i, j, svid, src, sat, prn;

    if (strstr(raw->opt, "-GALFNAV")) return 0;

    if (raw->len < 52) {
        trace(2, "sbf galrawinav length error: len=%d\n", raw->len);
        return -1;
    }
    svid = U1(p);
    src  = U1(p+3) & 0x1f;

    sat=svid2sat(svid);
    if (!sat || satsys(sat, &prn)!=SYS_GAL) {
        trace(2, "sbf galrawinav svid error: svid=%d src=%d\n", svid, src);
        return -1;
    }
    if (!U1(p+1)) {
        trace(3, "sbf galrawinav parity/crc error: prn=%d src=%d\n", prn, src);
        return 0;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype+strlen(raw->msgtype), " prn=%d src=%d", prn, src);
    }
    if (src!=17 && src!=21 && src!=22) { /* E1, E5b or E5 AltBOC */
        trace(2, "sbf galrawinav source error: prn=%d src=%d\n", prn, src);
        return -1;
    }
    for (i=0, p+=6; i<8; i++, p+=4) {
        setbitu(buff, 32*i, 32, U4(p)); /* 114(even) + 120(odd) bits */
    }
    part1 = getbitu(buff,   0, 1);
    page1 = getbitu(buff,   1, 1);
    part2 = getbitu(buff, 114, 1);
    page2 = getbitu(buff, 115, 1);

    if (part1!=0 || part2!=1) {
        trace(3, "sbf galrawinav part error: prn=%d even/odd=%d %d\n", prn, part1, part2);
        return -1;
    }
    if (page1==1 || page2==1) return 0; /* alert page */

    type = getbitu(buff, 2, 6); /* word type */

    if (type > 6) return 0;

    /* save 128 (112:even+16:odd) bits word (16 bytes * 7 word) */
    for (i=0, j=2; i<14; i++, j+=8) {
        raw->subfrm[sat-1][type*16+i] = getbitu(buff, j, 8);
    }
    for (i=14,j=116;i<16;i++,j+=8) {
        raw->subfrm[sat-1][type*16+i] = getbitu(buff, j, 8);
    }
    if (type != 5) return 0;
    if (!decode_gal_inav(raw->subfrm[sat-1], &eph, ion, utc)) return 0;

    if (eph.sat != sat) {
        trace(2, "sbf galrawinav satellite error: sat=%d %d\n", sat, eph.sat);
        return -1;
    }
    /* Data source: E1 or E5b */
    eph.code |= src==17 ? 1<<0 : (1<<2)|(1<<9);
    if (U1(raw->buff + 17)&0x20) eph.code |= 1<<2; // Mix of E1 and E5b

    adj_utcweek(raw->time, utc);
    matcpy(raw->nav.ion_gal, ion, 4, 1);
    matcpy(raw->nav.utc_gal, utc, 8, 1);

    if (!strstr(raw->opt, "-EPHALL")) {
        if (eph.iode == raw->nav.eph[sat-1].iode &&
            timediff(eph.toe,raw->nav.eph[sat-1].toe) == 0.0 &&
            timediff(eph.toc,raw->nav.eph[sat-1].toc) == 0.0) return 0;
    }
    raw->nav.eph[sat-1] = eph;
    raw->ephsat = sat;
    raw->ephset = 0; /* 0:I/NAV */

    return 2;
}
/* decode SBF raw nav message (raw navigation data) --------------------------*/
static int decode_georaw(raw_t *raw){
    uint8_t *p = raw->buff+14, buff[32];
    int i, svid, sat, prn;

    if (raw->len < 52) {
        trace(2, "sbf georawl1 length error: len=%d\n", raw->len);
        return -1;
    }

    svid = U1(p);
    sat = svid2sat(svid);
    if (!sat || satsys(sat, &prn)!=SYS_SBS) {
        trace(2, "sbf georawl1 svid error: svid=%d\n", svid);
        return -1;
    }
    if (!U1(p+1)) {
        trace(3, "sbf georaw parity/crc error: prn=%d err=%d\n", prn, U1(p+2));
        return 0;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype+strlen(raw->msgtype), " prn=%d", prn);
    }
    raw->sbsmsg.tow = (int)time2gpst(raw->time, &raw->sbsmsg.week);
    raw->sbsmsg.prn = prn;

    for (i=0; i<8; i++) {
        setbitu(buff, 32*i, 32, U4(p+6+4*i));
    }
    memcpy(raw->sbsmsg.msg, buff, 29); /* 226 bits w/o CRC */
    raw->sbsmsg.msg[28] &= 0xC0;

    return 3;
}

/* decode SBF raw nav message (raw navigation data) for COMPASS ---------*/
static int decode_cmpraw(raw_t *raw){
    eph_t eph = {0};
    double ion[8], utc[8];
    uint8_t *p = raw->buff+14, buff[40];
    int i, id, svid, sat, prn, pgn;

    if (raw->len < 52) {
        trace(2, "sbf cmpraw length error: len=%d\n", raw->len);
        return -1;
    }
    svid = U1(p);
    sat = svid2sat(svid);
    if (!sat || satsys(sat, &prn) != SYS_CMP) {
        trace(2, "sbf cmpraw svid error: svid=%d\n", svid);
        return -1;
    }
    if (!U1(p+1)) {
        trace(3, "sbf cmpraw parity/crc error: prn=%d\n", prn);
        return 0;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype+strlen(raw->msgtype), " prn=%d", prn);
    }
    for (i=0, p+=6; i<10; i++, p+=4) {
        setbitu(buff, 32*i, 32, U4(p));
    }
    id = getbitu(buff, 15, 3); /* subframe ID */
    if (id<1 || id>5) {
        trace(2, "sbf cmpraw id error: prn=%d id=%d\n", prn, id);
        return -1;
    }
    if (prn>=6 && prn<=58) { /* IGSO/MEO */
        memcpy(raw->subfrm[sat-1]+(id-1)*38, buff, 38);

        if (id == 3) {
            if (!decode_bds_d1(raw->subfrm[sat-1], &eph, NULL, NULL)) return 0;
        }
        else if (id == 5) {
            if (!decode_bds_d1(raw->subfrm[sat-1], NULL, ion, utc)) return 0;
            matcpy(raw->nav.ion_cmp, ion, 8, 1);
            matcpy(raw->nav.utc_cmp, utc, 8, 1);
            return 9;
        }
        else return 0;
    }
    else { /* GEO */
        pgn = getbitu(buff, 42, 4); /* page number */

        if (id==1 && pgn>=1 && pgn<=10) {
            memcpy(raw->subfrm[sat-1]+(pgn-1)*38, buff, 38);
            if (pgn != 10) return 0;
            if (!decode_bds_d2(raw->subfrm[sat-1], &eph, NULL)) return 0;
        }
        else if (id==1 && pgn==102) {
            memcpy(raw->subfrm[sat-1]+10*38, buff, 38);
            if (!decode_bds_d2(raw->subfrm[sat-1], NULL, utc)) return 0;
            matcpy(raw->nav.utc_cmp, utc, 8, 1);
            return 9;
        }
        else
            return 0;
    }
    if (!strstr(raw->opt, "-EPHALL")) {
        if (timediff(eph.toe, raw->nav.eph[sat-1].toe) == 0.0)
            return 0;
    }
    eph.sat = sat;
    raw->nav.eph[sat-1] = eph;
    raw->ephsat = sat;
    raw->ephset = 0;
    return 2;
}

#ifdef ENAIRN
/* decode SBF NavIC/IRNSS subframe -------------------------------------------*/
static int decode_navicraw(raw_t *raw)
{
    if (raw->len < 52) {
        trace(2, "sbf navicraw length error: len=%d\n", raw->len);
        return -1;
    }
    int svid = U1(raw->buff+14);
    int sat=svid2sat(svid), prn;
    if (!sat || satsys(sat,&prn)!=SYS_IRN) {
        trace(2, "sbf navicraw svid error: svid=%d\n", svid);
        return -1;
    }
    if (!U1(raw->buff+15)) {
        trace(3, "sbf navicraw parity/crc error: prn=%d err=%d\n", prn, U1(raw->buff+16));
        return 0;
    }
    if (raw->outtype) {
        sprintf(raw->msgtype+strlen(raw->msgtype), " prn=%d", prn);
    }
    uint8_t buff[40];
    for (int i=0; i<10; i++) {
        setbitu(buff, 32*i, 32, U4(raw->buff+20+i*4));
    }
    int id = getbitu(buff, 27, 2); /* subframe ID (0-3) */

    memcpy(raw->subfrm[sat-1]+id*37, buff, 37);

    if (id==1) { /* subframe 2 */
        eph_t eph = {0};
        if (!decode_irn_nav(raw->subfrm[sat-1], &eph, NULL, NULL)) return 0;

        if (!strstr(raw->opt, "-EPHALL")) {
            if (eph.iode==raw->nav.eph[sat-1].iode &&
                timediff(eph.toe, raw->nav.eph[sat-1].toe)==0.0) {
                return 0;
            }
        }
        eph.sat = sat;
        raw->nav.eph[sat-1] = eph;
        raw->ephsat = sat;
        raw->ephset = 0;
        return 2;
    }
    else if (id==2 || id==3) { /* subframe 3 or 4 */
        double ion[8], utc[9];
        int ret = 0;
        if (decode_irn_nav(raw->subfrm[sat-1], NULL, ion, NULL)) {
            matcpy(raw->nav.ion_irn, ion, 8, 1);
            ret = 9;
        }
        if (decode_irn_nav(raw->subfrm[sat-1], NULL, NULL, utc)) {
            adj_utcweek(raw->time, utc);
            matcpy(raw->nav.utc_irn, utc, 9, 1);
            ret = 9;
        }
        memset(raw->subfrm[sat-1]+id*37, 0, 37);
        return ret;
    }
    return 0;
}

#ifdef TESTING
/* decode SBF lnav message for NavIC (navigation data) --------------------------*/
static int decode_naviclnav(raw_t *raw)
{
    uint8_t *p = raw->buff+8;  /* points at TOW location */
    eph_t eph = {0};
    uint32_t tocs;
    uint8_t prn, sat, iodec;

    trace(4, "SBF decode_naviclnav: len=%d\n", raw->len);

    if (raw->len < 148) {
        trace(2, "SBF decode_naviclnav frame length error: len=%d\n", raw->len);
        return -1;
    }

    prn = U1(p+6);
    sat = satno(SYS_IRN, prn);

    if (sat == 0) return -1;

    if (!(prn>=1 && prn<=37)){
        trace(2, "SBF decode_naviclnav prn error: sat=%d\n", prn);
        return -1;
    }

    eph.week   = U2(p +   4);
    iodec      = U1(p +   7);
    eph.toes   = U4(p +   8);
    eph.A      = pow(R8(p +  12), 2);
    eph.deln   = R4(p +  20) * PI;
    eph.M0     = R8(p +  24) * PI;
    eph.e      = R8(p +  32);
    eph.omg    = R8(p +  40) * PI;
    eph.OMG0   = R8(p +  48) * PI;
    eph.OMGd   = R8(p +  56) * PI;
    eph.i0     = R8(p +  64) * PI;
    eph.idot   = R4(p +  72) * PI;
    eph.cis    = R4(p +  76);
    eph.cic    = R4(p +  80);
    eph.crs    = R4(p +  84);
    eph.crc    = R4(p +  88);
    eph.cus    = R4(p +  92);
    eph.cuc    = R4(p +  96);
    tocs       = U4(p + 100);
    eph.f2     = R4(p + 104);
    eph.f1     = R4(p + 108);
    eph.f0     = R8(p + 112);
    eph.tgd[0] = R4(p + 120);
    eph.flag   = U1(p + 124);
    eph.sva    = U1(p + 125);
    eph.svh    = U1(p + 140);

    eph.code   = 0;
    eph.fit    = 0;

    eph.iodc = iodec;
    eph.iode = iodec;

    eph.toe = gpst2time(eph.week, eph.toes);
    eph.toc = gpst2time(eph.week, tocs);
    eph.ttr = raw->time;

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF NavIC Decoded L-Navigation Data (PRN=%d, IODE=%d, IODC=%d, TOES=%6.0f )", prn, eph.iode, eph.iodc, eph.toes);
    }

    if (!strstr(raw->opt, "-EPHALL")) {
        if ((eph.iode == raw->nav.eph[sat-1].iode) &&
            (eph.iodc == raw->nav.eph[sat-1].iodc)) return 0;
    }

    eph.sat = sat;
    raw->nav.eph[sat-1] = eph;
    raw->ephsat = sat;
    raw->ephset = 0;

    return 2;
}
#endif
#endif



/* GPS Decoded Message Block */

/* decode SBF nav message for GPS (navigation data) --------------------------*/
static int decode_gpsnav(raw_t *raw)
{
    trace(4, "SBF decode_gpsnav: len=%d\n", raw->len);

    if (raw->len < 140) {
        trace(2, "SBF decode_gpsnav frame length error: len=%d\n", raw->len);
        return -1;
    }

    uint8_t prn = U1(raw->buff + 14);
    uint8_t sat = satno(SYS_GPS, prn);

    if (sat == 0) return -1;

    if (!(prn >= 1 && prn <= 37)) {
        trace(2, "SBF decode_gpsnav prn error: sat=%d\n", prn);
        return -1;
    }

    uint16_t week = U2(raw->buff + 12); /* WN */
    /* byte 15: reserved */
    /* byte 16-17: WN (modulo 1024) */
    eph_t eph = {0};
    eph.code = U1(raw->buff + 18);
    eph.sva = U1(raw->buff + 19); /* URA */
    eph.svh = U1(raw->buff + 20);
    eph.flag = U1(raw->buff + 21);
    eph.iodc = U2(raw->buff + 22);
    eph.iode = U1(raw->buff + 24);
    uint8_t iode3 = U1(raw->buff + 25);
    if (eph.iode != iode3)
        trace(2, "SBF decode_gpsnav: mismatch of IODE in subframe 2 and 3: iode2=%d iode3=%d\n",
              eph.iode, iode3);
    eph.fit = U1(raw->buff + 26) ? 0 : 4;
    /* byte 27: reserved */
    eph.tgd[0] = R4(raw->buff + 28);
    uint32_t tocs = U4(raw->buff + 32);
    eph.f2 = R4(raw->buff + 36);
    eph.f1 = R4(raw->buff + 40);
    eph.f0 = R4(raw->buff + 44);
    eph.crs = R4(raw->buff + 48);
    eph.deln = R4(raw->buff + 52) * PI;
    eph.M0 = R8(raw->buff + 56) * PI;
    eph.cuc = R4(raw->buff + 64);
    eph.e = R8(raw->buff + 68);
    eph.cus = R4(raw->buff + 76);
    eph.A = pow(R8(raw->buff + 80), 2);
    eph.toes = U4(raw->buff + 88);
    eph.cic = R4(raw->buff + 92);
    eph.OMG0 = R8(raw->buff + 96) * PI;
    eph.cis = R4(raw->buff + 104);
    eph.i0 = R8(raw->buff + 108) * PI;
    eph.crc = R4(raw->buff + 116);
    eph.omg = R8(raw->buff + 120) * PI;
    eph.OMGd = R4(raw->buff + 128) * PI;
    eph.idot = R4(raw->buff + 132) * PI;
    uint16_t week_toc = adjust_WN10(week, U2(raw->buff + 136)); /* WNt_oc, modulo 1024 */
    uint16_t week_toe = adjust_WN10(week, U2(raw->buff + 138)); /* WNt_oe, modulo 1024*/

    eph.week = adjgpsweek(week);
    eph.toe = gpst2time(adjgpsweek(week_toe), eph.toes);
    eph.toc = gpst2time(adjgpsweek(week_toc), tocs);
    eph.ttr = raw->time;

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF GPS Decoded Navigation Data (PRN=%d, IODE=%d, IODC=%d, TOES=%6.0f )", prn, eph.iode, eph.iodc, eph.toes);
    }

    if (!strstr(raw->opt, "-EPHALL")) {
        if ((eph.iode == raw->nav.eph[sat-1].iode) &&
            (eph.iodc == raw->nav.eph[sat-1].iodc)) return 0;
    }

    eph.sat = sat;
    raw->nav.eph[sat-1] = eph;
    raw->ephsat = sat;
    raw->ephset = 0;

    return 2;
}

/* decode SBF gpsalm --------------------------------------------------------*/
static int decode_gpsalm(raw_t *raw)
{
    uint8_t *p = raw->buff+8;                 /* points at TOW location */
    alm_t alm;
    uint16_t week;

    trace(4,"SBF decode_gpsalm: len=%d\n", raw->len);

    if (raw->len < 60)
    {
        trace(1, "SBF decode_gpsalm: Block too short\n");
        return -1;
    }

    week      = U2(p + 4);
    alm.sat   = satno(SYS_GPS, U1(p + 6));
    /* byte 7: reserved */
    alm.e     = R4(p + 8);
    alm.toas  = U4(p + 12);
    alm.i0    = R4(p + 16);
    alm.OMGd  = R4(p + 20);
    alm.A     = pow(R4(p + 24),2);
    alm.OMG0  = R4(p + 28);
    alm.omg   = R4(p + 32);
    alm.M0    = R4(p + 36);
    alm.f1    = R4(p + 40);
    alm.f0    = R4(p + 44);
    alm.week  = adjust_WN8(week, U1(p + 48));
    alm.svconf= U1(p + 49);
    alm.svh   = U1(p + 50);  /* 8 bit health */
    /* byte 51: health summary on 6 bits (from subframe 4, page 25 and sub-frame 5 page 25) */

    if (alm.sat == 0) return -1;

    alm.toa = gpst2time(alm.week, alm.toas);

    raw->nav.alm[alm.sat-1] = alm;

    if (raw->outtype) {
        sprintf(raw->msgtype,"SBF GPS Almanach (PRN=%d)", U1(p + 6));
    }

    return 9;
}

/* decode SBF gpsion --------------------------------------------------------*/
static int decode_gpsion(raw_t *raw){
    uint8_t *p = raw->buff+8;            /* points at TOW location */

    trace(4,"SBF decode_gpsion: len=%d\n", raw->len);

    if (raw->len < 48)
    {
        trace(1,"SBF decode_gpsion: Block too short\n");
        return -1;
    }

    raw->nav.ion_gps[0] = R4(p + 8);  /* alpha_0 */
    raw->nav.ion_gps[1] = R4(p + 12); /* alpha_1 */
    raw->nav.ion_gps[2] = R4(p + 16); /* alpha_2 */
    raw->nav.ion_gps[3] = R4(p + 20); /* alpha_3 */
    raw->nav.ion_gps[4] = R4(p + 24); /* beta_0 */
    raw->nav.ion_gps[5] = R4(p + 28); /* beta_1 */
    raw->nav.ion_gps[6] = R4(p + 32); /* beta_2 */
    raw->nav.ion_gps[7] = R4(p + 36); /* beta_3 */

    if (raw->outtype) {
        sprintf(raw->msgtype,"SBF GPS Ionospheric Data");
    }

    return 9;
}

/* decode SBF gpsutc --------------------------------------------------------*/
static int decode_gpsutc(raw_t *raw)
{
    uint8_t *p = raw->buff+8;                 /* points at TOW location */
    uint16_t week;

    trace(4,"SBF decode_gpsutc: len=%d\n", raw->len);

    if (raw->len < 37)
    {
        trace(1, "SBF decode_gpsutc: Block too short\n");
        return -1;
    }

    week = U2(p + 4);
    /* GPS delta-UTC parameters */
    raw->nav.utc_gps[1] = R4(p + 8);                                  /*        A1 */
    raw->nav.utc_gps[0] = R8(p + 12);                                 /*        A0 */
    raw->nav.utc_gps[2] = U4(p + 20);                                 /*       tot */
    raw->nav.utc_gps[3] = adjust_WN8(week, U1(p + 24));               /*       WNt */
    raw->nav.utc_gps[4] = I1(p + 25);                                 /*  DEL_t_LS */
    raw->nav.utc_gps[5] = adjust_WN8(week, U1(p + 26));               /*    WN_LSF */
    raw->nav.utc_gps[6] = U1(p + 27);                                 /*        DN */
    raw->nav.utc_gps[7] = I1(p + 28);                                 /* DEL_t_LSF */

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF GPS UTC Offsets");
    }

    return 9;
}

/* decode SBF cnav message for GPS (navigation data) --------------------------*/
static int decode_gpscnav(raw_t *raw, uint32_t sbf_id)
{
    uint8_t *p = raw->buff+8;  /* points at TOW location */
    eph_t eph = {0};
    uint32_t tocs, tops;
    uint8_t prn, sat;

    trace(4, "SBF decode_gpscnav: len=%d\n", raw->len);

    if (raw->len < 172) {
        trace(2, "SBF decode_gpscnav frame length error: len=%d\n", raw->len);
        return -1;
    }

    prn = U1(p+6);
    sat = satno(SYS_GPS, prn);

    if (sat == 0) return -1;

    if (!(prn>=1 && prn<=37)){
        trace(2, "SBF decode_gpscnav prn error: sat=%d\n", prn);
        return -1;
    }

    eph.code   = 0;
    eph.flag   = U1(p +   7);
    eph.week   = U2(p +   8);
    eph.svh    = U1(p +  10);
    eph.sva    = I1(p +  11); /* URA */
    tops       = U4(p +  12);
    eph.toes   = U4(p +  16);
    eph.A      = R8(p +  20);
    eph.Adot   = R8(p +  28);
    eph.deln   = R4(p +  36) * PI;
    eph.ndot   = R4(p +  40);
    eph.M0     = R8(p +  44) * PI;
    eph.e      = R8(p +  52);
    eph.omg    = R8(p +  60) * PI;
    eph.OMG0   = R8(p +  68) * PI;
    eph.OMGd   = R8(p +  76) * PI;
    eph.i0     = R8(p +  84) * PI;
    eph.idot   = R4(p +  92) * PI;
    eph.cis    = R4(p +  96);
    eph.cic    = R4(p + 100);
    eph.crs    = R4(p + 104);
    eph.crc    = R4(p + 108);
    eph.cus    = R4(p + 112);
    eph.cuc    = R4(p + 116);
    tocs       = U4(p + 120);
    /* byte 124: URA_NED0 */
    /* byte 125: URA_NED1 */
    /* byte 126: URA_NED2 */
    /* byte 127: WN_op */
    eph.f2     = R4(p + 128);
    eph.f1     = R4(p + 132);
    eph.f0     = R8(p + 136);
    if (sbf_id == ID_GPSCNAV) {
        if (R4(p + 144) != -2.e10)
            eph.tgd[0] = R4(p + 144);
        /* 144-147: ISC_L1CA */
        /* 148-151: ISC_L2C */
        /* 152-155: ISC_L5I5 */
        /* 156-159: ISC_L5Q5 */
    } else if (sbf_id == ID_GPSCNAV2) {
        /* 144-147: ISC_L1CP */
        /* 148-151: ISC_L1CD */
        /* 152-155: ISC_L1CA */
        /* 156-159: ISC_L2C */
        /* 160-163: ISC_L5Q5 */
        /* 164-167: ISC_L5Q5 */
    }
    eph.fit    = 0;

    eph.iodc = tops;  // IODC/IODE is not present in CNAV, use t_op instead
    eph.iode = tops;

    eph.toe = gpst2time(eph.week, eph.toes);
    eph.toc = gpst2time(eph.week, tocs);
    eph.ttr = raw->time;

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF GPS Decoded C-Navigation Data (PRN=%d, IODE=%d, IODC=%d, TOES=%6.0f )", prn, eph.iode, eph.iodc, eph.toes);
    }

    if (!strstr(raw->opt, "-EPHALL")) {
        if ((eph.iode == raw->nav.eph[sat-1].iode) &&
            (eph.iodc == raw->nav.eph[sat-1].iodc)) return 0;
    }

    eph.sat = sat;
    raw->nav.eph[sat-1] = eph;
    raw->ephsat = sat;
    raw->ephset = 0;

    return 2;
}


/* GLONASS Decoded Message Blocks */

/* decode SBF nav message for glonass (navigation data) ----------------------*/
static int decode_glonav(raw_t *raw){

    uint8_t *p = raw->buff+8;                 /* points at TOW location */
    geph_t eph = {0};
    int prn, sat;
    uint16_t week_toes;
    uint32_t toes;

    trace(4, "SBF decode_glonav: len=%d\n", raw->len);

    if (raw->len < 96) {
        trace(2, "SBF decode_glonav frame length error: len=%d\n", raw->len);
        return -1;
    }
    prn = U1(p+6)-37;
    sat = satno(SYS_GLO, prn);

    if (sat == 0) return -1;

    if (!(prn>=1 && prn<=24)){
        trace(2, "SBF decode_glonav prn error: sat=%d\n", prn);
        return -1;
    }

    double tow = U4(p +  0) * 0.001;
    uint16_t week = U2(p +  4);
    eph.frq    = U1(p +  7) - 8;
    eph.pos[0] = R8(p +  8) * 1000;
    eph.pos[1] = R8(p +  16) * 1000;
    eph.pos[2] = R8(p +  24) * 1000;
    eph.vel[0] = R4(p +  32) * 1000.0;
    eph.vel[1] = R4(p +  36) * 1000.0;
    eph.vel[2] = R4(p +  40) * 1000.0;
    eph.acc[0] = R4(p +  44) * 1000.0;
    eph.acc[1] = R4(p +  48) * 1000.0;
    eph.acc[2] = R4(p +  52) * 1000.0;
    eph.gamn   = R4(p +  56);
    eph.taun   = R4(p +  60);
    eph.dtaun  = R4(p +  64);
    toes       = U4(p +  68);
    week_toes  = adjust_WN10(week, U2(p +  72));  /* WN_toe modulo 1024 */
    /* byte 74: P1, time interval between adjacent values of t_b */
    /* byte 75: P2, 1-bit odd/eent flag of t_b */
    eph.age    = U1(p +  76);
    eph.svh    = U1(p +  77) >> 2;  /* 3-bit health flag, satellite unhealthy if MSB set */
    eph.iode   = U2(p +  78);
    /* byte 80:    M,   2-bit GLONASS-M satellite identifier (01, otherwise 00) */
    /* byte 81:    P,   2-bit mode of computation of time parameters */
    /* byte 82:    l,   1-bit health flag, 0=healthy, 1=unhealthy */
    /* byte 83:    P4,  1-bit ’updated’ flag of ephemeris data */
    /* byte 84-85: N_T, 1-bit current day number within 4-year interval */
    eph.sva    = U2(p +  86);

    eph.tof    = gpst2time(week, tow);
    eph.toe    = gpst2time(week_toes, toes);

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF GLONASS Decoded Navigation Data (PRN=%d, Frequency Number=%d IODE=%d, AGE=%d )", prn, eph.frq, eph.iode, eph.age);
    }

    if (!strstr(raw->opt, "-EPHALL")) {
        if (eph.iode == raw->nav.geph[prn-1].iode) return 0;
    }

    eph.sat = sat;
    raw->nav.geph[prn-1] = eph;
    raw->ephsat = sat;
    raw->ephset = 0;
    raw->nav.glo_fcn[prn-1] = eph.frq + 8; /* save frequency number */

    return 2;
}

/* decode SBF gloutc --------------------------------------------------------*/
static int decode_gloutc(raw_t *raw)
{
    uint8_t *p = raw->buff+8;                 /* points at TOW location */

    trace(4, "SBF decode_gloutc: len=%d\n", raw->len);

    if (raw->len<40)
    {
        trace(1, "SBF decode_gloutc: Block too short\n");
        return -1;
    }

    /* GPS delta-UTC parameters */

    /* byte 8: N_4: 4 year interval number, starting from 1996 */
    /* byte 9: KP: notification of leap second */
    /* byte 10-11: N: calendar day number within 4 year period */
    raw->nav.utc_glo[1] = R4(p + 12);                                 /*  tau_GPS */
    raw->nav.utc_glo[0] = R8(p + 16);                                 /*  tau_c */
    raw->nav.utc_glo[2] = U4(p + 24);                                 /*  B1 */
    raw->nav.utc_glo[3] = R4(p + 28);                                 /*  B2 */

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF GLONASS UTC Offsets");
    }

    return 9;
}


/* Galileo Decoded Message Blocks */

/* decode SBF nav message for Galileo (navigation data) --------------------------*/
static int decode_galnav(raw_t *raw){

    uint8_t *p = raw->buff+8;                 /* points at TOW location */
    eph_t eph = {0};
    uint32_t tocs;
    int prn, sat;
    uint16_t week_toe, week_toc;

    trace(4, "SBF decode_galnav: len=%d\n", raw->len);

    if (raw->len < 149) {
        trace(2, "SBF decode_galnav frame length error: len=%d\n", raw->len);
        return -1;
    }

    prn = U1(p+6)-70;
    sat = satno(SYS_GAL, prn);

    if (sat == 0) return -1;

    if (!(prn>=1 && prn<=36)){
        trace(2, "SBF decode_galnav prn error: sat=%d\n", prn);
        return -1;
    }

    double tow = U4(p +  0) * 0.001;
    eph.week   = U2(p +  4); /* GAL week number */
    /* byte 6: satellite id, see above */
    int code   = U1(p +  7); /* 2:INAV, 16:FNAV */
    if (code != 2 && code != 16) {
        trace(2, "SBF decode_galnav code error: sat=%d code=%d\n", prn, code);
        return -1;
    }
    eph.code   = code == 2 ? (1 << 0 ) | (1 << 2) | (1 << 9) : (1 << 1) | (1 << 8);
    eph.A      = pow(R8(p +  8), 2);
    eph.M0     = R8(p +  16) * PI;
    eph.e      = R8(p +  24);
    eph.i0     = R8(p +  32) * PI;
    eph.omg    = R8(p +  40) * PI;
    eph.OMG0   = R8(p +  48) * PI;
    eph.OMGd   = R4(p +  56) * PI;
    eph.idot   = R4(p +  60) * PI;
    eph.deln   = R4(p +  64) * PI;
    eph.cuc    = R4(p +  68);
    eph.cus    = R4(p +  72);
    eph.crc    = R4(p +  76);
    eph.crs    = R4(p +  80);
    eph.cic    = R4(p +  84);
    eph.cis    = R4(p +  88);
    eph.toes   = U4(p +  92);
    tocs       = U4(p +  96);
    eph.f2     = R4(p + 100);
    eph.f1     = R4(p + 104);
    eph.f0     = R8(p + 108);
    week_toe   = adjust_WN12(eph.week, U2(p + 116)); /* WNt_oc */
    week_toc   = adjust_WN12(eph.week, U2(p + 118));
    eph.iode   = U2(p + 120);
    eph.iodc   = 0;

    uint16_t health = U2(p + 122);
    int svh = 0;
    if (health & 0x001)
      svh |= (health >> 1) & 7; // L1B when code==2
    if (health & 0x010)
      svh |= ((health >> 5) & 7) << 6; // L5B when code==2
    if (health & 0x100)
      svh |= ((health >> 9) & 7) << 3; // L5A when code==16
    eph.svh = svh;
    /* byte 124: Health_PRS, reserved */
    uint8_t sva = U1(p + (code == 2 ? 126 : 125));
    eph.sva = sva == 255 ? 0 : sva;
    /* byte 127: SISA_L1AE6A, reserved */
    if (R4(p + 128) != -2.e10)
        eph.tgd[0] = R4(p + 128);
    if (R4(p + 132) != -2.e10)
        eph.tgd[1] = R4(p + 132);
    /* byte 136-139: BGD_L1AE6A, reserved */
    eph.fit    = 0;
    /* byte 140: CNAVenc: 2-bit C/NAV encryption status */

    eph.toe = gpst2time(week_toe, eph.toes);
    eph.toc = gpst2time(week_toc, tocs);
    eph.ttr = gpst2time(eph.week, tow);

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF Galileo Decoded Navigation Data (PRN=%d, IODE=%d, IODC=%d, TOES=%6.0f )", prn, eph.iode, eph.iodc, eph.toes);
    }

    if (!strstr(raw->opt, "-EPHALL")) {
        if (eph.iode == raw->nav.eph[sat-1].iode) return 0;
    }

    eph.sat = sat;
    raw->nav.eph[sat-1] = eph;
    raw->ephsat = sat;
    raw->ephset = 0;

    return 2;
}

/* decode SBF galalm --------------------------------------------------------*/
static int decode_galalm(raw_t *raw)
{
    uint8_t *p = raw->buff+8;                 /* points at TOW location */
    alm_t alm;
    uint16_t week, health;

    trace(4, "SBF decode_galalm: len=%d\n", raw->len);

    if (raw->len<61)
    {
        trace(1, "SBF decode_galalm: Block too short\n");
        return -1;
    }

    week      = U2(p + 4);
    alm.e     = R4(p + 8);
    alm.toas  = U4(p + 12);
    alm.i0    = R4(p + 16) + 0.3;  /* TODO: is this offset right? */
    alm.OMGd  = R4(p + 20);
    alm.A     = pow(R4(p + 24), 2);
    alm.OMG0  = R4(p + 28);
    alm.omg   = R4(p + 32);
    alm.M0    = R4(p + 36);
    alm.f1    = R4(p + 40);
    alm.f0    = R4(p + 44);
    alm.week  = week + U1(p + 48);
    alm.sat   = satno(SYS_GAL, U1(p + 49) - 70);
    health    = U2(p + 50);
    if (health & 0x01) health &= ~0x007;
    if (health & 0x08) health &= ~0x038;
    if (health & 0x40) health &= ~0x1B0;
    alm.svh   = health;

    alm.toa   = gpst2time(alm.week, alm.toas);
    alm.svconf= 0;
    /* byte 51: IODa, 4-bit Issue of Data for the almanac */

    if (alm.sat == 0) return -1;
    raw->nav.alm[alm.sat-1] = alm;

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF Galileo Almanach (PRN=%d)", U1(p + 49) - 70);
    }

    return 9;
}

/* decode SBF galion --------------------------------------------------------*/
static int decode_galion(raw_t *raw){
    uint8_t *p = raw->buff+8;            /* points at TOW location */

    trace(4, "SBF decode_galion: len=%d\n", raw->len);

    if (raw->len<29)
    {
        trace(1, "SBF decode_galion: Block too short\n");
        return -1;
    }

    raw->nav.ion_gal[0] = R4(p + 8);
    raw->nav.ion_gal[1] = R4(p + 12);
    raw->nav.ion_gal[2] = R4(p + 16);
    raw->nav.ion_gal[3] = 0;
    /* byte 20: Bit field containing the five ionospheric storm flags */

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF Galileo Ionospheric Data");
    }

    return 9;
}

/* decode SBF galutc --------------------------------------------------------*/
static int decode_galutc(raw_t *raw)
{
    uint8_t *p = raw->buff+8;                 /* points at TOW location */
    uint16_t week;

    trace(4, "SBF decode_galutc: len=%d\n", raw->len);

    if (raw->len<37)
    {
        trace(1, "SBF decode_galutc: Block too short\n");
        return -1;
    }

    /* Galileo delta-UTC parameters */
    week = U2(p + 4);
    raw->nav.utc_gal[1] = R4(p + 8);                                  /*     A1 */
    raw->nav.utc_gal[0] = R8(p + 12);                                 /*     A0 */
    raw->nav.utc_gal[2] = U4(p + 20);                                 /*    tot */
    raw->nav.utc_gal[3] = adjust_WN8(week, U1(p + 24));               /*     WN */
    raw->nav.utc_gal[4] = I1(p + 25);                                 /*  D_tls */
    raw->nav.utc_gal[5] = adjust_WN8(week, U1(p + 26));               /* WN_LSF */
    raw->nav.utc_gal[6] = U1(p + 27);                                 /*     DN */
    raw->nav.utc_gal[7] = I1(p + 28);                                 /* D_tlsf */

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF Galileo UTC Offsets");
    }

    return 9;
}

/* BeiDou Decoded Message Blocks */

/* decode SBF nav message for Compass/Beidou (navigation data) --------------------------*/
static int decode_cmpnav(raw_t *raw)
{
    trace(4, "SBF decode_cmpnav: len=%d\n", raw->len);

    if (raw->len < 140) {
        trace(2, "SBF decode_cmpnav frame length error: len=%d\n", raw->len);
        return -1;
    }

    uint8_t prn = U1(raw->buff + 14) - 140;
    int sat = satno(SYS_CMP, prn);

    if (sat == 0) return -1;

    if (!((prn >= 1) && (prn <= 32))) {
        trace(2, "SBF decode_cmpnav prn error: sat=%d\n", prn);
        return -1;
    }

    eph_t eph = {0};
    eph.code = 0;
    eph.sva = U1(raw->buff + 18);
    eph.svh = U1(raw->buff + 19);
    eph.iodc = U1(raw->buff + 20);
    eph.iode = U1(raw->buff + 21);
    /* byte 22, 23: reserved */
    eph.tgd[0] = R4(raw->buff + 24);
    if (R4(raw->buff + 28) != -2.e10) eph.tgd[1] = R4(raw->buff + 28);
    uint32_t tocs = U4(raw->buff + 32);  // BDT
    eph.f2 = R4(raw->buff + 36);
    eph.f1 = R4(raw->buff + 40);
    eph.f0 = R4(raw->buff + 44);
    eph.crs = R4(raw->buff + 48);
    eph.deln = R4(raw->buff + 52) * PI;
    eph.M0 = R8(raw->buff + 56) * PI;
    eph.cuc = R4(raw->buff + 64);
    eph.e = R8(raw->buff + 68);
    eph.cus = R4(raw->buff + 76);
    eph.A = pow(R8(raw->buff + 80), 2);
    eph.toes = U4(raw->buff + 88);  // BDT
    eph.cic = R4(raw->buff + 92);
    eph.OMG0 = R8(raw->buff + 96) * PI;
    eph.cis = R4(raw->buff + 104);
    eph.i0 = R8(raw->buff + 108) * PI;
    eph.crc = R4(raw->buff + 116);
    eph.omg = R8(raw->buff + 120) * PI;
    eph.OMGd = R4(raw->buff + 128) * PI;
    eph.idot = R4(raw->buff + 132) * PI;
    uint16_t week = U2(raw->buff + 12) - 1356; // GPS week to approx BDS week (14 sec diff)
    uint16_t week_toc = adjust_WN14(week, U2(raw->buff + 136)); /* WNt_oc */
    uint16_t week_toe = adjust_WN14(week, U2(raw->buff + 138)); /* WNt_oe */
    eph.week = week_toe;
    eph.fit = 0;

    eph.toe = bdt2gpst(bdt2time(week_toe, eph.toes)); // BDS to GPS
    eph.toc = bdt2gpst(bdt2time(week_toc, tocs));     // BDS to GPS
    eph.ttr = raw->time;

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF Compass Decoded Navigation Data (PRN=%d, IODE=%d, IODC=%d, TOES=%6.0f )", prn, eph.iode, eph.iodc, eph.toes);
    }

    if (!strstr(raw->opt, "-EPHALL")) {
        if (eph.iode == raw->nav.eph[sat - 1].iode) return 0;
    }

    eph.sat = sat;
    raw->nav.eph[sat - 1] = eph;
    raw->ephsat = sat;
    raw->ephset = 0;

    return 2;
}

/* decode SBF cnav2 message for BDS (navigation data) --------------------------*/
static int decode_cmpcnav2(raw_t *raw, uint32_t sbf_id)
{
    uint8_t *p = raw->buff+8;  /* points at TOW location */
    eph_t eph = {0};
    uint32_t tocs, tops;
    uint8_t prn, sat;

    trace(4, "SBF decode_cmpcnav2: len=%d\n", raw->len);

    if (raw->len < 164) {
        trace(2, "SBF decode_cmpcnav2 frame length error: len=%d\n", raw->len);
        return -1;
    }

    prn = U1(p+6);
    sat = satno(SYS_CMP, prn);

    if (sat == 0) return -1;

    if (!(prn>=1 && prn<=37)){
        trace(2, "SBF decode_cmpcnav2 prn error: sat=%d\n", prn);
        return -1;
    }

    eph.week   = U2(p +   4);
    eph.code   = 0;
    eph.flag   = U1(p +   7);
    eph.toes   = U4(p +   8);
    eph.A      = R8(p +  12);
    eph.Adot   = R8(p +  20);
    eph.deln   = R4(p +  28) * PI;
    eph.ndot   = R4(p +  32);
    eph.M0     = R8(p +  36) * PI;
    eph.e      = R8(p +  44);
    eph.omg    = R8(p +  52) * PI;
    eph.OMG0   = R8(p +  60) * PI;
    eph.OMGd   = R8(p +  68) * PI;
    eph.i0     = R8(p +  76) * PI;
    eph.idot   = R4(p +  84) * PI;
    eph.cis    = R4(p +  88);
    eph.cic    = R4(p +  92);
    eph.crs    = R4(p +  96);
    eph.crc    = R4(p + 100);
    eph.cus    = R4(p + 104);
    eph.cuc    = R4(p + 108);
    tocs       = U4(p + 112);
    eph.f2     = R4(p + 116);
    eph.f1     = R4(p + 120);
    eph.f0     = R8(p + 124);
    tops       = U4(p + 132);
    eph.sva    = U1(p + 139); /* to be checked */
    eph.svh    = U1(p + 140);

    if (sbf_id == ID_BDSCNAV1) {
        if (R4(p + 144) != -2.e10)
            eph.tgd[4] = R4(p + 144);  /* ISC_B1Cd */
        if (R4(p + 148) != -2.e10)
            eph.tgd[2] = R4(p + 148);  /* T_GDB1Cp */
        if (R4(p + 152) != -2.e10)
            eph.tgd[3] = R4(p + 152);  /* T_GDB2ap */
    } else if (sbf_id == ID_BDSCNAV2) {
        if (R4(p + 144) != -2.e10)
            eph.tgd[5] = R4(p + 144);  /* ISC_B2ad */
        if (R4(p + 148) != -2.e10)
            eph.tgd[3] = R4(p + 148);  /* T_GDB2ap */
        if (R4(p + 152) != -2.e10)
            eph.tgd[2] = R4(p + 152);  /* T_GDB1Cp */
    } else if (sbf_id == ID_BDSCNAV3) {
        if (R4(p + 144) != -2.e10)
            eph.tgd[1] = R4(p + 144);  /* T_GDB2bI */
    }

    eph.fit    = 0;

    eph.iodc = tops;  // IODC/IODE is not present in CNAV, use t_op instead
    eph.iode = tops;

    eph.toe = gpst2time(eph.week, eph.toes);
    eph.toc = gpst2time(eph.week, tocs);
    eph.ttr = raw->time;

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF BDS Decoded C-Navigation 2 Data (PRN=%d, IODE=%d, IODC=%d, TOES=%6.0f )", prn, eph.iode, eph.iodc, eph.toes);
    }

    if (!strstr(raw->opt, "-EPHALL")) {
        if ((eph.iode == raw->nav.eph[sat-1].iode) &&
            (eph.iodc == raw->nav.eph[sat-1].iodc)) return 0;
    }

    eph.sat = sat;
    raw->nav.eph[sat-1] = eph;
    raw->ephsat = sat;
    raw->ephset = 0;

    return 2;
}


/* decode SBF cmpalm --------------------------------------------------------*/
static int decode_cmpalm(raw_t *raw)
{
    uint8_t *p = raw->buff + 8;                 /* points at TOW location */
    alm_t alm;
    uint16_t week;

    trace(4, "SBF decode_cmpalm: len=%d\n", raw->len);

    if (raw->len < 60)
    {
        trace(1, "SBF decode_cmpalm: Block too short\n");
        return -1;
    }

    week      = U2(p + 4);
    alm.sat   = satno(SYS_CMP, U1(p + 6) - 140);
    alm.week  = adjust_WN8(week, U1(p + 7));
    alm.toas  = U4(p + 8);
    alm.A     = pow(R4(p + 12), 2);
    alm.e     = R4(p + 16);
    alm.omg   = R4(p + 20);
    alm.M0    = R4(p + 24);
    alm.OMG0  = R4(p + 28);
    alm.OMGd  = R4(p + 32);
    alm.i0    = R4(p + 36);
    alm.f0    = R4(p + 40);
    alm.f1    = R4(p + 44);
    alm.svh   = U2(p + 48);
    alm.svconf= 0;

    alm.toa   = gpst2time(alm.week, alm.toas);

    if (alm.sat == 0) return -1;

    raw->nav.alm[alm.sat-1] = alm;

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF Compass Almanach (PRN=%d)", U1(p + 6));
    }

    return 9;
}


/* decode SBF cmpion --------------------------------------------------------*/
static int decode_cmpion(raw_t *raw){
    uint8_t *p = raw->buff+8;            /* points at TOW location */

    trace(4, "SBF decode_cmpion: len=%d\n", raw->len);

    if (raw->len < 48)
    {
        trace(1, "SBF decode_cmpion: Block too short\n");
        return -1;
    }

    raw->nav.ion_cmp[0] = R4(p + 8);
    raw->nav.ion_cmp[1] = R4(p + 12);
    raw->nav.ion_cmp[2] = R4(p + 16);
    raw->nav.ion_cmp[3] = R4(p + 20);
    raw->nav.ion_cmp[4] = R4(p + 24);
    raw->nav.ion_cmp[5] = R4(p + 28);
    raw->nav.ion_cmp[6] = R4(p + 32);
    raw->nav.ion_cmp[7] = R4(p + 36);

    if (raw->outtype) {
        sprintf(raw->msgtype,"SBF Compass Ionospheric Data");
    }

    return 9;
}


/* decode SBF cmputc --------------------------------------------------------*/
static int decode_cmputc(raw_t *raw)
{
    uint8_t *p = raw->buff+8;                 /* points at TOW location */
    uint16_t week;

    trace(4, "SBF decode_cmputc: len=%d\n", raw->len);

    if (raw->len < 32)
    {
        trace(1, "SBF decode_cmputc: Block too short\n");
        return -1;
    }

    week = U2(p + 4);

    /* Compass delta-UTC parameters */
    raw->nav.utc_cmp[1] = R4(p + 8);                                  /*        A1 */
    raw->nav.utc_cmp[0] = R8(p + 12);                                 /*        A0 */
    raw->nav.utc_cmp[2] = 0;                                          /*       tot */
    raw->nav.utc_cmp[3] = 0;                                          /*       WNt */
    raw->nav.utc_cmp[4] = I1(p + 20);                                 /*     dt_LS */
    raw->nav.utc_cmp[5] = adjust_WN8(week, U1(p + 21));               /*    WN_LSF */
    raw->nav.utc_cmp[6] = U1(p + 22);                                 /*        DN */
    raw->nav.utc_cmp[7] = I1(p + 23);                                 /* DEL_t_LSF */

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF Compass UTC Offsets");
    }

    return 9;
}

/* QZSS Decoded Message Blocks */

/* decode SBF nav message for QZSS (navigation data) --------------------------*/
static int decode_qzssnav(raw_t *raw){
    trace(4, "SBF decode_qzssnav: len=%d\n", raw->len);

    if (raw->len < 140) {
        trace(2, "SBF decode_qzssnav frame length error: len=%d\n", raw->len);
        return -1;
    }

    int svid = U1(raw->buff + 14);
    int sat = svid2sat(svid);

    if (sat == 0) {
        trace(2, "SBF decode_qzssnav svid error: svid=%d prn=%d\n", svid, svid - 180);
        return -1;
    }

    uint16_t week = U2(raw->buff + 12); /* WN */
    eph_t eph = {0};
    eph.code = U1(raw->buff + 18);
    eph.sva = U1(raw->buff + 19);
    eph.svh = U1(raw->buff + 20);
    /* byte 21 L2 P data flag */
    eph.iodc = U2(raw->buff + 22);
    eph.iode = U1(raw->buff + 24);
    /* byte 25: IODE from frame 3 */
    eph.fit = U1(raw->buff + 26);
    /* byte 27: reserved */
    if (R4(raw->buff + 28) != -2.e10) eph.tgd[0] = R4(raw->buff + 28);
    double toc = U4(raw->buff + 32);
    eph.f2 = R4(raw->buff + 36);
    eph.f1 = R4(raw->buff + 40);
    eph.f0 = R4(raw->buff + 44);
    eph.crs = R4(raw->buff + 48);
    eph.deln = R4(raw->buff + 52) * PI;
    eph.M0 = R8(raw->buff + 56) * PI;
    eph.cuc = R4(raw->buff + 64);
    eph.e = R8(raw->buff + 68);
    eph.cus = R4(raw->buff + 76);
    eph.A = pow(R8(raw->buff + 80), 2);
    eph.toes = U4(raw->buff + 88);
    eph.cic = R4(raw->buff + 92);
    eph.OMG0 = R8(raw->buff + 96) * PI;
    eph.cis = R4(raw->buff + 104);
    eph.i0 = R8(raw->buff + 108) * PI;
    eph.crc = R4(raw->buff + 116);
    eph.omg = R8(raw->buff + 120) * PI;
    eph.OMGd = R4(raw->buff + 128) * PI;
    eph.idot = R4(raw->buff + 132) * PI;
    uint16_t week_oc = U2(raw->buff + 136); /* WNt_oc */
    uint16_t week_oe = U2(raw->buff + 138); /* WNt_oe l*/

    eph.week = adjust_WN10(week, week_oc);
    eph.toe = gpst2time(adjust_WN10(week, week_oe), eph.toes);
    eph.toc = gpst2time(eph.week, toc);
    eph.ttr = raw->time;

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF QZSS Decoded Navigation Data (PRN=%d, IODE=%d, IODC=%d, TOES=%6.0f )", svid - 180, eph.iode, eph.iodc, eph.toes);
    }

    if (!strstr(raw->opt, "-EPHALL")) {
        if (eph.iode == raw->nav.eph[sat-1].iode) return 0;
    }

    eph.sat = sat;
    raw->nav.eph[sat-1] = eph;
    raw->ephsat = sat;
    raw->ephset = 0;
    return 2;
}

#ifdef TESTING

/* decode SBF almanach message for QZSS --------------------------*/
static int decode_qzssalm(raw_t *raw)
{
    uint8_t *p = raw->buff+8;                 /* points at TOW location */
    alm_t alm;
    uint16_t week;

    trace(4, "SBF decode_cmpalm: len=%d\n", raw->len);

    if (raw->len < 60)
    {
        trace(1, "SBF decode_cmpalm: Block too short\n");
        return -1;
    }

    week      = U2(p + 4);
    alm.sat   = satno(SYS_QZS, U1(p + 6) - 180);
    /* byte 7: reserved */
    alm.e     = R4(p + 8);
    alm.toas  = U4(p + 12);
    alm.i0    = R4(p + 16);
    alm.OMGd  = R4(p + 20);
    alm.A     = pow(R4(p + 24), 2);
    alm.OMG0  = R4(p + 28);
    alm.omg   = R4(p + 32);
    alm.M0    = R4(p + 36);
    alm.f1    = R4(p + 40);
    alm.f0    = R4(p + 44);
    alm.week  = adjust_WN8(week, U1(p + 48));
    /* byte 49: reserved */
    alm.svh   = U1(p + 50);
    alm.svconf= 0;

    alm.toa   = gpst2time(alm.week, alm.toas);

    if (alm.sat == 0) return -1;

    raw->nav.alm[alm.sat-1] = alm;

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF QZSS Almanach (PRN=%d)", U1(p + 6));
    }

    return 9;
}
#endif /* TESTING */



/* decode SBF nav message for sbas (navigation data) ----------------------*/
static int decode_geonav(raw_t *raw)
{
    uint8_t *p = raw->buff+8;                 /* points at TOW location */
    seph_t eph = {0};
    int prn, sat;
    uint32_t tod;

    trace(4, "SBF decode_geonav: len=%d\n", raw->len);

    if (raw->len < 104) {
        trace(2, "SBF decode_geonav frame length error: len=%d\n", raw->len);
        return -1;
    }
    prn = U1(p+6);
    sat = satno(SYS_SBS, prn);

    if (!(prn>=120 && prn<=140)){
        trace(2, "SBF decode_geonav prn error: sat=%d\n", prn);
        return -1;
    }

    if (sat == 0) return -1;

    double tow = U4(p +  0) * 0.001;
    uint16_t week = U2(p +  4);
    /* byte 7: reserved */
    /* byte 8, 9: IODN */
    eph.sva    = U2(p + 10);
    tod        = U4(p + 12);
    eph.pos[0] = R8(p + 16);
    eph.pos[1] = R8(p + 24);
    eph.pos[2] = R8(p + 32);
    eph.vel[0] = R8(p + 40);
    eph.vel[1] = R8(p + 48);
    eph.vel[2] = R8(p + 56);
    eph.acc[0] = R8(p + 64);
    eph.acc[1] = R8(p + 72);
    eph.acc[2] = R8(p + 80);
    eph.af0    = R4(p + 88);
    eph.af1    = R4(p + 92);

    eph.tof    = gpst2time(week, tow);
    eph.t0     = adjday(eph.tof, tod);
    eph.svh    = eph.sva==15 ? 1 : 0;

    /* debug */
    trace(2, "sat=%2d, week=%d, tow=%f\n", sat, week, tow);

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF SBAS Decoded Navigation Data (PRN=%d, TOW=%lf, SVA=%d )", prn, tow, eph.sva);
    }

    if (!strstr(raw->opt, "-EPHALL")) {
        if (fabs(timediff(eph.t0, raw->nav.seph[prn-120].t0)) < 1.0 &&
            eph.sva == raw->nav.seph[prn-120].sva)
            return 0;
    }

    eph.sat = sat;
    raw->nav.seph[prn-120] = eph;
    raw->ephsat = eph.sat;
    raw->ephset = 0;

    return 2;
}

#if 1 /* UNUSED */

/* type 2-5,0: fast corrections ---------------------------------------*/
static int decode_sbsfast(raw_t *raw)
{
    int i, j;
    int prn, sat;
    uint8_t sbLength, sbCount, iodf, type;
    double prc_old, dt;
    gtime_t t0_old;
    uint8_t *p = raw->buff+8;

    trace(4, "SBF decode_sbsfast: len=%d\n", raw->len);

    if (raw->len < 20)
    {
        trace(1, "SBF decode_sbsfast: Block too short\n");
        return -1;
    }

    /* get satellite number */
    prn = U1(p+6);
    if (prn < 120) return -1;
    if (prn > 140) return -1;

    sat = satno(SYS_SBS, prn);
    if (sat == 0) return -1;

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF SBAS Ionosphere Fast Correction from PRN=%d", prn);
    }

    double tow = U4(p+0) * 0.001;
    uint16_t week = U2(p+4);

    type = U1(p+7);
    if (raw->nav.sbssat.iodp != U1(p+8)) return 0;
    iodf = U1(p+9);
    sbCount = U1(p+10);
    sbLength = U1(p+11);

    if (type > 5 || type == 1) return -1;

    for (i=0; i<sbCount; i++) {
        j = U1(p+12+i*sbLength+1);
        if (j >= raw->nav.sbssat.nsat) break;
        t0_old = raw->nav.sbssat.sat[j].fcorr.t0;
        prc_old = raw->nav.sbssat.sat[j].fcorr.prc;

        raw->nav.sbssat.sat[j].fcorr.t0 = gpst2time(week, tow);
        raw->nav.sbssat.sat[j].fcorr.udre = U1(p+12+i*sbLength+1);
        raw->nav.sbssat.sat[j].fcorr.prc = R4(p+12+i*sbLength+4);

        dt = timediff(raw->nav.sbssat.sat[j].fcorr.t0, t0_old);
        if (t0_old.time==0 || dt<=0.0 || 18.0<dt || raw->nav.sbssat.sat[j].fcorr.ai==0) {
            raw->nav.sbssat.sat[j].fcorr.rrc = 0.0;
            raw->nav.sbssat.sat[j].fcorr.dt = 0.0;
        }
        else {
            raw->nav.sbssat.sat[j].fcorr.rrc = (raw->nav.sbssat.sat[j].fcorr.prc-prc_old)/dt;
            raw->nav.sbssat.sat[j].fcorr.dt = dt;
        }
        raw->nav.sbssat.sat[j].fcorr.iodf = iodf;
    }
    trace(5, "SBF decode_sbsfast: type=%d iodf=%d\n", U1(p+9), iodf);
    return 3;
}

/* decode type 1: prn masks --------------------------------------------------*/
static int decode_sbsprnmask(raw_t *raw)
{
    int i, n, sat, prn;
    uint8_t *p = raw->buff+8;

    trace(4, "SBF decode_sbsprnmask:\n");

    if (raw->len < 17)
    {
        trace(1, "SBF decode_sbsprnmask: Block too short\n");
        return -1;
    }

    /* get satellite number */
    prn = U1(p+6);
    if (prn < 120) return -1;
    if (prn > 139) return -1;

    raw->nav.sbssat.iodp = U1(p+7);
    raw->nav.sbssat.nsat = U1(p+8);

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF SBAS PRN Mask from PRN=%d", prn);
    }

    for (n=0; n<raw->nav.sbssat.nsat && n<MAXSAT; n++) {
       i = U1(p+9+n);
       if      (i<= 37) sat = satno(SYS_GPS, i);    /*   0- 37: GPS */
       else if (i<= 61) sat = satno(SYS_GLO, i-37); /*  38- 61: GLONASSs */
       else if (i<=119) sat = 0;                    /*  62-119: future GNSS */
       else if (i<=138) sat = satno(SYS_SBS, i);    /* 120-138: geo/waas */
       else if (i<=182) sat = 0;                    /* 139-182: reserved */
       else if (i<=192) sat = satno(SYS_SBS, i+10); /* 183-192: QZSS ref [2] */
       else if (i<=202) sat = satno(SYS_QZS, i);    /* 193-202: QZSS ref [2] */
       else             sat = 0;                    /* 203-   : reserved */
       raw->nav.sbssat.sat[n].sat = sat;
    }

    trace(5, "SBF decode_sbsprnmask: nprn=%d iodp=%d\n", n, raw->nav.sbssat.iodp);

    return 3;
}

/* decode type 6: integrity info ---------------------------------------------*/
static int decode_sbsintegriy(raw_t *raw)
{
    int i, prn;
    uint8_t *p = raw->buff+6;
    uint8_t iodf[4], udre;


    trace(4, "decode_sbsintegriy:\n");

    if (raw->len < 71)
    {
        trace(1, "SBF decode_sbsintegriy: Block too short\n");
        return -1;
    }

    /* get satellite number */
    prn = U1(p+8);
    if (prn < 120) return -1;
    if (prn > 139) return -1;

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF SABS Integrity Data from PRN=%d", prn);
    }

    for (i=0; i<4; i++) {
        iodf[i] = U1(p+10+i);
    }
    /* Limited to 51 to avoid overflow of iodf[]. TODO check the logic */
    for (i=0; i<raw->nav.sbssat.nsat && i<51; i++) {
        if (raw->nav.sbssat.sat[i].fcorr.iodf != iodf[i/13]) continue;
        udre = U1(p+14+i);
        raw->nav.sbssat.sat[i].fcorr.udre = udre;
    }
    trace(5, "SBF decode_sbsintegriy: iodf=%d %d %d %d\n", iodf[0], iodf[1], iodf[2], iodf[3]);
    return 3;
}

/* decode type 7: fast correction degradation factor -------------------------*/
static int decode_sbsfastcorrdegr(raw_t *raw)
{
    int i,prn;
    uint8_t *p = raw->buff+6;


    trace(4, "SBF decode_sbsfastcorrdegr:\n");

    if (raw->len < 68)
    {
        trace(1, "SBF decode_sbsfastcorrdegr: Block too short\n");
        return -1;
    }

    /* get satellite number */
    prn = U1(p+8);
    if (prn < 120) return -1;
    if (prn > 139) return -1;

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF SBAS Fast Correction Degradation Factor from PRN=%d", prn);
    }

    if (raw->nav.sbssat.iodp != U1(p+9)) return 0;

    raw->nav.sbssat.tlat = U1(p+10);

    for (i=0; i<raw->nav.sbssat.nsat && i<MAXSAT; i++) {
        raw->nav.sbssat.sat[i].fcorr.ai = U1(p+11+i);
    }
    return 0;
}

/* decode type 26: ionospheric delay corrections -----------------------------*/
static int decode_sbsionodelay(raw_t *raw)
{
    int i, j, give, prn;
    int band;
    uint8_t *p = raw->buff+8, sbLength, count;

    trace(4, "SBF decode_sbsionodelay:\n");

    if (raw->len < 20)
    {
        trace(1, "SBF decode_sbsionodelay: Block too short\n");
        return -1;
    }

    /* get satellite number */
    prn = U1(p+6);
    if (prn < 120) return -1;
    if (prn > 139) return -1;

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF SBAS Ionospheric Delay Correction from PRN=%d", prn);
    }

    band = U1(p+7);

    if (band>MAXBAND || raw->nav.sbsion[band].iodi != U1(p+8)) return 0;

    double tow = U4(p) * 0.001;
    uint16_t week = U2(p+4);

    count = U1(p+9);
    sbLength = U1(p+10);

    if (count != 15)
    {
        trace(1, "SBF decode_sbsionodelay: wrong number of IDC blocks: %d\n", count);
        return -1;
    }

    for (i=0; i<count; i++) {
        j = U1(p+12+i*sbLength);
        give = U1(p+12+i*sbLength+1);

        raw->nav.sbsion[band].igp[j].t0 = gpst2time(week, tow);
        raw->nav.sbsion[band].igp[j].delay = R4(p+12+i*sbLength+4);
        raw->nav.sbsion[band].igp[j].give = give;

        if (raw->nav.sbsion[band].igp[j].give >= 16) {
            raw->nav.sbsion[band].igp[j].give = 0;
        }
    }
    trace(5, "decode_sbsionodelay: band=%d\n", band);
    return 3;
}

/* decode type 18: ionospheric grid point masks ------------------------------*/
static int decode_sbsigpmask(raw_t *raw) /* TODO: verify this function */
{
    const sbsigpband_t *b;
    int i, j, n, m, prn;
    uint8_t band;

    uint8_t *p = raw->buff+8;

    trace(4, "SBF decode_sbsigpmask:\n");

    if (raw->len < 20)
    {
        trace(1, "SBF decode_sbsigpmask: Block too short\n");
        return -1;
    }

    /* get satellite number */
    prn = U1(p+6);
    if (prn < 120) return -1;
    if (prn > 139) return -1;

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF SBAS Ionospheric Grid Points from PRN=%d", prn);
    }

    band = U1(p+8);

    if      (band <= 8) {b = igpband1[band]; m = 8;}
    else if (band <= 10) {b = igpband2[band-9]; m = 5;}
    else return 0;

    raw->nav.sbsion[band].iodi = U1(p+9);
    raw->nav.sbsion[band].nigp = U1(p+10);

    for (n=0; n<raw->nav.sbsion[band].nigp; n++)
    {
        i = U1(p+11+n);
        for (j=0; j<m; j++) {
            if (i<b[j].bits || b[j].bite<i) continue;
            raw->nav.sbsion[band].igp[n].lat   = band<=8 ? b[j].y[i-b[j].bits] : b[j].x;
            raw->nav.sbsion[band].igp[n++].lon = band<=8 ? b[j].x : b[j].y[i-b[j].bits];
            break;
        }
    }

    trace(5, "decode_sbsigpmask: band=%d nigp=%d\n", band, n);

    return 3;
}
/* decode long term correction ------------------------------------------*/
static int decode_sbslongcorrh(raw_t* raw)
{
    int prn, i;
    uint8_t *p = raw->buff+8;
    uint8_t count, sbLength, no;

    trace(4, "SBF decode_sbslongcorrh:\n");

    if (raw->len<20)
    {
        trace(1, "SBF decode_sbslongcorrh: Block too short\n");
        return -1;
    }

    /* get satellite number */
    prn = U1(p+6);
    if (prn < 120) return -1;
    if (prn > 139) return -1;

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF SBAS Long Term Corrections from PRN=%d", prn);
    }

    double tow = U4(p + 0) * 0.001;
    uint16_t week = U2(p+4);

    count = U1(p+7);
    sbLength = U1(p+8);

    if (count > 4) return -1;

    for (i=0; i<count; i++)
    {
        no = U1(p+12+i*sbLength+1);
        raw->nav.sbssat.sat[no-1].lcorr.iode    = U1(p+12+i*sbLength+ 3);
        raw->nav.sbssat.sat[no-1].lcorr.dpos[0] = R4(p+12+i*sbLength+ 4);
        raw->nav.sbssat.sat[no-1].lcorr.dpos[1] = R4(p+12+i*sbLength+ 8);
        raw->nav.sbssat.sat[no-1].lcorr.dpos[2] = R4(p+12+i*sbLength+12);
        raw->nav.sbssat.sat[no-1].lcorr.daf0 = R4(p+12+i*sbLength+28);
        if (U1(p+12+i*sbLength+0) == 1)
        {
            raw->nav.sbssat.sat[no-1].lcorr.dvel[i] = R4(p+12+i*sbLength+16);
            raw->nav.sbssat.sat[no-1].lcorr.dvel[i] = R4(p+12+i*sbLength+20);
            raw->nav.sbssat.sat[no-1].lcorr.dvel[i] = R4(p+12+i*sbLength+24);

            raw->nav.sbssat.sat[no-1].lcorr.daf1 = R4(p+12+i*sbLength+32);

            /* Time of day.  Calculate day offset from TOW. */
            double t = U4(p+12+i*sbLength+36) - fmod(tow, 86400);
            if      (t <= -43200) t += 86400;
            else if (t >   43200) t -= 86400;
            raw->nav.sbssat.sat[no-1].lcorr.t0 = gpst2time(week, tow+t);
        } else
        {
            raw->nav.sbssat.sat[no-1].lcorr.dvel[0] = raw->nav.sbssat.sat[no-1].lcorr.dvel[1] = raw->nav.sbssat.sat[no-1].lcorr.dvel[2] = 0.0;
            raw->nav.sbssat.sat[no-1].lcorr.daf1 = 0;
            raw->nav.sbssat.sat[no-1].lcorr.t0 = gpst2time(week, tow);
        };

    };

    return 3;
}

#endif /* UNUSED */

/* decode external event block -----------------------------*/
static int decode_extevent(raw_t *raw)
{
    uint8_t *p = raw->buff+8;
    gtime_t eventime;
    uint8_t source, polarity;
    double offset, clk_bias;

    trace(4, "SBF decode_extevent:\n");

    if (raw->len < 20)
    {
        trace(1, "SBF decode_extevent: Block too short\n");
        return -1;
    }

    double tow = U4(p+0) * 0.001;
    uint16_t week = U2(p+4);
    source = U1(p+6);
    polarity = U1(p+7);
    offset = R4(p+8);
    clk_bias = R8(p+12);

    if (clk_bias == -2e10) /* do-not-use value*/
        clk_bias = 0;

    eventime = gpst2time(week, tow + offset - clk_bias);

    raw->obs.flag = 1 + source; /* Event flag */
    raw->obs.data[0].eventime = eventime;
    raw->obs.tmcount++;

    if (raw->outtype) {
        sprintf(raw->msgtype, "SBF external event WN=%d, tow=%f: source=%d, polarity=%d",
                week, tow + offset - clk_bias, source, polarity);
    }

    return 0;
}


/* decode SBF raw message --------------------------------------------------*/
static int decode_sbf(raw_t *raw)
{
    unsigned short crc;
    char tstr[40];

    /* read the SBF block ID and revision */
    int type = U2(raw->buff+4) & 0x1fff << 0;
    int revision = U2(raw->buff+4) >> 13;
    (void)revision;  /* unused */

    trace(3, "decode_sbf: type=%04x len=%d\n", type, raw->len);

    /* read the SBF block CRC */
    crc = U2(raw->buff+2);

    /* checksum skipping first 4 bytes */
    if (sbf_checksum(raw->buff+4, raw->len-4) !=  crc){
        trace(2, "sbf checksum error: type=%04x len=%d\n", type, raw->len);
        return -1;
    }

    if (raw->len < 14) {
        trace(2, "sbf length error: type=%d len=%d\n", type, raw->len);
        return -1;
    }
    uint32_t tow = U4(raw->buff+8);
    uint16_t week = U2(raw->buff+12);
    if (tow == 4294967295u || week == 65535u) {
        trace(2, "sbf tow/week error: type=%d len=%d\n", type, raw->len);
        return -1;
    }
    raw->time = gpst2time(week, tow*0.001);

    if (raw->outtype) {
        time2str(raw->time, tstr, 2);
        sprintf(raw->msgtype, "SBF %4d (%4d): %s", type, raw->len, tstr);
    }

    switch (type) {
        /* Measurement Blocks */

        /* only read and store data and indicate new obs data only at ID_MEASEPOCH_END */
        case ID_MEASEPOCH:      return decode_measepoch(raw);
        case ID_MEASEPOCHEXTRA: return decode_measepochextra(raw);
        case ID_MEASE3RNG:      return decode_meas3ranges(raw);
        case ID_MEASE3DOPPLER:  return decode_meas3Doppler(raw);
        case ID_MEASE3CN:       return decode_meas3CN(raw);
        case ID_MEASEPOCH_END:
            if (raw->outtype) {
                sprintf(raw->msgtype, "SBF Measurement Epoch End");
            }
            return flushobuf(raw);


        /* Navigation Page Blocks */
        case ID_GPSRAWCA:       return decode_gpsrawcanav(raw, SYS_GPS);
        case ID_GPSRAWL2C:
        case ID_GPSRAWL5:       return decode_gpsrawcnav(raw, SYS_GPS);
#ifdef ENAGLO
        case ID_GLORAWCA:       return decode_glorawcanav(raw);
#endif
#ifdef ENAGAL
        case ID_GALRAWINAV:     return decode_galrawinav(raw);
        case ID_GALRAWFNAV:     return decode_galrawfnav(raw);
        /*case ID_GALRAWCNAV:     return decode_galrawcnav(raw); */
#endif
        case ID_GEORAWL1:
        case ID_GEORAWL5:       return decode_georaw(raw);
#ifdef ENACMP
        case ID_BDSRAW:         return decode_cmpraw(raw);
        case ID_BDSRAWB1C:
        case ID_BDSRAWB2A:
        case ID_BDSRAWB2B:      return 0; /* yet unsupported by rtklib */
#endif
#ifdef ENAQZS
        case ID_QZSRAWL1CA:    return decode_gpsrawcanav(raw, SYS_QZS);
        case ID_QZSRAWL2C:
        case ID_QZSRAWL5:      return decode_gpsrawcnav(raw, SYS_QZS);
        case ID_QZSSL6:
        case ID_QZSSL1C:
        case ID_QZSSL1S:
        case ID_QZSSL5S:
            return 0;
#endif
#ifdef ENAIRN
        case ID_IRNSSRAW:       return decode_navicraw(raw);
#ifdef TESTING /* not tested */
        case ID_NAVICLNAV:      return decode_naviclnav(raw);
#else
        case ID_NAVICLNAV:      return 0;
#endif
#endif

        /* GPS Decoded Message Blocks */
        case ID_GPSNAV:         return decode_gpsnav(raw);
        case ID_GPSALM:         return decode_gpsalm(raw);
        case ID_GPSION:         return decode_gpsion(raw);
        case ID_GPSUTC:         return decode_gpsutc(raw);
        case ID_GPSCNAV:
        case ID_GPSCNAV2:       return decode_gpscnav(raw, type);

        /* GLONASS Decoded Message Blocks */
#ifdef ENAGLO
        case ID_GLONAV:         return decode_glonav(raw);
        case ID_GLOALM:         return 0; /* not suppported by rtklib */
        case ID_GLOTIME:        return decode_gloutc(raw);
#endif
        /* Galileo Decoded Message Blocks */
#ifdef ENAGAL
        case ID_GALNAV:         return decode_galnav(raw);
        case ID_GALALM:         return decode_galalm(raw);
        case ID_GALION:         return decode_galion(raw);
        case ID_GALUTC:         return decode_galutc(raw);
#endif
        /* BeiDou Decoded Message Blocks */
#ifdef ENACMP
        case ID_BDSNAV:         return decode_cmpnav(raw);
        case ID_BDSCNAV1:
        case ID_BDSCNAV2:
        case ID_BDSCNAV3:
                                return decode_cmpcnav2(raw, type);
        case ID_BDSALM:         return decode_cmpalm(raw);
        case ID_BDSION:         return decode_cmpion(raw);
        case ID_BDSUTC:         return decode_cmputc(raw);
#endif

        /* QZSS Decoded Message Blocks */
#ifdef ENAQZS
        case ID_QZSSNAV:       return decode_qzssnav(raw);
#ifdef TESTING /* not tested */
        case ID_QZSSALM:       return decode_qzssalm(raw);
#else
        case ID_QZSSALM:
            return 0;
#endif
#endif

        /* NavIC/IRNSS Decoded Message Blocks */

        /* SBAS L1 Decoded Message Blocks */
        case ID_GEOMT00:        return decode_sbsfast(raw);
        case ID_GEOPRNMASK:     return decode_sbsprnmask(raw);
        case ID_GEOFASTCORR:    return decode_sbsfast(raw);
        case ID_GEOINTEGRITY:   return decode_sbsintegriy(raw);
        case ID_GEOFASTCORRDEGR:return decode_sbsfastcorrdegr(raw);
        case ID_GEONAV:         return decode_geonav(raw);
        case ID_GEOIGPMASK:     return decode_sbsigpmask(raw);
        case ID_GEOLONGTERMCOR: return decode_sbslongcorrh(raw);
        case ID_GEOIONODELAY:   return decode_sbsionodelay(raw);

        /* External Event Blocks */
        case ID_EXTEVENT:       return decode_extevent(raw);


#if 1 /* UNUSED */
        case ID_GENMEASEPOCH:   return 0; /* to be implemented */
        case ID_MEASFULLRANGE:  return 0; /* to be implemented */
        case ID_SBASL5NAV:      return 0; /* to be implemented */
        case ID_SBASL5ALM:      return 0; /* to be implemented */
        /* Differential Correction Blocks */
        case ID_DIFFCORRIN:     return 0; /* not yet supported */

        case ID_GEONETWORKTIME: return 0; /* not suppported by rtklib */
        case ID_GEOALM:         return 0;
        case ID_PVTSATCART:     return 0; /* to be checked */
        case ID_GEOCORR:        return 0; /* to be checked */
#endif /* UNUSED */

        /* commands that don't need to be handled */
            /* Measurement Blocks */
        case ID_MEASE3PP:
        case ID_MEASE3MP:
        case ID_IQCORR:
        case ID_ISMR:
        case ID_SQMSAMPLES:
            /* Navigation Page Blocks */
        case ID_GNSSNAVBITS:
        case ID_GNSSSYMBOLS:
            /* Galileo Decoded Message Blocks */
        case ID_GALGSTGPS:
        case ID_GALARRLM:
            /* SBAS L1 Decoded Message Blocks */
        case ID_GEOSERVICELEVEL:
        case ID_GEOCLOCKEPHCOVMATRIX:
        case ID_GEODEGRFACTORS:
            /* GNSS Position, Velocity and Time Blocks */
        case ID_PVTCART1:
        case ID_PVTGEOD1:
        case ID_DOP1:
        case ID_PVTRESIDUALS1:
        case ID_RAIMSTATS1:
        case ID_PVTRESIDUALS2:
        case ID_PVTCART2:
        case ID_PVTGEOD2:
        case ID_PVTGEODAUTH:
        case ID_COVCART:
        case ID_COVGEOD:
        case ID_VELCOVCART:
        case ID_VELCOVGEOD:
        case ID_DOP2:
        case ID_DOPAUTH:
        case ID_POSCART:
        case ID_PVTLOCAL:
        case ID_POSPROJ:
        case ID_RAIMSTATS2:
        case ID_BASEVECCART:
        case ID_BASEVECGEOD:
        case ID_AMBIGUITIES:
        case ID_PVTSUPPORT:
        case ID_PVTSUPPORTA:
        case ID_PVTEND:
        case ID_BASELINE:
            /* INS/GNSS Integrated Blocks */
        case ID_INTPVCART:
        case ID_INTPVGEOD:
        case ID_INTPOSCOVCART:
        case ID_INTVELCOVCART:
        case ID_INTPOSCOVGEOD:
        case ID_INTVELCOVGEOD:
        case ID_INTATTEULER:
        case ID_INTATTCCOEULER:
        case ID_INTPVAAGEOD:
        case ID_INSNAVCART:
        case ID_INSNAVGEOD:
        case ID_IMUBIAS:
            /* GNSS Attitude Blocks */
        case ID_ATTEULER:
        case ID_ATTCOVEULER:
        case ID_AUXPOS:
        case ID_ENDATT:
        case ID_ATTQUAD:
        case ID_ATTCOVQUAD:
            /* Receiver Time Blocks */
        case ID_RXTIME:
        case ID_PPSOFFSET:
        case ID_SYSTIMEOFF:
        case ID_FUGROTIMEOFF:
            /* External Event Blocks */
        case ID_EXTEVENTCART:
        case ID_EXTEVENTGEO:
        case ID_EXTEVENTBASEVECCART:
        case ID_EXTEVENTBASEVECGEOD:
        case ID_EXTEVENTINSNAVCART:
        case ID_EXTEVENTINSNAVGEOD:
        case ID_EXTEEVENTAATTEULER:
            /* Differential Correction Blocks */
        case ID_BASESTATION:
        case ID_RTCMDATUM:
        case ID_BASELINK:
            /* L-Band Demodulator Blocks */
        case ID_LRECEIVER:
        case ID_LTRACK:
        case ID_LDECODE:
        case ID_LBEAMS:
        case ID_FUGRODSS:
        case ID_FUGROSTAT:
            /* External Sensor Blocks */
        case ID_EXTSENSORMEAS:
        case ID_EXTSENSORSTATUS:
        case ID_EXTSENSORSETUP:
        case ID_EXTSENSORSTATUS2:
        case ID_EXTSENSORINFO:
        case ID_IMUSETUP:
        case ID_VELSENSORSETUP:
        case ID_LEVERARMSUPPORT1:
            /* Status Blocks */
        case ID_RXSTATUS1:
        case ID_TRACKSTATUS:
        case ID_CHNSTATUS:
        case ID_RXSTATUS2:
        case ID_SATVISIBILITY:
        case ID_INPUTLINK:
        case ID_OUTPUTLINK:
        case ID_NTRIPCLTSTAT:
        case ID_NTRIPSVRSTAT:
        case ID_IPSTATUS:
        case ID_WIFIAPSTATUS:
        case ID_WIFICLTSTATUS:
        case ID_CELLULARSTATUS:
        case ID_BLUETOOTHSTATUS:
        case ID_DYNDNSSTATUS:
        case ID_POWERSTATUS:
        case ID_QUALIND:
        case ID_DISKSTATUS:
        case ID_LOGSTATUS:
        case ID_UHFSTATUS:
        case ID_RFSTATUS:
        case ID_RIMSHEALTH:
        case ID_OSNMASTATUS:
        case ID_GALNAVMONITOR:
        case ID_GALNAVRCEDMONITOR:
        case ID_INAVMONITOR:
        case ID_P2PPSTATUS:
        case ID_COSMOSTATUS:
        case ID_GALAUTHSTATUS:
        case ID_FUGROAUTHSTATUS:
            /* Miscellaneous Blocks */
        case ID_RXSETUP:
        case ID_RXMESSAGE:
        case ID_COMMANDS:
        case ID_COMMENT:
        case ID_BBSMPS:
        case ID_RXCOMPS:
        case ID_ASCIIIN:
        case ID_ENCAPSOUT:
        case ID_RAWDATAIN:
        case ID_IMURAWSAMPLES:
        case ID_INTERFACESTATS:
            /* Advanced Blocks */
        case ID_SYSINFO:

            return 0;
        default:
#if 0 /* debug output */
            if (raw->outtype) {
                sprintf(raw->msgtype,"SBF 0x%04X (%4d):",type, raw->len);
            }
#endif
            trace(3, "decode_sbf: unused frame type=%04x len=%d\n", type, raw->len);
        /* there are many more SBF blocks to be extracted */
    }
    return 0;
}

/* sync to the beginning of a block ------------------------------------------*/
static int sync_sbf(unsigned char *buff, unsigned char data)
{
    buff[0] = buff[1];
    buff[1] = data;
    return buff[0]== SBF_SYNC1 && buff[1]==SBF_SYNC2;
}
/* input sbf raw data from stream ----------------------------------------------
* get to the next sbf raw block from stream
* args   : raw_t  *raw   IO     receiver raw data control struct
*          unsigned char data I stream data (1byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 3: input sbas message,
*                  9: input ion/utc parameter)
*          to specify input options for sbf, set raw->opt to the following
*          option strings separated by spaces.
*
*
*          -EPHALL  : input all ephemerides
*          -AUX1    : select antenna Aux1  (default: main)
*          -AUX2    : select antenna Aux2  (default: main)
*          -GL1W    : select 1W for GPS L1 (default: 1C)
*          -GL1L    : select 1L for GPS L1 (default: 1C)
*          -GL2L    : select 2L for GPS L2 (default: 2W)
*          -RL1P    : select 1P for GLO G1 (default: 1C)
*          -RL2C    : select 2C for GLO G2 (default: 2P)
*          -JL1L    : select 1L for QZS L1 (default: 1C)
*          -JL1Z    : select 1Z for QZS L1 (default: 1C)
*          -CL1P    : select 1P for BDS B1 (default: 2I)
*          -GALINAV : select I/NAV for Galileo ephemeris (default: all)
*          -GALFNAV : select F/NAV for Galileo ephemeris (default: all)
*          -NO_MEAS2: ignore range measurements version 2 blocks
*          -NO_MEAS3: ignore range measurements version 3 blocks
*          -RCVSTDS : save receiver stdevs to unused RINEX fields
*-----------------------------------------------------------------------------*/
extern int input_sbf(raw_t *raw, unsigned char data)
{
    trace(5, "input_sbf: data=%02x\n",data);

    if (raw->nbyte == 0) {
        if (sync_sbf(raw->buff, data)) raw->nbyte = 2;
        return 0;
    }
    raw->buff[raw->nbyte++] = data;

    if (raw->nbyte < 8) return 0;

    if ((raw->len = U2(raw->buff+6)) > MAXRAWLEN) {
        trace(2, "sbf length error: len=%d\n", raw->len);
        raw->nbyte = 0;
        return -1;
    }
    if (raw->nbyte<raw->len) return 0;
    raw->nbyte = 0;

    return decode_sbf(raw);
}
/* sbf raw block finder --------------------------------------------------------
* get to the next sbf raw block from file
* args   : raw_t  *raw   IO     receiver raw data control struct
*          FILE   *fp    I      file pointer
* return : status(-2: end of file, -1...9: same as above)
*-----------------------------------------------------------------------------*/
extern int input_sbff(raw_t *raw, FILE *fp)
{
    int i, data;
    trace(4, "input_sbff:\n");

    /* go to the beginning of the first block */
    if (raw->nbyte == 0) {
        for (i=0;;i++) {
            data=fgetc(fp);
            if (data == EOF) return -2;
            if (sync_sbf(raw->buff, (unsigned char)data)) break;
            if (i >= MAXRAWLEN) return 0;
        }
    }

    /* load block header content (8 bytes) in raw->buff */
    /* since we already read the first two, we just read the next 6 bytes */
    if (fread(raw->buff+2, 1, 6, fp) < 6) return -2;
    raw->nbyte = 8;

    /* decode the length of the block and store it in len*/
    if ((raw->len = U2(raw->buff+6)) > MAXRAWLEN) {
        trace(2, "sbf length error: len=%d\n", raw->len);
        raw->nbyte = 0;
        return -1;
    }

    /* let's store in raw->buff the whole block of length len */
    /* 8 bytes have been already read, we read raw->len-8 more */
    if (fread(raw->buff+8, 1, raw->len-8, fp) < (size_t)(raw->len-8)) return -2;
    raw->nbyte = 0;           /* this indicates where we point inside raw->buff */

    /* decode SBF block */
    return decode_sbf(raw);
}
