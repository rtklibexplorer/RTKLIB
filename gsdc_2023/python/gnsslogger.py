#!/usr/bin/env python3
"""
Module to process log files from Google's Android GNSS Logger app
"""
import datetime, time
import math
import re
import sys

# Input parameters
MAX_PRU_DEF = 25 # default value
MAX_PRU = {'sm-a325f':50, 'samsunga325g':50, 'samsunga32':50,
           'sm-a205u':25, 'sm-a505g':25, 'sm-a505u':25, 'sm-a600t':25, 
           'mi8':15, 'xiaomimi8':15, 'pixel5':10}      # max pseudorange uncertainty (m)
MAX_ADRU_DEF = 0.2   # .2 .1  # max carrier phase uncertainty (m)
MAX_ADRU = {'sm-a325f':0.001, 'samsunga325g':0.001, 'samsunga32':0.001,
            'mi8':0.1, 'sm-a505g':0.1}
MAX_DU_DEF = 2    # max doppler uncertainty (cycles)
MAX_DU = {'pixel5':2}
MIN_CN0 = 10
MAX_SVTIME_UNCERTAINTY = 500

# Flags to check wether the measurement is correct or not
# https://developer.android.com/reference/android/location/GnssMeasurement.html#getState()
STATE_2ND_CODE_LOCK = int(0x00010000)
STATE_BDS_D2_BIT_SYNC = int(0x00000100)
STATE_BDS_D2_SUBFRAME_SYNC = int(0x00000200)
STATE_BIT_SYNC = int(0x00000002)
STATE_CODE_LOCK = int(0x00000001)
STATE_GAL_E1BC_CODE_LOCK = int(0x00000400)
STATE_GAL_E1B_PAGE_SYNC = int(0x00001000)
STATE_GAL_E1C_2ND_CODE_LOCK = int(0x00000800)
STATE_GLO_STRING_SYNC = int(0x00000040)
STATE_GLO_TOD_DECODED = int(0x00000080)
STATE_GLO_TOD_KNOWN = int(0x00008000)
STATE_MSEC_AMBIGUOUS = int(0x00000010)
STATE_SBAS_SYNC = int(0x00002000)
STATE_SUBFRAME_SYNC = int(0x00000004)
STATE_SYMBOL_SYNC = int(0x00000020)
STATE_TOW_DECODED = int(0x00000008)
STATE_TOW_KNOWN = int(0x00004000)
STATE_UNKNOWN = int(0x00000000)

ADR_STATE_UNKNOWN = int(0x00000000)
ADR_STATE_VALID = int(0x00000001)
ADR_STATE_RESET = int(0x00000002)
ADR_STATE_HALF_CYCLE_RESOLVED = int(0x00000008)
ADR_STATE_HALF_CYCLE_REPORTED = int(0x00000010)
ADR_STATE_CYCLE_SLIP = int(0x00000004)


# Define constants
SPEED_OF_LIGHT = 299792458.0  # [m/s]
GPS_WEEKSECS = 604800  # Number of seconds in a week
NS_TO_S = 1.0e-9
NS_TO_M = NS_TO_S * SPEED_OF_LIGHT  # Constant to transform from nanoseconds to meters
BDST_TO_GPST = 14  # Leap seconds difference between BDST and GPST
GLOT_TO_UTC = 10800  # Time difference between GLOT and UTC in seconds
# Origin of the GPS time scale
GPSTIME = datetime.datetime(1980, 1, 6)
DAYSEC = 86400  # Number of seconds in a day
CURRENT_GPS_LEAP_SECOND = 18
OBS_LIST = ['C', 'L', 'D', 'S']

EPOCH_STR = 'epoch'

FREQ1     = 1.57542E9 
FREQ1_GLO = 1.60200E9
FREQ1_BDS = 1.561098E9
FREQ5     = 1.17645E9
DFREQ1_GLO = 0.56250E6


# Constellation types
CONSTELLATION_GPS = 1
CONSTELLATION_SBAS = 2
CONSTELLATION_GLONASS = 3
CONSTELLATION_QZSS = 4
CONSTELLATION_BEIDOU = 5
CONSTELLATION_GALILEO = 6
CONSTELLATION_UNKNOWN = 0

L1_FREQS ={CONSTELLATION_GPS: FREQ1,
          CONSTELLATION_SBAS: FREQ1,
          CONSTELLATION_GLONASS: FREQ1_GLO,
          CONSTELLATION_QZSS: FREQ1,
          CONSTELLATION_BEIDOU : FREQ1_BDS,
          CONSTELLATION_GALILEO : FREQ1,
          CONSTELLATION_UNKNOWN : 0}
L5_FREQS ={CONSTELLATION_GPS: FREQ5,
          CONSTELLATION_SBAS: FREQ5,
          CONSTELLATION_GLONASS: 0,
          CONSTELLATION_QZSS: FREQ5,
          CONSTELLATION_BEIDOU : FREQ5,
          CONSTELLATION_GALILEO : FREQ5,
          CONSTELLATION_UNKNOWN : 0}

CONSTELLATION_LETTER = {
        CONSTELLATION_GPS : 'G',
        CONSTELLATION_SBAS : 'S',
        CONSTELLATION_GLONASS : 'R',
        CONSTELLATION_QZSS : 'J',
        CONSTELLATION_BEIDOU : 'C',
        CONSTELLATION_GALILEO : 'E',
        CONSTELLATION_UNKNOWN : 'X'
}

# global variables
clockDiscontinuities = -1
fullbiasnanos = 0
biasnanos = 0


class GnssLogHeader(object):
    """
    Class that manages the header from the log file.
    """

    def __init__(self, filename):
        """
        Initializes the header from a file handler to the file. Loads the
        parameters and field names present in the header part of the log
        """

        self.parameters = {}
        self.fields = {}

        with open(filename, 'r') as fh:

                for line in fh:

                        # Detect end of header
                        if not line.startswith('#'):
                                break

                        # Skip empty lines
                        if line.strip() == '#':
                                continue

                        fields = re.split('[: ,]', line.strip())

                        method_name = 'parse_{0}'.format(fields[1].lower())

                        # Call the method that processes the line
                        try:
                            getattr(GnssLogHeader, method_name)(self, line)
                        except:
                            pass


    def get_fieldnames(self, line):
        """
        """

        fields = [f.strip() for f in line[2:].strip().split(',')] # Skip initial hash character

        key = fields[0]
        field_names = fields[1:]

        self.fields[key] = field_names

        return

    def parse_header(self, line):
        """
        Get parameters from header line, in this case do nothing
        """
        pass

    def parse_version(self, line):
        """
        Return a dictionary
        """

        fields = line.strip().split(' ')

        for field in fields[1:]:

            if field.endswith(':'):
                key = field[:-1]
                self.parameters[key] = ''

            else:

                self.parameters[key] += ' ' + field

        # Clean superfluous spaces
        self.parameters = { k:self.parameters[k].strip() for k in self.parameters }

    def parse_fix(self, line):

        self.get_fieldnames(line)

    def parse_raw(self, line):
        """
        """
        self.get_fieldnames(line)

    def parse_nav(self, line):
        """
        """
        self.get_fieldnames(line)

# ------------------------------------------------------------------------------

class GnssLog(object):
    """
    """

    CONVERTER = {
        'AccumulatedDeltaRangeState' : int,
        'ConstellationType' : int,
        'MultipathIndicator' : int,
        'State' : int,
        'Svid' : int
    }

    def __init__(self, filename):
        """
        """

        # Criteria by which the different data batches are delimited
        self.BATCH_DELIMITER = 'TimeNanos'

        self.filename = filename

        self.header = GnssLogHeader(self.filename)

    def __field_conversion__(self, fname, valuestr):
        """
        Convert the field, by default will be float, unless it exists in
        the CONVERTER structure. If an exception occurs, the field will be
        left as is.
        """

        if fname in GnssLog.CONVERTER:
            try:
                return GnssLog.CONVERTER[fname](valuestr)
            except ValueError:
                return valuestr

        try:
                return float(valuestr)
        except ValueError:
                return valuestr

    def __parse_line__(self, line):
        """
        """

        line_fields = line.strip().split(',')

        field_names = self.header.fields[line_fields[0]]

        fields = { field_names[i] :  \
                   self.__field_conversion__(field_names[i], line_fields[i + 1]) \
                                        for i in range(len(line_fields) - 1)}

        return fields

    def raw_batches(self):
        """
        Generator function use to yield a batch
        """

        batch = []

        with open(self.filename, 'r') as fh:

            for line in fh:

                if not line.startswith('Raw'):
                        continue

                line_fields = self.__parse_line__(line)

                if len(batch) > 0 and line_fields[self.BATCH_DELIMITER] != batch[0][self.BATCH_DELIMITER]:
                        yield batch
                        batch = []

                batch.append(line_fields)

            # Yield last batch
            yield batch

    def fix_batches(self):
        """
        Generator function used to yield a position batch
        """

        with open(self.filename, 'r') as fh:

            for line in fh:

                if not line.startswith('Fix'):
                        continue

                yield self.__parse_line__(line)

# ------------------------------------------------------------------------------

def get_rnx_band_from_freq(frequency):
    """
    Obtain the frequency band

    >>> get_rnx_band_from_freq(1575420030.0)
    1
    >>> get_rnx_band_from_freq(1600875010.0)
    1
    >>> get_rnx_band_from_freq(1176450050.0)
    5
    >>> get_rnx_band_from_freq(1561097980.0)
    2
    """

    # Backwards compatibility with empty fields (assume GPS L1)
    ifreq = 154 if frequency == '' else round(frequency / 10.23e6)

    # QZSS L1 (154), GPS L1 (154), GAL E1 (154), and GLO L1 (156)
    if ifreq >= 154:
        return 1
    # QZSS L5 (115), GPS L5 (115), GAL E5 (115)
    elif ifreq == 115:
        return 5
    # BDS B1I (153)
    elif ifreq == 153:
        return 2
    else:
        raise ValueError("Cannot get Rinex frequency band from frequency [ {0} ]. "
        "Got the following integer frequency multiplier [ {1:.2f} ]\n".format(frequency, ifreq))

    return ifreq

# ------------------------------------------------------------------------------


def get_rnx_attr(band, constellation='G', state=0x00):
    """
    Generate the RINEX 3 attribute from a given band. Assumes 'C' for L1/E1
    frequency and 'Q' for L5/E5a frequency. For E5a it assumes Q tracking.
    """

    attr = 'C'

    # Make distinction between GAL E1C and E1B code
    if band == 1 and constellation == 'E':
        if (state & STATE_GAL_E1C_2ND_CODE_LOCK) == 0 and (state & STATE_GAL_E1B_PAGE_SYNC) != 0:
            attr = 'B'

    # GAL E5, QZSS L5, and GPS L5 (Q)
    if band == 5:
        attr = 'Q'

    # BDS B1I
    if band == 2 and constellation == 'C':
        attr = 'I'

    return attr

# ------------------------------------------------------------------------------

def get_frequency(measurement):
    # get freq from measurement

    v = measurement['CarrierFrequencyHz']
    return 154 * 10.23e6 if v == '' else v

# ------------------------------------------------------------------------------
def lookup_frequency(measurement, glo_freq_chns):
    # lookup precise freq for constellation and freq band of measurement
    constellation = measurement['ConstellationType']
    meas_freq = get_frequency(measurement)
    freq_band = get_rnx_band_from_freq(meas_freq)
    if freq_band == 1:
        actual_freq = L1_FREQS[constellation]
        if constellation == CONSTELLATION_GLONASS:
            # adjust for satellite
            sat = get_satname(measurement)
            try:
                slot = glo_freq_chns[sat]
                actual_freq += slot * DFREQ1_GLO
            except:
                pass
    elif freq_band == 5:
        actual_freq = L5_FREQS[constellation]
    else:
        actual_freq = meas_freq
    return actual_freq

# ------------------------------------------------------------------------------

def get_obscode(measurement):
    """
    Obtain the measurement code (RINEX 3 format)

    >>> get_obscode({'CarrierFrequencyHz': 1575420030.0, 'ConstellationType': 1})
    '1C'
    >>> get_obscode({'CarrierFrequencyHz': 1176450050.0, 'ConstellationType': 5})
    '5X'
    """

    band = get_rnx_band_from_freq(get_frequency(measurement))

    attr = get_rnx_attr(band, constellation=get_constellation(measurement), state=measurement['State'])

    return '{0}{1}'.format(band, attr)

# ------------------------------------------------------------------------------

def get_obslist(batches):
    """
    Obtain the observable list (array of RINEX 3.0 observable codes), particularized
    per each constellation, e.g.

    obs = {
        'G' : [C1C, L1C, D1C, S1C, C5Q],
        'E' : [C1C, L1C, D1C, C5Q],
        'R' : [C1P, C2P]
    }
    """

    obslist = {}

    for batch in batches:

        for measurement in batch:

                obscode = get_obscode(measurement)

                constellation = get_constellation(measurement)

                if constellation not in obslist:
                        obslist[constellation] = []

                arr = obslist[constellation]

                if obscode not in arr:
                        obslist[constellation].append(obscode)


    # Sort observable list for all constellations
    for c in obslist:
        arr = sorted(obslist[c])
        obslist[c] = [ m + o for o in arr for m in OBS_LIST  ]

    return obslist

# ------------------------------------------------------------------------------


def get_glo_freq_chn_list(batches):
    """
    Obtain the GLO frequency channel list (array of RINEX 3.0 observable codes), particularized
    per each constellation

    """

    freq_chn_list = {}

    for batch in batches:

        for measurement in batch:

            if measurement['ConstellationType'] == CONSTELLATION_GLONASS:
                try:
                    sat = get_satname(measurement)
                except ValueError as e:
                    sys.stderr.write("{0}\n".format(e))
                    continue
                
                if sat not in freq_chn_list:
                    freq = get_frequency(measurement)
                    freq_chn = round((freq -FREQ1_GLO ) / DFREQ1_GLO)
                    freq_chn_list[sat] = freq_chn

    return freq_chn_list

# ------------------------------------------------------------------------------


def get_glo_cod_phs_bis_list(batches):
    """
    Obtain the GLO code phase bias for the receiver used. This is unknown for the moment

    """

    cod_phs_bis_list = {}

    return cod_phs_bis_list

# ------------------------------------------------------------------------------


def check_adr_state(measurement, model):
    """
    Set ADR flags (cycle slip and half cycle)
    """
    # Obtain state, constellation type and frquency value to apply proper sync state
    adr_state = measurement['AccumulatedDeltaRangeState']
    slip = 0

    # get half cycle status, not supported for MI8
    if model != 'mi8' and not (adr_state & ADR_STATE_HALF_CYCLE_RESOLVED):
        slip |= 2
    # get cycle slip status
    if (adr_state & ADR_STATE_CYCLE_SLIP):
        slip |= 1
    elif not (adr_state & ADR_STATE_VALID):
        raise ValueError("ADR State [ 0x{0:2x} {0:8b} ] has ADR_STATE_VALID [ 0x{1:2x} {1:8b} ] not valid".format(adr_state, ADR_STATE_VALID))
    
    return slip

# ------------------------------------------------------------------------------


def filter_obs(measurement, model, cn0):
    """
    Use criteria from Google decimeter challenge baseline methodology
    
    Invalid measurements are discarded if:
        - the BiasUncertaintyNanos is larger or equal than 1E6
        - GnssClock values are invalid, e.g. FullBiasNanos is not a meaningful number
        - STATE_CODE_LOCK (for non-GAL E1) or STATE_GAL_E1BC_CODE_LOCK (for GAL E1) is not set
        - STATE_TOW_DECODED and STATE_TOW_KNOWN are not set for non-GLO signals
        - STATE_GLO_TOD_DECODED and STATE_GLO_TOD_KNOWN are not set for GLO signals,
         -CN0 is less than threshold
         - ReceivedSvTimeUncertaintyNanos is larger than threshold
         - Carrier frequency is out of nominal range of each band.
    """
   
    # Obtain state, constellation type and frequency value to apply proper sync state
    state = measurement['State']
    constellation = measurement['ConstellationType']
    #freq = get_frequency(measurement)
    #freq_band = get_rnx_band_from_freq(freq)

    # check code lock
    if state & (STATE_CODE_LOCK | STATE_GAL_E1BC_CODE_LOCK) == 0:
            raise ValueError('No code lock bits set: state=%x' % state)
    
    # check TOW or TOD decoded
    if constellation == CONSTELLATION_GLONASS:
        if state & (STATE_GLO_TOD_DECODED | STATE_GLO_TOD_KNOWN) == 0:
            raise ValueError('STATE_GLO_TOD not set: state=%x' % state)
    else:
        if state & (STATE_TOW_DECODED | STATE_TOW_KNOWN) == 0:
            raise ValueError('STATE_TOW not set: state=%x' % state)

    if constellation == CONSTELLATION_UNKNOWN:
        raise ValueError('ConstellationType Unknown')
    
    if cn0 < MIN_CN0:
        print('low cn0: %.1f' % cn0, file = sys.stderr)
        raise ValueError("CNo below threshold")
        
    if measurement['ReceivedSvTimeUncertaintyNanos'] > MAX_SVTIME_UNCERTAINTY:
        raise ValueError('ReceivedSvTime uncertainty exceeds threshold %.1f' % measurement['ReceivedSvTimeUncertaintyNanos'] )
        
    # Additional constraints beyond Google list

    #if  measurement['MultipathIndicator'] > 0:
    #    raise ValueError("Multipath indicated")
        

        
    # if measurement['PseudorangeRateUncertaintyMetersPerSecond'] > 100:
    #     raise ValueError('Range uncertainty exceeds threshold')

    return True

# ------------------------------------------------------------------------------

def get_constellation(measurement):
    """
    Return the constellation letter from a given measurement

    >>> get_constellation({'ConstellationType': 1})
    'G'
    >>> get_constellation({'ConstellationType': 6})
    'E'
    >>> get_constellation({'ConstellationType': 3})
    'R'
    >>> get_constellation({'ConstellationType': 5})
    'C'
    """

    ctype = measurement['ConstellationType']

    return CONSTELLATION_LETTER[ctype]

# ------------------------------------------------------------------------------


def get_satname(measurement):
    """
    Obtain the satellite name from a GNSS Logger measurement

    >>> get_satname({'ConstellationType': 1, 'Svid': 5})
    'G05'
    >>> get_satname({'ConstellationType': 6, 'Svid': 11})
    'E11'
    >>> get_satname({'ConstellationType': 3, 'Svid': 24})
    'R24'
    """

    c = get_constellation(measurement)

    svid = measurement['Svid']
    if c == 'J':
        svid -= 192

    satname = '{0}{1:02d}'.format(c, svid)

    # Make sure that we report GLONASS OSN (PRN) instead of FCN
    #  https://developer.android.com/reference/android/location/GnssStatus.html#getSvid(int)
    if svid > 50 and c == CONSTELLATION_LETTER[CONSTELLATION_GLONASS]:
        raise ValueError("-- WARNING: Skipping measurement for GLONASS sat "
                         "without OSN [ {0} ]".format(satname))

    return satname

# ------------------------------------------------------------------------------

def reset_clock():
    global clockDiscontinuities
    
    clockDiscontinuities = -1
    

def process(measurement, model, fix_bias=True, timeadj=1e-7, pseudorange_bias=0.0, 
            filter_mode="sync", glo_freq_chns = {}, slip_mask=3):
    """
    Process a log measurement. This method computes the pseudorange, carrier-phase (in cycles)
    Doppler (cycles/s) as well as CN/0

    :param measurement: GNSS Logger measurement line to process
    :param fullbiasnanos: Full Bias Nanos, used to either fix it to a certain
                          value (if value is provided) or update it with the
                          data if None (default value)
    :param timeadj: Interval to adjust the measurements
                       to the nearest rounded time stamp 
    :param pseudorange_bias: Add an externally computed bias to the pseudorange.
                             Default is 0.0
    :param filter_mode: Specify the filtering scheme for the raw file. Acceptable values are
                        sync: TOW/TOD known, code locked, no ambiguities are
                        detected, and all remaining flags for the signal are set
                        trck: TOW/TOD known, code locked and no ambiguities are detected
    """
    
    global clockDiscontinuities, fullbiasnanos, biasnanos

    try:
        satname = get_satname(measurement)
    except ValueError as e:
        sys.stderr.write("{0}\n".format(e))
        return None
    
    obscode = get_obscode(measurement)
    
    # get phone specific thresholds
    max_pru = MAX_PRU.get(model, MAX_PRU_DEF)
    max_adru = MAX_ADRU.get(model, MAX_ADRU_DEF)
    max_du = MAX_DU.get(model, MAX_DU_DEF)

    # force use of biasnano counts if clock discontinuity
    zeroCarrierPhase = False
    if clockDiscontinuities != measurement['HardwareClockDiscontinuityCount']:
        #print('Reload biases %.2f: %d %d' % (measurement['utcTimeMillis']/1000,clockDiscontinuities,measurement['HardwareClockDiscontinuityCount']))
        fullbiasnanos = measurement['FullBiasNanos']
        biasnanos = measurement['BiasNanos']
        if clockDiscontinuities > 0: # skip for initial epoch
            zeroCarrierPhase = True 
    elif not fix_bias:
    # Set the fullbiasnanos if not set or if we need to update the fullbiasnanos at each epoch
        fullbiasnanos = measurement['FullBiasNanos']
        biasnanos = measurement['BiasNanos']
    clockDiscontinuities = measurement['HardwareClockDiscontinuityCount']
    
    # Obtain time nanos and bias nanos. Skip if None
    try:
        timenanos = float(measurement['TimeNanos'])
    except ValueError:
        raise ValueError("-- WARNING: Invalid value of TimeNanos or satellite  [ {0} ]\n".format(satname))


    # Compute the GPS week number and reception time (i.e. clock epoch)
    gpsweek = math.floor(-fullbiasnanos * NS_TO_S / GPS_WEEKSECS)
    local_est_GPS_time = timenanos - (fullbiasnanos + biasnanos)
    gpssow = local_est_GPS_time * NS_TO_S - gpsweek * GPS_WEEKSECS

    # Adjust timestamp to rounded interval
    if timeadj > 0.0:
        toff = (gpssow / timeadj - int(gpssow / timeadj + 0.5)) * timeadj;
    else:
        toff = 0.0

    # Convert the epoch to Python's buiit-in datetime class
    gpssow_adj = gpssow - toff  
    gpst_epoch = GPSTIME + datetime.timedelta(weeks=gpsweek, seconds=gpssow_adj)

    try:
        timeoffsetnanos = float(measurement['TimeOffsetNanos'])
    except ValueError:
        timeoffsetnanos = 0.0

    # Compute the reception times
    tRxSeconds = gpssow - timeoffsetnanos * NS_TO_S

    # Compute transmitted wavelengths in cycles
    freq = lookup_frequency(measurement, glo_freq_chns)
    #freq = get_frequency(measurement)  # rcvr measurement of freq
    wavelength = SPEED_OF_LIGHT / freq
    
    # hack for bad cn0s for samsung phones
    cn0 = measurement['Cn0DbHz']
    if cn0 == 5.0:
        if measurement['AccumulatedDeltaRangeMeters'] == 0:
            cn0 = 20.1
        else:
            cn0 = 25.1
        print('fix cn0: %.1f -> %.1f' % (measurement['Cn0DbHz'], cn0), file = sys.stderr)
        
    
    # output debug info
    state = measurement['State']
    satname = get_satname(measurement)
    constellation = measurement['ConstellationType']
    adr_state = measurement['AccumulatedDeltaRangeState']
    multipath = measurement['MultipathIndicator']
    freq_band = get_rnx_band_from_freq(freq)

    PRU = min(999, measurement['ReceivedSvTimeUncertaintyNanos'] / 1e9 * SPEED_OF_LIGHT)
    ADRU = min(999, measurement['AccumulatedDeltaRangeUncertaintyMeters'])
    DU = min(999, measurement['PseudorangeRateUncertaintyMetersPerSecond'] / wavelength)
    print('%s: %s %s: STATE=%5x ADR_STATE=%2x MP=%1d ADRU=%8.4f PRU=%6.2f DU=%4.3f CN0=%4.1f ClkDisc=%d utc=%d' % 
          (gpst_epoch.strftime("%Y\%d\%m %H:%M:%S.%f"), satname, obscode, state,
           adr_state, multipath, ADRU, PRU, DU, cn0, measurement['HardwareClockDiscontinuityCount'],
          measurement['utcTimeMillis']), file = sys.stderr)
    
    # Filter out bad and low quality observations
    try:
        filter_obs(measurement, model, cn0)
    except ValueError as e:
        sys.stderr.write("-- WARNING: {0} for satellite [ {1} ]\n".format(e, satname))
        return None # return an empty/None dictionary

    # Compute transmit time (depends on constellation of origin)
    # GLOT is given as TOD, need to change to TOW
    if constellation == CONSTELLATION_GLONASS:
        # Compute the UTC time
        tod_secs = measurement['ReceivedSvTimeNanos'] * NS_TO_S
        tTxSeconds = glot_to_gpst(gpst_epoch, tod_secs)
        # Compute the travel time, which will be eventually the pseudorange
        tau = check_week_crossover(tRxSeconds, tTxSeconds)

    # BDST uses different epoch as GPS
    elif constellation == CONSTELLATION_BEIDOU:
        tTxSeconds = measurement['ReceivedSvTimeNanos'] * NS_TO_S + BDST_TO_GPST
        # Compute the travel time, which will be eventually the pseudorange
        tau = check_week_crossover(tRxSeconds, tTxSeconds)

    # GPS, QZSS, GAL and SBAS share the same epoch time
    else:
        tTxSeconds = measurement['ReceivedSvTimeNanos'] * NS_TO_S
        # Compute the travel time, which will be eventually the pseudorange
        tau = check_week_crossover(tRxSeconds, tTxSeconds)

    # Compute the range as the difference between the received time and
    # the transmitted time
    if PRU < max_pru:
        range = tau * SPEED_OF_LIGHT - pseudorange_bias
    else:
        range = 0.0
        sys.stderr.write("WARNING: Range uncertainty exceeds threshold\n")

    # Check ADR state of the satellite for carrier phase computation
    slip = Lstd = Pstd = 0
    cphase = 0.0
    try:
        slip = check_adr_state(measurement, model)
    except ValueError as e:
        sys.stderr.write("-- CHECK_ADR_STATE: WARNING: {0} for satellite [ {1} ]\n".format(e, satname))
    except Exception as e:
        sys.stderr.write("Error in check_adr_state:%s\n" % e)
    else:
        if slip == 1:
           sys.stderr.write("WARNING: Carrier phase slip\n") 
        # Process the accumulated delta range (i.e. carrier phase). This
        # needs to be translated from meters to cycles (i.e. RINEX format
        # specification)
        if ADRU < max_adru and zeroCarrierPhase == False:
            cphase = measurement['AccumulatedDeltaRangeMeters'] / wavelength
        else:
            cphase = 0.0
            slip |= 1
            if ADRU >= max_adru: 
                sys.stderr.write("WARNING: Carrier phase uncertainty exceeds threshold\n")
            else: # zeroCarrierPhase == True
                sys.stderr.write("WARNING: Hardware clock discontinuity\n")
    # check doppler uncertainty threshold against threshld
    if DU < max_du:
        doppler = -measurement['PseudorangeRateMetersPerSecond'] / wavelength
    else:
        doppler = 0.0
        sys.stderr.write("WARNING: Doppler uncertainty exceeds threshold\n")
    
    f = str(freq_band)
    
    # Check if the range and cphase need to be modified with the range rate in
    # order to make it consistent with the timestamp. Also compute uncertainties
    # in RTKLIB (u-blox) units
    if range != 0.0:
        # convert to RTKLIB units of 0.01*2^(n+5) meters/count
        Pstd = max(0,min(9, round(math.log2(PRU * 100)) - 5))
        if toff != 0.0:
            range -= toff * SPEED_OF_LIGHT
    if cphase != 0.0:
        cyc = ADRU * (freq / SPEED_OF_LIGHT) # convert to cycles
        # adjust phase uncertainity by LLI flags
        if slip & 1:
            ADRU += 2 * wavelength # add two cycles for cycle slip
        elif slip & 2:
            ADRU += 0.5 * wavelength # add half cycle for half-cycle uncertainty
        # convert to RTKLIB units of 0.004 cycles/count
        Lstd = max(0,min(9, round(cyc / 0.004))) 
        if toff != 0.0:
            cphase -= toff * freq
        
    print('    %s: %s: prange=%.5f, cphase=%.5f, lam=%.9f, slip=%d, Lstd=%d, Pstd=%d' 
          % (satname, obscode, range, cphase,wavelength,slip, Lstd, Pstd), file = sys.stderr)

    slip &= int(slip_mask)
    
    range = max(range, 0)  # reject range < 0
        
    return { EPOCH_STR : (gpst_epoch, gpssow_adj),
             satname : { 'C' + obscode : range,
                         'L' + obscode : cphase,
                         'D' + obscode : doppler,
                         'S' + obscode : cn0,
                         'slip' + f    : slip,
                         'L'+ f +'std' : Lstd,
                         'P'+ f + 'std': Pstd}}

# ------------------------------------------------------------------------------

def get_leap_seconds(current_epoch):
    """
    Computes the number of leap seconds passed since the start of GPST 
    :param current_epoch: current datetime value representing the measurements epoch
    :return: number of leap seconds since GPST
    """
    return None

# ------------------------------------------------------------------------------


def glot_to_gpst(gpst_current_epoch, tod_seconds):
    """
    Converts GLOT to GPST
    :param gpst_current_epoch: Current epoch of the measurement in GPST
    :param tod_seconds: Time of days as number of seconds
    :return: Time of week in seconds
    """
    (tod_sec_frac, tod_sec) = math.modf(tod_seconds)
    tod_sec = int(tod_sec)

    # Get the GLONASS epoch given the current GPS time
    glo_epoch = datetime.datetime(year=gpst_current_epoch.year,
                                  month=gpst_current_epoch.month,
                                  day=gpst_current_epoch.day,
                                  hour=gpst_current_epoch.hour,
                                  minute=gpst_current_epoch.minute,
                                  second=gpst_current_epoch.second)\
                + datetime.timedelta(hours=3, seconds=-CURRENT_GPS_LEAP_SECOND)

    # Adjust the GLONASS time with the TOD measurements
    glo_tod = datetime.datetime(year=glo_epoch.year,
                                month=glo_epoch.month,
                                day=glo_epoch.day) + datetime.timedelta(seconds=tod_sec)

    # The day of week in seconds needs to reflect the time passed before the current day starts
    day_of_week_sec = (glo_tod.isoweekday()) * DAYSEC

    # Compute time of week in seconds
    tow_sec = day_of_week_sec + tod_seconds - GLOT_TO_UTC + CURRENT_GPS_LEAP_SECOND

    return tow_sec

# ------------------------------------------------------------------------------


def check_week_crossover(tRxSeconds, tTxSeconds):
    """
    Checks time propagation time for week crossover
    :param tRxSeconds: received time in seconds of week
    :param tTxSeconds: transmitted time in seconds of week
    :return: corrected propagation time

    """

    tau = tRxSeconds - tTxSeconds
    if abs(tau) > GPS_WEEKSECS / 2:
        del_sec = round(tau/GPS_WEEKSECS)*GPS_WEEKSECS
        rho_sec = tau - del_sec

        if rho_sec > 10:
            tau = 0.0
        else:
            tau = rho_sec

    return tau

# ------------------------------------------------------------------------------


def check_day_crossover(tRxSeconds, tTxSeconds):
    """
    Checks time propagation time for day crossover
    :param tRxSeconds: received time in seconds of week
    :param tTxSeconds: transmitted time in seconds of week
    :return: corrected propagation time

    """

    tau = tRxSeconds - tTxSeconds
    if tau > DAYSEC / 2:
        del_sec = round(tau/DAYSEC)*DAYSEC
        rho_sec = tau - del_sec

        if rho_sec > 10:
            tau = 0.0
        else:
            tau = rho_sec

    return tau

# ------------------------------------------------------------------------------


def merge(measdict):
    """
    Merge a list of processed batches, which are dictionaries with an epoch
    and an internal dictionary with the satellite measurements

    """

    res = None

    for m in measdict:

        # Skip empty measurements
        if m is None:
            continue

        # Initialize
        if res is None:
            res = m
            continue

        exp_epoch = res[EPOCH_STR]
        got_epoch = m[EPOCH_STR]

        if got_epoch != exp_epoch:
            sys.stderr.write("Wrong measurement when merging batches. Expected "
                             "[ {0} ], got [ {1} ]. Will be skipped\n".format(exp_epoch, got_epoch))
            continue

        # Lambda method to get the satellites from the batch
        satsin = lambda x : [k for k in x.keys() if k is not EPOCH_STR]

        exp_sats = satsin(res)
        got_sats = satsin(m)

        # Loop over all the got satellites and merge them
        for sat in got_sats:

            if sat in exp_sats:
                res[sat].update(m[sat])
            else:
                res[sat] = m[sat]

    return res

# ------------------------------------------------------------------------------

if __name__ == '__main__':
    import doctest
    doctest.testmod(raise_on_error=True)
