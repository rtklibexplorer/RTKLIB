#!/usr/bin/env python
"""
Tool to convert from logfile of GPS-measurements to RINEX format

Usage: gnsslogger_to_rnx logfile <options>

Example using sample data file: 
    
    gnsslogger_to_rnx ../data/gnss_log.txt

See main() below for a list of command line options
"""

import argparse
import os, sys

import gnsslogger as alogger
import rinex3 as arinex


def convert2rnx(args):
    
    errFile = args.input_log[:-4] + '.trc'
    sys.stderr = open(errFile, "w")
    
    gnsslog = alogger.GnssLog(args.input_log)


    # Get all raw measurements
    raw_batches = [ b for b in gnsslog.raw_batches()]

    # Full bias nanos to be used in the process
    #fullbiasnanos = raw_batches[0][0]['FullBiasNanos'] if args.fix_bias else None
    #biasnanos = raw_batches[0][0]['BiasNanos'] if args.fix_bias else None

    # Get GLONASS freq channel, prn list, and code biases
    glo_freq_chns = alogger.get_glo_freq_chn_list(raw_batches)
    glo_cod_phs_bis = alogger.get_glo_cod_phs_bis_list(raw_batches)
    alogger.reset_clock()
    
    # Get phone model
    #model = gnsslog.header.parameters['Model']
    model = args.receiver_type
    print('Model = %s' % model, file = sys.stderr)


    # Process all batches of the file
    proc = lambda m : alogger.process(m,
                                      model=model,
                                      fix_bias=args.fix_bias,
                                      timeadj=float(args.timeadj),
                                      pseudorange_bias=args.pseudorange_bias,
                                      filter_mode=args.filter_mode,
                                      glo_freq_chns = glo_freq_chns,
                                      slip_mask = args.slip_mask)

    batches = [alogger.merge([proc(m) for m in rm]) for rm in raw_batches]

    # Get a list of the available observations
    obslist = alogger.get_obslist(raw_batches)

    # find first valid epoch
    for batch in batches:
        try:
            firstepoch=batch['epoch'][0]
            break
        except:
            pass

    # Write header and body
    header = arinex.write_header(obslist,
                                 firstepoch=firstepoch,
                                 lastepoch=batches[-1]['epoch'][0],
                                 markername=args.marker_name,
                                 observer=args.observer,
                                 agency=args.agency,
                                 rec=args.receiver_number,
                                 rec_type=args.receiver_type,
                                 rec_version=args.receiver_version,
                                 antenna=args.antenna_number,
                                 ant_type=args.antenna_type,
                                 pos=[0.0, 0.0, 0.0],
                                 hen=[0.0, 0.0, 0.0],
                                 glo_slot_freq_chns=glo_freq_chns,
                                 glo_cod_phs_bis=glo_cod_phs_bis)
    body = ''.join([arinex.write_obs(b, obslist) for b in batches])

    # Write to output
    if args.output is None:
        outFile = args.input_log[:-4] + '.obs'
        with open(outFile, "w") as fh:
            fh.write(header + body)
    else:
        outFile = os.path.join(os.path.dirname(args.input_log), args.output)
        with open(outFile, "w") as fh:
            fh.write(header + body)
        #print(args.output, outFile)
        
#sys.stderr.close()


if __name__ == "__main__":

    # Parse command line
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument('input_log', metavar='<input log file>', type=str,
                        help="Log file as recorded by the Google's Android app GnssLogger")
    parser.add_argument('--output', '-o', metavar='<output rinex file>', type=str, default=None,
                        help="Output RINEX file. If not set (default), RINEX will be written to the standard output")
    parser.add_argument('--marker-name', '-m', metavar='<marker name>', type=str, default="UNKN",
                        help="Specify the marker name (station id)")
    parser.add_argument('--observer', '-n', metavar='<observer name>', type=str, default="UNKN",
                        help="Specify the observer name or e-mail")
    parser.add_argument('--agency', '-a', metavar='<agency name>', type=str, default="UNKN",
                        help="Specify the agency name")
    parser.add_argument('--receiver-number',  metavar='<str>', type=str, default="UNKN",
                        help="Specify the receiver number")
    parser.add_argument('--receiver-type',  metavar='<str>', type=str, default="UNKN",
                        help="Specify the receiver type")
    parser.add_argument('--receiver-version',  metavar='<str>', type=str, default="AndroidOS >7.0",
                        help="Specify the receiver version")
    parser.add_argument('--antenna-number',  metavar='<str>', type=str, default="UNKN",
                        help="Specify the antenna number")
    parser.add_argument('--skip-edit', dest='skip_edit', action='store_true',
                        help="Skip pseudorange data edit that checks that the range is within bounds")
    parser.add_argument('--pseudorange-bias',  metavar='<double>', type=float, default=0,
                        help="Define a pseudorange bias to substract the range."
                        "This might be useful when the TOW has not been "
                        "decoded properly from the GNSS log. Default is 0. "
                        "Values must be specified in meters.")
    parser.add_argument('--antenna-type',  metavar='<str>', type=str, default="internal",
                        help="Specify the receiver type")
    parser.add_argument('--fix-bias', '-b', dest='fix_bias', action='store_true',
                        help="FIx and hold FullBiasNanos. Use this flag to take "
                        "the first FullBiasNanos and fix it during all data "
                        "take. This will avoid pseudorange jumps that would "
                        "appear if this option is not used. Note that in some "
                        "cases, it has detected that, while the pseudorange does "
                        "have these jumps, the carrier phase does not have it.")
    parser.add_argument('--time-adj', '-t', dest='timeadj', default=1e-7,
                        help="Adjust epochs to nearest interval. If "+
                             "selected, the range rate will be used to refer "+
                             "the range to the integer epoch as well and thus, "+
                             "maintain the consistency between time stamp and "+
                             "measurement. By default, this option is set to 100 nsec")
    parser.add_argument('--slip-mask', '-s', dest='slip_mask', default=3,
                        help="Maskfor slip and half cycle, 1=enable slip, 2 = enable half cycle," 
                             "3= enable both, Default=3")
    parser.add_argument('--filter-mode',  metavar='<str>', type=str, default="sync",
                        help="Specify the filtering mode for the data. Options include"
                             "sync: TOW/TOD known, code locked, no ambiguities detected, and all remaining flags for the signal are set"
                             "trck: TOW/TOD known, code locked and no ambiguities are detected")

    args = parser.parse_args()
    
                      
    convert2rnx(args)
    #sys.stderr.close()


