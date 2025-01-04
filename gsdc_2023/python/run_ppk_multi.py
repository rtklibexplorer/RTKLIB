"""
run_ppk_multi.py - convert raw android files to rinex and run PPK solutions for GDSC_2023
data set with RTKLIB    
"""

import sys
import numpy as np
if 'rtklib-py/src' not in sys.path:
    sys.path.append('rtklib-py/src')
if 'android_rinex/src' not in sys.path:
    sys.path.append('android_rinex/src')

import os, shutil
from os.path import join, isdir, isfile, abspath
from glob import glob
from multiprocessing import Pool
import subprocess
import gnsslogger_to_rnx as rnx
from time import time

# set run parameters
maxepoch = None # max number of epochs, used for debug, None = no limit

# Set solution choices
ENABLE_PY = False        # Use RTKLIB-PY to generate solutions, not fully supported 
ENABLE_RTKLIB = True     # Use RTKLIB to generate solutions
OVERWRITE_RINEX = True  # overwrite existing rinex filex
OVERWRITE_SOL = False    # overwrite existing solution files

# specify location of input folder and files
#datadir = '../data/train_small'   # relative to python script
datadir = '../data/test'

# base and nav file locations are relative to obs files
#bases = ['slac', 'vdcy']  # San Jose, LA
bases = ['p222', 'torp']  # San Jose, LA
#basefiles = '../base.obs' # rinex3, use this for python only
navfiles = '../BRDC*MN.rnx' # navigation files with wild cards

# Setup for RTKLIB,  
binpath_rtklib  = r'C:\gps\GSDC_2023\demo5_tips\app\consapp\rnx2rtkp\bcc\Release_Build/_rnx2rtkp'
cfgfile_rtklib = '../config/gsdc_0127.conf' # path relative to python script
soltag_rtklib = 'd5_0518a' # postfix for solution file names

# Setup for rtklib-py - not supported in this notebook
#cfgfile = '../config/ppk_phone_0510.py'
soltag_py = '_py0510'  # not used

# create alternate config file name
cfgfile_rtklib_mi8 = cfgfile_rtklib[:-5] + '_mi8.conf'
cfgfile_rtklib_sam = cfgfile_rtklib[:-5] + '_sam.conf'
cfgfile_rtklib_sam900 = cfgfile_rtklib[:-5] + '_sam900.conf'
cfgfile_rtklib_inst = cfgfile_rtklib[:-5] + '_inst.conf'

# convert relative paths to absolute paths
datadir = abspath(datadir)
binpath_rtklib = abspath(binpath_rtklib)
cfgfile_rtklib = abspath(cfgfile_rtklib)
cfgfile_rtklib_mi8 = abspath(cfgfile_rtklib_mi8)
cfgfile_rtklib_sam = abspath(cfgfile_rtklib_sam)
cfgfile_rtklib_sam900 = abspath(cfgfile_rtklib_sam900)
cfgfile_rtklib_inst = abspath(cfgfile_rtklib_inst)


# Select phones to process
# all phones
PHONES = ['pixel4', 'pixel4xl', 'pixel5', 'pixel5a', 'pixel6pro', 'pixel7pro',
          'mi8', 'xiaomimi8',
          'sm-g988b', 'sm-g955f', 'sm-s908b', 'sm-a226b', 'sm-a600t',
          'sm-a325f', 'sm-a217m', 'sm-a205u','sm-a505g',  'sm-a505u', 
          'samsungs22ultra', 'samsunga325g', 'samsunga32', 'samsung21ultra']
          
          # phones in test set
# PHONES = ['pixel4', 'pixel4xl', 'pixel5', 'pixel6pro', 'pixel7pro',
#           'mi8', 'xiaomimi8',
#           'sm-g988b', 'sm-s908b', 'sm-a325f', 'sm-a505u', 'sm-a205u',
#           'samsunga325g', 'samsunga32']

config_mi8 = ['mi8', 'xiaomimi8']
config_inst = ['sm-a205u', 'sm-a505g', 'sm-a505u', 'sm-g955f', 'sm-a226b', 'sm-a600t']
config_sam = ['sm-a325f', 'sm-a217m', 'sm-a205u', 'sm-a505g',  'sm-a505u',
            'samsungs22ultra', 'samsunga325g', 'samsunga32', 'samsung21ultra']
config_sam900 = ['sm-g988b', 'sm-s908b']

no_fix_bias = ['sm-g955f', 'sm-s908b', 'sm-a226b', 
               'sm-a217m', 'sm-a205u', 'sm-a505g', 'sm-a505u', 'sm-a600t',
               'sm-a325f','samsunga325g', 'samsunga32']


# These are only for rtklib-py, see the bases.sta file described above for RTKLIB base locations
BASE_POS = {'slac' : [-2703116.3527, -4291766.8501, 3854248.1361],  # WGS84 XYZ coordinates
            'vdcy' : [-2497836.8748, -4654543.0665, 3563029.0635],
            'p222' : [-2689640.5799, -4290437.1653, 3865051.0923]}

# input structure for rinex conversion
class args:
    def __init__(self):
        # Input parameters for conversion to rinex
        self.slip_mask = 0 # overwritten below
        self.fix_bias = True
        self.timeadj = 1e-7
        self.pseudorange_bias = 0
        self.filter_mode = 'sync'
        # Optional hader values for rinex files
        self.marker_name = ''
        self.observer = ''
        self.agency = ''
        self.receiver_number = ''
        self.receiver_type = ''
        self.receiver_version = ''
        self.antenna_number = ''
        self.antenna_type = ''

# Copy and read config file
if ENABLE_PY:
    shutil.copyfile(cfgfile, '__ppk_config.py')
    import __ppk_config as cfg
    import rinex as rn
    import rtkcmn as gn
    from rtkpos import rtkinit
    from postpos import procpos, savesol

# function to convert single rinex file
def convert_rnx(folder, rawFile, rovFile, slipMask, fix_bias):
    print(folder,fix_bias)
    os.chdir(folder)
    argsIn = args()
    argsIn.input_log = rawFile
    argsIn.output = os.path.basename(rovFile)
    argsIn.slip_mask = slipMask
    argsIn.fix_bias = fix_bias
    argsIn.receiver_type = folder.split('\\')[-1]
    rnx.convert2rnx(argsIn)

# function to run single RTKLIB-Py solution
def run_ppk(folder, rovfile, basefile, navfile, solfile):
    # init solution
    os.chdir(folder)
    gn.tracelevel(0)
    nav = rtkinit(cfg)
    nav.maxepoch = maxepoch
    print(folder)

    # load rover obs
    rov = rn.rnx_decode(cfg)
    print('    Reading rover obs...')
    if nav.filtertype == 'backward':
        maxobs = None   # load all obs for backwards
    else:
        maxobs = maxepoch
    rov.decode_obsfile(nav, rovfile, maxobs)

    # load base obs and location
    base = rn.rnx_decode(cfg)
    print('   Reading base obs...')
    base.decode_obsfile(nav, basefile, None)
    
    # determine base location from original base obs file name
    if len(BASE_POS) > 1:
        baseName = glob('../*.2*o')[0][-12:-8]
        nav.rb[0:3]  = BASE_POS[baseName]
    elif nav.rb[0] == 0:
        nav.rb = base.pos # from obs file
        
    # load nav data from rover obs
    print('   Reading nav data...')
    rov.decode_nav(navfile, nav)

    # calculate solution
    print('    Calculating solution...')
    sol = procpos(nav, rov, base)

    # save solution to file
    savesol(sol, solfile)
    return rovfile

# function to run single RTKLIB solution
def run_rtklib(binpath_rtklib, cfgfile_rtklib, folder, rovfile, basefile, 
               navfile, solfile):
    # create command to run solution
    rtkcmd = ['%s' % binpath_rtklib, '-x', '0', '-y', '0', '-k', cfgfile_rtklib,
              '-o', solfile, rovfile, basefile, navfile]
    
    # use this command line for debug from console, run from path in folder variable
    rtkcmd_debug = '%s -x 0 -y 0 -k %s -o %s %s %s %s' % (binpath_rtklib, cfgfile_rtklib,
              solfile, rovfile, basefile, navfile)
    
    # run command
    os.chdir(folder)
    subprocess.run(rtkcmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)   

####### Start of main code ##########################

def main():

    # get list of data sets in data path
    datasets = np.sort(os.listdir(datadir))

    # loop through data set folders
    rinexIn = []
    ppkIn = []
    rtklibIn = []
    for dataset in datasets:
        for phone in PHONES:
            # skip if no folder for this phone
            folder = join(datadir, dataset, phone)
            if not isdir(folder):  
                continue
            os.chdir(folder)
            rawFile = join('supplemental', 'gnss_log.txt')
            rovFile = join('supplemental', 'gnss_log.obs')

            rinex = False
            # check if need rinex conversion
            if OVERWRITE_RINEX or not isfile(rovFile):
                # generate list of input parameters for each rinex conversion
                if phone in config_sam: # Use cycle slip flags for Samsung phones
                    slipMask = 3 # 1 to unmask recevier cycle slips
                else:
                    slipMask = 0
                slipMask = 0
                fix_bias = False if phone in no_fix_bias else True
                rinexIn.append((folder, rawFile, rovFile, slipMask, fix_bias))
                print(rawFile, '->', rovFile) 
                rinex = True
            
            # check if need to create PPK solution
            ixb = 1 if '-lax-' in dataset else 0
            basefiles = '../' + bases[ixb] + '*0.2*o'
            try:
                baseFile = glob(basefiles)[0]
                navFile = glob(navfiles)[0]
                solFile = rovFile[:-4] + soltag_py + '.pos'
                solFile_rtklib = join('supplemental', soltag_rtklib + '_' + 
                                      bases[ixb] + '.pos')
            except:
                print(folder,'  Error: Missing file')
                continue
            if ENABLE_PY and (OVERWRITE_SOL == True or len(glob(solFile)) == 0 
                              or rinex == True):
                # generate list of input/output files for each python ppk solution
                print('PY: ', join(dataset, phone))
                ppkIn.append((folder, rovFile, baseFile, navFile, solFile))
            if ENABLE_RTKLIB and (OVERWRITE_SOL == True or 
                        len(glob(solFile_rtklib)) == 0 or rinex == True):
                # generate list of input/output files for each rtklib ppk solution
                print('RTKLIB: ', join(dataset, phone))
                # use different config file for samsung phone
                if phone in config_mi8:
                    cfgfile = cfgfile_rtklib_mi8
                elif phone in config_inst:
                    cfgfile = cfgfile_rtklib_inst
                elif phone in config_sam: 
                    cfgfile = cfgfile_rtklib_sam
                elif phone in config_sam900: 
                        cfgfile = cfgfile_rtklib_sam900
                else:
                    cfgfile = cfgfile_rtklib
                #print(cfgfile)
                rtklibIn.append((binpath_rtklib, cfgfile,
                        folder, rovFile, baseFile, navFile, solFile_rtklib))

    if len(rinexIn) > 0:
        print('\nConvert rinex files...')
        # generate rinx obs files in parallel, does not give error messages
        with Pool() as pool: # defaults to using cpu_count for number of procceses
            res = pool.starmap(convert_rnx, rinexIn)
        # run sequentially, use for debug
        # for input in rinexIn:
        #     convert_rnx(input[0],input[1],input[2],input[3])

    if ENABLE_PY and len(ppkIn) > 0:
        print('Calculate rtklib-py solutions...')
        # run PPK solutions in parallel, does not give error messages
        with Pool() as pool: # defaults to using cpu_count for number of procceses
            res = pool.starmap(run_ppk, ppkIn)
        # run sequentially, use for debug
        # for input in ppkIn:
        #     run_ppk(input[0],input[1],input[2],input[3],input[4])

    if ENABLE_RTKLIB and len(rtklibIn) > 0:
        print('Calculate RTKLIB solutions...')
        # run PPK solutions in parallel, does not give error messages
        with Pool() as pool: # defaults to using cpu_count for number of procceses
            res = pool.starmap(run_rtklib, rtklibIn)
        # run sequentially, use for debug
        # for input in rtklibIn:
        #     run_rtklib(input[0],input[1],input[2],input[3],input[4],input[5],input[6])

if __name__ == '__main__':
    t0 = time()
    main()
    print('Runtime=%.1f' % (time() - t0))