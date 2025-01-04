""" create_baseline_csv_from_pos.py -  Create csv file PPK solution files using timestamps in reference file
    from individual RTKLIB solution files
"""

import os
from os.path import join, isfile
import numpy as np
from datetime import date

########### Input parameters ###############################

# specify data set and solution set
#DATA_SET = 'train_small'
DATA_SET = 'test'
SOL_TAG = 'd5_0518'
datapath = '../data' # relative to python script
rovfile = 'gnss_log'
hdrlen = 25    # 25 for RTKLIB, 1 for RTKLIB-py

# Specify which base station solutions to use, primary and backup, run once with each line
#bases = ['slac', 'vdcy', 'p222', 'torp']  # San Jose, LA, backup San Jose, backup LA
bases = ['p222', 'torp', 'p222', 'torp']   #

outThresh = 100   # max horizontal accuracy estimate

# Specify which phones to use
#phones in test set
# PHONES = ['pixel4', 'pixel4xl', 'pixel5', 'pixel6pro', 'pixel7pro',
#           'mi8', 'xiaomimi8',
#           'sm-g988b', 'sm-s908b', 'sm-a205u', 'sm-a325f', 'sm-a505u', 'sm-a205u',
#           'samsunga325g', 'samsunga32']
PHONES = []  # use all phones

# Use filtered solutions for these rides since they are problematic
useFilterRides = ['2022-10-06-20-46-us-ca-sjc-r/sm-a205u',  # test
                  '2023-05-09-23-10-us-ca-sjc-r/sm-a505u', 
                  
                 '2023-05-23-19-56-us-ca-mtv-ie2/sm-a505g', ## train
                 '2023-05-23-19-56-us-ca-mtv-ie2/sm-a600t'
                  ]

# Also make sure the appropriate reference file is in the datapath
#  test: best_submission.csv - best available sample submission
# train: ground_truths_train.csv - created with create_ground_truths.py

############################################################

GPS_TO_UTC = 315964782  # differece in seconds

def create_csv(datapath, DATA_SET, SOL_TAG):
    # get timestamps from existing baseline file
    datapath = os.path.abspath(datapath)
    os.chdir(datapath)
    if DATA_SET[:5] == 'train':
        baseline_file = 'ground_truths_' + DATA_SET + '.csv'
    else: # 'test'
        baseline_file = 'best_submission.csv'
    # read data from baseline file
    base_txt = np.genfromtxt(baseline_file, delimiter=',',invalid_raise=False, 
                             skip_header=1, dtype=str)
    msecs_base = base_txt[:,1].astype(np.int64)
    phones_base = base_txt[:,0]
    pos_base = base_txt[:,2:4].astype(float) # baseline positions
    
    # open output file
    fout =open('locations_' + DATA_SET + '_' + date.today().strftime("%m_%d") + '.csv','w')
    fout.write('tripId,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n')
    
    # get list of data sets in data path
    os.chdir(join(datapath, DATA_SET))
    trips = np.sort(os.listdir())
    
    # loop through data set folders
    ix_b, npts = [], 0
    for trip in trips:
        if isfile(trip):
            continue
        phones = os.listdir(trip)
        # loop through phone folders
        for phone in phones:
            if isinstance(phone, bytearray):
                phone = phone.decode('utf-8')
            # check for valid folder and file
            folder = join(trip, phone)
            if isfile(folder):
                continue
            if PHONES != [] and phone not in PHONES:
                continue
            trip_phone = trip + '/' + phone
            print(trip_phone)
    
            ix_b = np.where(phones_base == trip_phone)[0]
            ixb = 1 if '-lax-' in trip else 0
            if trip_phone in useFilterRides:
                sol_path = join(folder, 'supplemental', SOL_TAG + '_' + bases[ixb] + '_filt.pos')
                print('    Use filt: ', trip_phone)
            else:
                sol_path = join(folder, 'supplemental', SOL_TAG + '_' + bases[ixb] +'.pos')
            fields = []
            if isfile(sol_path):
                # parse solution file
                fields = np.genfromtxt(sol_path, invalid_raise=False, skip_header=hdrlen)
            else:
                if trip_phone in useFilterRides:
                    sol_path = join(folder, 'supplemental', SOL_TAG + '_' + bases[ixb+2] + '_filt.pos')
                    print('   Use  backup filt: ', trip_phone)
                else:
                    sol_path = join(folder, 'supplemental', SOL_TAG + '_' + bases[ixb+2] +'.pos')
                if isfile(sol_path):
                    # parse solution file
                    fields = np.genfromtxt(sol_path, invalid_raise=False, skip_header=hdrlen)
                
            if len(fields) > 1:
                if int(fields[0,1]) > int(fields[-1,1]): # invert if backwards solution
                    fields = fields[::-1]
                pos = fields[:,2:5]
                qs = fields[:,5].astype(int)
                nss = fields[:,6].astype(int)
                acc = fields[:,7:10]
                msecs = (1000 * (fields[:,0] * 7 * 24 * 3600 + fields[:,1])).astype(np.int64)
                msecs += GPS_TO_UTC * 1000
            # if no data, use baseline data
            if not isfile(sol_path) or len(fields) == 0:
                print('Warning: data substitution: ', sol_path)
                msecs = msecs_base[ix_b].copy()
                pos = np.zeros((len(msecs), 3))
                pos[:,:2] = pos_base[ix_b].copy()
                qs = nss = np.zeros(len(msecs))
                acc = 1e9 * np.ones((len(msecs), 3))
           
            # interpolate to baseline timestamps to fill in missing samples
            llhs = []; stds = []
            for j in range(6):
                if j < 3:
                    llhs.append(np.interp(msecs_base[ix_b], msecs, pos[:,j]))
                    stds.append(np.interp(msecs_base[ix_b], msecs, acc[:,j]))
            qsi = np.interp(msecs_base[ix_b], msecs, qs)
            nssi = np.interp(msecs_base[ix_b], msecs, nss)
    
            # write results to combined file
            for i in range(len(ix_b)):
                fout.write('%s,%d,%.12f,%.12f,%.2f,%.0f,%.0f,%.3f,%.3f,%.3f\n' % 
                        (trip_phone, msecs_base[ix_b[i]], llhs[0][i], llhs[1][i],
                         llhs[2][i], qsi[i], nssi[i], stds[0][i], stds[1][i], 
                         stds[2][i]))
                try:
                    npts += len(fields)
                except:
                    pass
    
    fout.close()
    return npts

if __name__ == '__main__':
    create_csv(datapath, DATA_SET, SOL_TAG)
