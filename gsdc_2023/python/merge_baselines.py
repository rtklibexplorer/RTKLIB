"""
merge_baselines.py - merge multiple solutions using weighted averages, result to 'merged.csv' file  
"""

import numpy as np
import os

# specify solution folders to merge
datapath = r'C:\gps\GSDC_2023\data'
merge_files = [
    #'locations_train_small_01_27a.csv',
    #'locations_train_small_01_27b.csv'
    'locations_test_05_18a.csv',
    'locations_test_05_18b.csv'
    ]

# load solution data
os.chdir(datapath)
msec, ll, q, var, wgt = [], [], [], [], []
for file in merge_files:
    d = np.genfromtxt(file, delimiter=',',invalid_raise=False, skip_header=1, dtype=str)
    msec.append(d[:,1].astype(float))
    ll.append(d[:,2:5].astype(float))
    x = d[:,7:10].astype(float) 
    var.append(x**2) # / np.median(x)**2) # normalize variances
    q.append(d[:,5].astype(int))
msec = np.array(msec)
ll = np.array(ll)
var = np.array(var)
q = np.array(q)

# calculate weights from variances
wgt = np.divide(1, var[:,:,0] + var[:,:,1])
wgt[np.where(np.isinf(wgt))] = 1e-30 # remove any divide by zero errors

# calc weighted average locations of phones for all timestamps
for i in range(3):
    d[:,2+i] = np.average(ll[:,:,i], weights=wgt, axis=0).astype(str)
    d[:,7+i] = np.average(np.sqrt(var[:,:,i]), weights=wgt, axis=0).astype(str)
    #d[:,2+i] = np.median(ll[:,:,i], axis=0).astype(str)
    #d[:,7+i] = np.median(np.sqrt(var[:,:,i]), axis=0).astype(str)
d[:,5] = np.min(q ,axis=0).astype(str)

# save results
np.savetxt('merged.csv', d,'%s,%s,%s,%s,%s,%s,%s,%s,%s,%s',
           header = '    ')