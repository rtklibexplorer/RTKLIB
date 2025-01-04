"""
filter_ride.py - filter RTKLIB solutions, only required for problematic solutions   
"""

import numpy as np
import os
from glob import glob
from os.path import join
from scipy.signal import savgol_filter
from savitzky_golay_werrors import savgol_filter_werror
from utm import from_latlon as ll2en
from utm import to_latlon as en2ll
import matplotlib.pyplot as plt
from numpy.linalg import norm
from copy import copy



# specify data locations

#datapath = r'C:\gps\GSDC_2023\data\train_small'  #'/*/*/supplemental'
datapath = r'C:\gps\GSDC_2023\data\test'
rides = '*' #  '2022-04-25-22-36-us-ca-ebf-z'
 

# specify which solutions to filter
datafile = 'd5_0518_????.pos' # wildcard expression for solution files 
phones = 'sm-a*u'              # wildcard expression for phone types
truthfile = 'span_log.nmea'
imufile = 'device_imu.csv'

# filter parameters
winpos_def = 0  # window size for pos filter
winpos = {'sm-a205u':9, 'sm-a505g':9, 'sm-a505u':9, 'sm-a600t':19}
winacc_def = 13 # window size for acc filter
winacc = {'mi8':21, 'sm-a205u':13, 'sm-a505g':13, 'sm-a505u':13, 'sm-a205u':13} 
order = 4 #2
out_x_def = 2.0  # position outlier threshold
out_x = {'mi8': 5, 'sm-a205u':100, 'sm-a325f':5, 'sm-a505g':5.0, 'sm-a505u':100, 
         'sm-a600t':2.0, 'samsunga325g':5.0, 'samsunga32':5.0}
out_v_def = 2.0  # velocity outlier threshold
out_v = {'mi8': 5, 'sm-a205u':100, 'sm-a325f':20.0, 'sm-a505g':5.0, 
         'sm-a505u':100, 'sm-a600t':5.0,
         'samsunga325g':20.0, 'samsunga32':20.0}
zero_acc_def = 0.01
zero_acc = {'mi8':0.15, 'sm-a205u':0.10, 'sm-a505g':0.10, 'sm-a505u':0.10, 'sm-a600t':0.10,
            'sm-a325f':0.10}
min_zacc_len = 12
zero_vel_def=0.05
zero_vel = {'sm-a205u':0.10, 'sm-a505g':0.30, 'sm-a505u':0.30, 
            'samsunga325g':0.25, 'samsunga32':0.25}

plots = True

sol_hdr = '%  GPST          latitude(deg) longitude(deg)  height(m)   Q  ns   sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio'
no_gt = datapath.split('\\')[-1] == 'test'

hdrlen=25
GPS_TO_UTC = 315964782
g = 9.807


for datafile in glob(join(datapath,rides,phones,'supplemental',datafile)):
    # parse solution file
    print(datafile)
    path = os.path.dirname(datafile)
    [ride, phone, _, file] = datafile.split('\\')[5:9]
    fields = np.genfromtxt(datafile, invalid_raise=False, skip_header=hdrlen)
    llhs = fields[:,2:5]
    vels = fields[:,15:18] # NEU
    qs = fields[:,5].astype(int)
    nss = fields[:,6].astype(int)
    stds = fields[:,7:10]  # NEU
    velstds = np.sqrt(fields[:,18:21])  # NEU
    secs0 = (fields[:,0] * 7 * 24 * 3600 + fields[:,1]) % (24*3600)
    toff = (fields[0,1]-fields[0,1] % (24*3600)) % 10000
    
    # parse ground truth file
    if  not no_gt:
        fields_gt = np.genfromtxt(join(path, truthfile), invalid_raise=False, dtype=str)
        ix = np.where(np.char.startswith(fields_gt,'$GPGGA'))[0]
        fields_gt = np.vstack(np.char.split(fields_gt[ix], ','))
        secs_gt=[]
        llhs_gt=[]
        for field in fields_gt:
            d = field[1]
            t = int(d[0:2])*3600 + int(d[2:4])*60 + float(d[4:])+18
            secs_gt.append(t)
            lat = field[2]
            lon = field[4]
            hgt = field[9]
            llhs_gt.append([int(lat[0:2]) + float(lat[2:])/60, 
                         -(int(lon[0:3]) + float(lon[3:])/60), float(hgt)])
    else:
        llhs_gt = llhs
        secs_gt = secs0
    
    # parse imu file
    fields_acc = np.genfromtxt(join(os.path.dirname(path),imufile), invalid_raise=False, skip_header=1, dtype=str)
    ix = np.where(np.char.startswith(fields_acc,'UncalAccel'))[0]
    fields_acc = np.vstack(np.char.split(fields_acc[ix], ','))
    secs_acc = (fields_acc[:,1].astype(float)/1000 - GPS_TO_UTC) % (24*3600)
    accels = fields_acc[:,3:6].astype(float)
    
    # convert llh to enu
    neus0 = np.zeros((len(llhs),3))
    neus2 = np.zeros((len(llhs),3))
    neus0[:,1], neus0[:,0], zone, NS = ll2en(llhs[:,0], llhs[:,1])
    neus0[:,2] = llhs[:,2]
    neus_gt = np.zeros((len(llhs_gt),3))
    llhs_gt = np.array(llhs_gt)
    neus_gt[:,1], neus_gt[:,0], zone, NS = ll2en(llhs_gt[:,0], llhs_gt[:,1])
    neus_gt[:,2] = llhs_gt[:,2]
    
    # find outliers
    xstds = np.sqrt(stds[:,0]**2 + stds[:,1]**2)
    max_x = out_x.get(phone, out_x_def)
    ix = np.where(xstds > max_x)[0]  # find outliers
    vstds = np.sqrt(velstds[:,0]**2 + velstds[:,1]**2)
    max_v = out_v.get(phone, out_v_def)
    iv = np.where(vstds <= max_v)[0]  # find non-outliers
    
    # interpolate velocity outliers
    if len(iv) < len(vstds):
        for i in range(3):
            vels[:,i] = np.interp(secs0, secs0[iv], vels[iv,i]) # fill in velocity outliers

    
    # create continuous time vector
    ts = np.arange(max(secs0[0],secs_gt[0]), min(secs0[-1],secs_gt[-1]))
    dt = ts[1] - ts[0]
    npts = len(ts)
    
    # fill in missing position outliers with velocity
    neus1 = copy(neus0)
    for i in ix: # step through position outliers
        if i==0: continue
        neus1[i] = neus1[i-1] + vels[i] * (secs0[i] - secs0[i-1])
        #print('%.0f %d: %f.2 -> %f.2' % (secs0[i]+toff,i,neus0[i,0], neus1[i,0]))
        
    
    # initialize arrays
    v_i = np.zeros((npts,3))
    xgt_i = np.zeros((len(ts),3))
    x0_i = np.zeros((npts,3)); x1_i = np.zeros((npts,3))
    dx0_i = np.zeros((npts,3)); dx1_i = np.zeros((npts,3))
    x1_i_f = np.zeros((npts,3)); dx1_i_f = np.zeros((npts,3))
    accs = np.zeros((npts,3))
    
    
    # process data
    for i in range(3):
        # interpolate pos
        x0_i[:,i] = np.interp(ts, secs0, neus0[:,i]) # raw position
        x1_i[:,i] = np.interp(ts, secs0, neus1[:,i]) # position with outliers removed
        # filter pos
        winp = winpos.get(phone, winpos_def)
        if winp > 0:
            x1_i_f[:,i] = savgol_filter(x1_i[:,i], winp, order, deriv = 0)
        else:
            x1_i_f[:,i] = copy(x1_i[:,i])
        xgt_i[:,i] = np.interp(ts, secs_gt, neus_gt[:,i])  # ground truth
        accs[:,i] = np.interp(ts, secs_acc, accels[:,i])
    
        v_i[:,i] = np.interp(ts, secs0, vels[:,i])
        v_i[:,i] = np.convolve(v_i[:,i],[0.5,0.5],'same') # shift half sample
        #std_i = np.interp(ts, secs0, stds[:,i],left=1000, right=1000)
    
          
    # process acceleration
    acc_mag = np.abs(norm(np.abs(accs), axis=1) - g)
    vel_mag = norm(v_i, axis=1)
    wina = winacc.get(phone, winacc_def)
    acc_magf = savgol_filter(acc_mag, wina, order, deriv = 0)
    zacc = zero_acc.get(phone, zero_acc_def)
    zvel = zero_vel.get(phone, zero_vel_def)
    ix_acc0 = np.where((abs(acc_magf) < zacc) | (abs(vel_mag) < zvel))[0]
    ix_acc0 = np.unique((ix_acc0-1, ix_acc0, ix_acc0+1))[1:-2]
    zstart = 0; zlen = 0; nz = 0 
    for i in range(1,len(ix_acc0)):
        if ix_acc0[i] == ix_acc0[i-1] + 1: # continuation of zero accel interval
            zlen += 1
        else: # break in zero accel 
            if zlen >= min_zacc_len + 2: # check length of zero accel interval
                #x1_i[zstart:zstart+zlen,:] = np.median(x1_i[zstart:zstart+zlen,:], axis=0)
                x1_i_f[zstart:zstart+zlen,:] = np.median(x1_i_f[zstart:zstart+zlen,:], axis=0)
                #print('zeroacc', zstart, zlen)
                nz += zlen
            zstart = ix_acc0[i]
            zlen = 0
            
            
    print('pos outliers=%d, vel outliers=%d zero_acc=%d' % (len(ix), len(vels) - len(iv), nz))     
            
    # generate velocity data
    for i in range(3):
        dx0_i[:,i] = np.hstack((0, np.diff(x0_i[:,i])))
        dx1_i[:,i] = np.hstack((0, np.diff(x1_i[:,i])))
        dx1_i_f[:,i] = np.hstack((0, np.diff(x1_i_f[:,i])))
    
    # convert filtered enu back to llh and save result
    for i in range(3):
        neus2[:,i] = np.interp(secs0, ts, x1_i_f[:,i]).astype(str)
    fields[:,2:4] = np.array(en2ll(neus2[:,1], neus2[:,0], zone, NS)).T.astype(str)
    np.savetxt(datafile[:-4] + '_filt.pos', fields,'%s',
               header = sol_hdr)
    
    # filter by convolution
    #for i in range(3):
        # dx0_i_f[:,i] = np.convolve(dx0_i[:,i], [.5, 0, .5], 'same')
        # errs[:,i] = (dx0_i[:,i] - dx0_i_f[:,i])
        # errs[np.abs(errs[:,i])<1,i] = 0
        #x1_i_f[1:-1,i] = np.convolve(x1_i_f[:,i], [.25, .5, .25], 'valid')
    
    # calculate errors
    errs0 = x0_i - xgt_i
    errs1 = x1_i - xgt_i
    errs1f = x1_i_f - xgt_i
    herrs0 = np.sort(np.sqrt(errs0[:,0]**2 + errs0[:,1]**2))
    herrs1 = np.sort(np.sqrt(errs1[:,0]**2 + errs1[:,1]**2))
    herrs1f = np.sort(np.sqrt(errs1f[:,0]**2 + errs1f[:,1]**2))

    ix50 = int(np.round(npts*.5))
    ix95 = int(np.round(npts*.95))
    
    # plot results
    if plots:
        ylabs = ['North', 'East', 'Up']
        fig = plt.figure(figsize=(10,8))
        fig.suptitle(ride + ' ' + phone + ' ' + file[-8:-4])
        for i in range(2):
            ax = fig.add_subplot(2, 2, i + 1)
            # calculate velocities
            vgt_xi = np.hstack((0, np.diff(xgt_i[:,i])))
        

            ax.plot(ts+toff, dx0_i[:,i], 'o-', label = 'dpos')
            ax.plot(ts+toff, dx1_i[:,i], 'x-', label = 'dpos no out')
            ax.plot(ts+toff, dx1_i_f[:,i], '.-', label = 'dpos filt')
            #ax.plot(ts+toff, errs[:,i], '.-', label = 'errs')
            ax.plot(ts+toff, vgt_xi, '.-', label = 'dpos gt')
            ax.plot(ts+toff, acc_mag, '.-', label = 'accel')
            ax.plot(ts+toff, acc_magf, '.-', label = 'accel filt')
            ax.plot(ts+toff, v_i[:,i], '.-', label = 'vel')
            ax.grid('on')
            ax.legend(loc = 3, frameon=0, ncol=2, fontsize=13)
            ax.set_ylabel(ylabs[i])
            #ax.set_xticklabels([])
            
            if not no_gt:
                ax = fig.add_subplot(2, 2, i + 3)
                ax.plot(ts+toff, errs0[:,i], 'o-', label = 'xerr %.2f' % ((herrs0[ix50] + herrs0[ix95]) / 2))
                ax.plot(ts+toff, errs1[:,i], 'x-', label = 'xerr no out %.2f' % ((herrs1[ix50] + herrs1[ix95]) / 2))
                ax.plot(ts+toff, errs1f[:,i], '.-', label = 'xerr filt %.2f' % ((herrs1f[ix50] + herrs1f[ix95]) / 2))
                ax.grid('on')
                ax.legend(loc = 3, frameon=0, ncol=2, fontsize=13)
                ax.set_ylabel(ylabs[i])
