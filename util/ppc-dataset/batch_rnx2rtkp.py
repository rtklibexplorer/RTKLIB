'''
 batch_rnx2rtkp - template to run multiple simultaneous solutions of rnx2rtkp in Windows.
    This example is configured to run the PPC_dataset benchmark files at https://github.com/taroz/PPC-Dataset
 '''
 
import os
import subprocess
import psutil
import time
 
# set location of data and rnx2rtkp executable
datapaths = [r'C:\gps\data\PPC\PPC-Dataset\nagoya\run1',
        r'C:\gps\data\PPC\PPC-Dataset\nagoya\run2',
        r'C:\gps\data\PPC\PPC-Dataset\nagoya\run3',
        r'C:\gps\data\PPC\PPC-Dataset\tokyo\run1',
        r'C:\gps\data\PPC\PPC-Dataset\tokyo\run2',
        r'C:\gps\data\PPC\PPC-Dataset\tokyo\run3',
        ]
binpath = r'C:\gps\rtklib\bins\RTKLIB_EX_2.5.1'
 
# set input files
cfgs = [r'C:\gps\data\PPC\PPC-Dataset\tokyo\ppc_benchmark']  # list of config files to run (files should have .conf ext)
rovFile = 'rover.obs'  # rover files with wild cards
baseFile = 'base.obs' # base files with wild cards
navFile = 'base.nav' # navigation files with wild cards
outFile = ''
 
# set maximum number of simultaneous occurences of rnx2rtkp to run
max_windows = 10
 
# get list of current threads running in Windows
current_process = psutil.Process()
num_start = len(current_process.children())
 
# loop through data folders
for datapath in datapaths:
    #datepath = datapath + '/' + date
    print(datapath)
    os.chdir(datapath)

    # Run a solution for each config file in list       
    for cfg in cfgs:
        out = cfg.split('\\')[-1] + '_' + outFile
        # create command to run solution
        rtkcmd=r'%s\rnx2rtkp -x 0 -y 2 -k %s.conf -o %s.pos %s %s %s' % \
            (binpath, cfg, out, rovFile, baseFile, navFile)    
         
        # run command
        subprocess.Popen(rtkcmd)
 
 
    # if max windows open, wait for one to close
    while len(current_process.children())-num_start >= max_windows:
        time.sleep(1) #wait here for existing window to close
 
# Wait for all solutions to finish
print('Waiting for solutions to complete ...')  
while len(current_process.children())-num_start > 0:
    pass #wait here if max windows open        
print('Done')