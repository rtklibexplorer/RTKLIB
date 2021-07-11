#!/usr/bin/env python3
# -*- coding: iso-8859-1 -*-
# /opt/RTKLIB/app/rtkrcv/gcc/rtkrcv -s -o /opt/rtl.conf -m 5008 -d /dev/null >/dev/null 2>&1 &
# HIGHLY Recommended: https://csvkit.readthedocs.io/en/latest/
#import pandas as pd
#df = pd.read_csv('meine_datei.csv')
#df = df.drop(df.index[500:550]) # welche zeilen auch immer du loswerden willst. Ab 0 gezaehlt!
#df.to_csv("meine_datei_ohne_500-500.csv")

from gps import *
from optparse import OptionParser
from pyproj import Proj, transform
from math import radians, sin, cos, tan, atan, hypot, degrees, atan2, sqrt, pi
from datetime import datetime, timedelta
from socket import error as SocketError
from dateutil.relativedelta import relativedelta
from datetime import datetime
import errno,subprocess,os,time,sys,configparser,io,shlex,time,getopt,argparse,textwrap,socket,math,pyproj,csv,json,asyncio,dateparser,pandas as pd

# Global Variables
devnull = open(os.devnull, "wb")
lat=lng=epv=alt=0
input_EPSG  = "4978"  # 4978  = Geodetic CRS -> Earth centred, earth fixed, righthanded 3D coordinate system, consisting of 3 orthogonal axes with X and Y axes in the equatorial plane, positive Z-axis parallel to mean earth rotation axis and pointing towards North Pole.
output_EPSG = "25832" # 25832 = ETRS1989 UTM 32N
input_proj = pyproj.Proj("EPSG:"+input_EPSG, nadgrids='@/var/www/html/grid/BWTA2017.gsb,@/var/www/html/grid/BETA2007.gsb,null')
output_proj = pyproj.Proj("EPSG:"+output_EPSG)
# os.environ["PROJ_DEBUG"] = '0'
# x_out, y_out, z_out = pyproj.transform(input_proj, output_proj, x_in, y_in, z_in)

# NOTE Start RTK GNSS receiver
def Start_server():
    print("INFO: Start RTK GNSS receiver ...")
    rtkargs=[]
    rtkargs.append('/opt/RTKLIB/app/rtkrcv/gcc/rtkrcv -s -o ')
    rtkargs.append(CFILE)
    rtkargs.append(' -m ')
    rtkargs.append(str(args.mon))
    rtkargs.append(' -d /dev/null')
    rtkargs=''.join(rtkargs)
    rtkargs=shlex.split(rtkargs)
    rtkproc = subprocess.Popen(rtkargs, stdout=devnull, stderr=devnull)
    time.sleep(3)

def stop_server():
    # Läuft der Streamserver bereits?
    pid = os.popen("pidof rtkrcv").read()
    if pid: 
        os.system("kill " + str(pid))
        time.sleep(5)

def chkproj():
    x_in = 4138451.590 
    y_in = 640012.068  
    z_in = 4795041.866
    print(x_in, y_in, z_in)
    x_out, y_out, z_out = pyproj.transform(input_proj, output_proj, x_in, y_in, z_in)
    print(x_out, y_out, z_out)

# NOTE based on:
#      You, Rey-Jer. (2000). Transformation of Cartesian to Geodetic Coordinates without Iterations.
#      Journal of Surveying Engineering. doi: 10.1061/(ASCE)0733-9453
def ecef2UTM(x,y,z):
    r = sqrt(x ** 2 + y ** 2 + z ** 2)
    E = sqrt(6.378137E+6 ** 2 - 6.356752314E+6 ** 2)

    # eqn. 4a
    u = sqrt(0.5 * (r ** 2 - E ** 2) + 0.5 * sqrt((r ** 2 - E ** 2) ** 2 + 4 * E ** 2 * z ** 2))
    Q = hypot(x, y)
    huE = hypot(u, E)

    # eqn. 4b
    try:
        Beta = atan(huE / u * z / hypot(x, y))
    except ZeroDivisionError:
        if z >= 0:
            Beta = pi / 2
        else:
            Beta = -pi / 2

    # eqn. 13
    eps = ((6.356752314E+6 * u - 6.378137E+6 * huE + E ** 2) * sin(Beta)) / (
        6.378137E+6 * huE * 1 / cos(Beta) - E ** 2 * cos(Beta)
    )

    Beta += eps
    # %% final output
    lat = atan(6.378137E+6 / 6.356752314E+6 * tan(Beta))
    lon = atan2(y, x)

    lat = degrees(lat)
    lon = degrees(lon)

    ZONE_LETTERS = "CDEFGHJKLMNPQRSTUVWXX"
    if -80 <= lat <= 84:
        ZL = ZONE_LETTERS[int(lat + 80) >> 3]
    else:
        ZL = None

    if 56 <= lat < 64 and 3 <= lon < 12:
        ZN = 32

    if 72 <= lat <= 84 and lon >= 0:
        if lon < 9:
            ZN = 31
        elif lon < 21:
            ZN = 33
        elif lon < 33:
            ZN = 35
        elif lon < 42:
            ZN = 37
    else:
            ZN = int((lon + 180) / 6) + 1

    return str(ZN)+ZL

# NOTE Readline from RTK TCP server
async def tcp_echo_client(*server_address):
    reader, writer = await asyncio.open_connection(*server_address)  
    data = await reader.readline()
    writer.close()
    return data.decode()

# NOTE START Command line parsing
parser = argparse.ArgumentParser(
formatter_class=argparse.RawDescriptionHelpFormatter,description=textwrap.dedent('''
Python script to control RTK Solution Server
'''))
parser.add_argument('action', choices=['start', 'stop', 'restart'], nargs='?', help="[start]/stop/restart RTK service",)
parser.add_argument("-c", "--conf", type=str, nargs=1, help="config file [/opt/rtl.conf]")
parser.add_argument("-m", "--mon", type=int, nargs=1, help="monitor port [5008]")
parser.add_argument("-r", "--rtk", type=str, nargs=1, help="RTK data output file [/var/www/html/rtksvr.csv]")
parser.add_argument("-o", "--obs", type=str, nargs=1, help="Sat visibility output file [/var/www/html/strobs.csv]")
parser.set_defaults(action='start',conf='/opt/rtl.conf',mon=5008,rtk='/var/www/html/rtksvr.csv',obs='/var/www/html/strobs.csv')
args = parser.parse_args()
PJSON='/var/www/html/rtkpos.json'

# NOTE Action switcher
posRequest=False
if args.action == 'stop':
  stop_server()
  print("INFO: Stream Server Stopped!")
  sys.exit(0)

# NOTE Config file
CFILE = args.conf
if not os.path.isfile(CFILE):
    print("ERROR: RTK Configuration file missing!")
    sys.exit(1)

# NOTE Output Data as CSV File
DataFields=['Timestamp', 'X', 'Y', 'Z', 'UTMZ', 'UTMR', 'UTMH', 'ALT', 'R95', 'PDOP', 'SDX', 'SDY', 'SDZ','Q','NS','RATIO']
if not os.path.isfile(args.rtk):
    with open(args.rtk, 'a') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=DataFields)
        writer.writeheader()
        csvfile.close()

DataFieldsB=['Timestamp', 'az', 'el', 'cno', 'PRN']
if not os.path.isfile(args.obs):
    with open(args.obs, 'a') as csvfileB:
        writerB = csv.DictWriter(csvfileB, fieldnames=DataFieldsB)
        writerB.writeheader()
        csvfileB.close()

# NOTE Stream-Server + GPS-Service are ready?
pid = os.popen("pidof str2str").read()
if not pid: 
  print("ERROR: Stream Server not running!")
  sys.exit(1)
pid = os.popen("pidof gpsd").read()
if not pid: 
  print("ERROR: GPS service not running!")
  sys.exit(1)

stop_server()
Start_server()

# NOTE RTK receiver streaming socket
rtksvr_address = ('localhost', 5005)

# NOTE Control Loop
print("INFO: Enter Control Loop ...")
CycleTime=datetime.now()
while True:
    try:
        data = asyncio.run(tcp_echo_client(*rtksvr_address))
    except:
        data = ""
        print("INFO: Reconnect")
    time.sleep(.5)
    DArr = repr(data).split(";")
    #    [0]	    [1]	            [2]           [3]           [4] [5]  [6]      [7]      [8]     [9]      [10]     [11]    [12]    [13]
    # %  GPST           x-ecef(m)      y-ecef(m)      z-ecef(m)   Q  ns   sdx(m)   sdy(m)   sdz(m)  sdxy(m)  sdyz(m)  sdzx(m) age(s)  ratio
    # 2106 601309   4138451.7897    640011.7954   4795039.4922   2   5   3.7878   1.7982   3.4559   0.7144   0.7729   3.0019   0.99    0.0
    # print "Received:", DArr[1], DArr[2], DArr[3]
    sdx, sdy, sdz = float(DArr[6]), float(DArr[7]), float(DArr[8])
    R95 = 2.0789 * (62 * sdy + 56 * sdx)
    PDOP = sqrt( sdx ** 2 + sdy ** 2 + sdz ** 2)						# print("R95=%.3f PDOP=%.4f"%(R95,PDOP))    
    x_out, y_out, z_out = pyproj.transform(input_proj, output_proj, DArr[1], DArr[2], DArr[3])	# print x_out, y_out, z_out
    if int(DArr[4]) == 1:
        wrj=True
        if os.path.isfile(PJSON): 		# Datei vorhanden?
            file_mtime = datetime.fromtimestamp(os.path.getmtime(PJSON))
            max_mtime = datetime.now() - timedelta(minutes=60)
            if file_mtime > max_mtime:		# Datei jünger als 60 Minuten? os.remove(PJSON)
                 wrj=False
        if wrj:
            jdata = {'TS': DArr[0][1:20].replace("/","-"),'ECEFX': DArr[1], 'ECEFY': DArr[2], 'ECEFZ': DArr[3], 'UTMZ': ecef2UTM(float(DArr[1]),float(DArr[2]),float(DArr[3])), 'UTMR': x_out, 'UTMH': y_out, 'ALT': z_out, 'R95': R95, 'PDOP': PDOP}
            with open(PJSON, 'w') as outfile:
                json.dump(jdata, outfile)
                outfile.close
    with open(args.rtk, 'a') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=DataFields)
        writer.writerow({'Timestamp': DArr[0][1:20].replace("/","-"), 'X': DArr[1], 'Y': DArr[2], 'Z': DArr[3], 'UTMZ': ecef2UTM(float(DArr[1]),float(DArr[2]),float(DArr[3])), 'UTMR': x_out, 'UTMH': y_out, 'ALT': z_out, 'R95': R95, 'PDOP': PDOP, 'SDX': DArr[6], 'SDY': DArr[7], 'SDZ': DArr[8],'Q': DArr[4], 'NS': DArr[5], 'RATIO': DArr[13]})
        csvfile.close

    # NOTE Look for GPS
    wrj=True
    if os.path.isfile(args.obs): 						# Datei vorhanden?
        file_mtime = datetime.fromtimestamp(os.path.getmtime(args.obs))
        max_mtime = datetime.now() - timedelta(seconds=30)
        if file_mtime > max_mtime:						# Datei jünger als 30 Sekunden? os.remove(PJSON)
            wrj=False
    if wrj:
        finished=False
        retrycnt=0
        gpsdd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)
        while not finished:
            retrycnt += 1
            try:
                nx = next(gpsdd)
            except SocketError as SE:
                if SE.errno != errno.ECONNRESET:
                    print("SOCKET ERROR!")
                print("ERROR: " + SE.errno)
            if nx['class'] == 'SKY':
                skyview = getattr(nx,'satellites')
                for sat in skyview:
                     if int(getattr(sat,'ss')) > 0:
                         with open(args.obs, 'a') as csvfileB:
                             writerB = csv.DictWriter(csvfileB, fieldnames=DataFieldsB)
                             writerB.writerow({'Timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S"), 'az': getattr(sat,'az'), 'el': getattr(sat,'el'), 'cno': getattr(sat,'ss'), 'PRN': getattr(sat,'PRN')})
                             csvfileB.close
                finished=True
            if retrycnt == 10:
                print("ERROR: Gave up looking for gpsd socket after 10 times")
                finished=True
    # NOTE Garbage Collection every 60 Minutes
    delta=datetime.now()-CycleTime
    if delta.seconds < 3600: 
        time.sleep(0.8)
    else:
        print("INFO: RTK Data Garbage collection.")
        CycleTime=datetime.now()
        try:
            df=pd.read_csv(args.rtk,parse_dates=['Timestamp'],date_parser=lambda x: datetime.strptime(x, '%Y-%m-%d %H:%M:%S'))
        except:
            df=pd.read_csv(args.rtk,skiprows=1,parse_dates=['Timestamp'],date_parser=lambda x: datetime.strptime(x, '%Y-%m-%d %H:%M:%S'))
        LastRec=df['Timestamp'].iloc[-2] 
        CutOff = LastRec + relativedelta( hours = -25 )
        df=df.loc[df['Timestamp'] >= CutOff]
        df.to_csv(args.rtk, index=False)

        try:
            df=pd.read_csv(args.obs,parse_dates=['Timestamp'],date_parser=lambda x: datetime.strptime(x, '%Y-%m-%d %H:%M:%S'))
        except:
            df=pd.read_csv(args.obs,skiprows=1,parse_dates=['Timestamp'],date_parser=lambda x: datetime.strptime(x, '%Y-%m-%d %H:%M:%S'))
        LastRec=df['Timestamp'].iloc[-2] 
        CutOff = LastRec + relativedelta( hours = -25 )
        df=df.loc[df['Timestamp'] >= CutOff]
        df.to_csv(args.obs, index=False)
        
sys.exit(0)

##########################
# Jonn Person's interpretation of dilution of precision values
# DOP	Rating	Description
# 1	Ideal	This is the highest possible confidence level to be used for applications demanding the highest possible precision at all times.
# 2-3	Excellent	At this confidence level, positional measurements are considered accurate enough to meet all but the most sensitive applications.
# 4-6	Good	Represents a level that marks the minimum appropriate for making business decisions. Positional measurements could be used to make reliable in-route navigation suggestions to the user.
# 7-8	Moderate	Positional measurements could be used for calculations, but the fix quality could still be improved. A more open view of the sky is recommended.
# 9-20	Fair	Represents a low confidence level. Positional measurements should be discarded or used only to indicate a very rough estimate of the current location.
# 21-50	Poor	At this level, measurements are inaccurate by as much as half a football field and should be discarded.
#########################

