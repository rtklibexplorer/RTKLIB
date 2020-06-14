#!/usr/bin/env python
# -*- coding: iso-8859-1 -*-

from gps import *
from optparse import OptionParser
import subprocess,os,sys,ConfigParser,io,shlex,time,getopt,argparse,textwrap,datetime,csv
from datetime import datetime, timedelta

# Global Variables
devnull = open(os.devnull, "wb")
lat=lng=epv=alt=0
CDATAFILE='/var/www/html/strobs.csv'

def stop_server():
  # Läuft der Streamserver bereits?
  pid = os.popen("pidof str2str").read()
  if pid: 
    os.system("kill " + str(pid))

# Command Line Parser
parser = argparse.ArgumentParser(
formatter_class=argparse.RawDescriptionHelpFormatter,description=textwrap.dedent('''
Python script to conrol RTKlib Stream Server
'''))
parser.add_argument('action', choices=['start', 'stop', 'update'],help="start/stop/update stream server")
group = parser.add_argument_group('Base station position')
group.add_argument('lat', metavar="X", action='store', type=float, nargs='?', help='ECEF X | [Latitude]')
group.add_argument('lng', metavar="Y", action='store', type=float, nargs='?', help='ECEF Y | [Longitude]')
group.add_argument('alt', metavar="Z", action='store', type=float, nargs='?', help='ECEF Z | [Altitude]')
parser.add_argument('-ecef', action='store_true', help="use ECEF instead of LLH coordinate system")
parser.add_argument("-c", "--conf", type=str, nargs=1, help="config file")
parser.add_argument("-g", "--gps", type=str, nargs=1, help="GNSS device [auto detect]")
args = parser.parse_args()

# GPS Device
if args.gps is None:
 Serial = os.popen("find /sys/devices -iname 'ttyA*' -printf %f").read()
else:
 Serial = args.gps[0]
if Serial is "":
 print "ERROR: No GPS device found or set."
 sys.exit(2)

# Config file
if args.conf is None:
 CFILE='/opt/RTKLIB/misc/strstr.ini'
else:
 CFILE = args.conf[0]
config = ConfigParser.RawConfigParser()
config.read(CFILE)
 
# Action switcher
posRequest=False
if args.action == 'stop':
  stop_server()
  print "INFO: Stream Server Stopped!"
  sys.exit(0)

# Action: START + UPDATE
if args.lat is None or args.lng is None or args.alt is None:
  if args.action == 'update':
    print "ERROR: Coordinates Missing!"
    sys.exit(2)
  posRequest=True

# Retrieve Position
if posRequest:
   # PHASE 1 - Setup Stream Server for initalization 
   print("INFO: Init Stream Server for initial position ...")
   args=[]
   args.append('/opt/RTKLIB/app/str2str/gcc/str2str -c ')
   args.append(config.get('gps', 'init_script'))
   args.append(' -in serial://')
   args.append(Serial)
   args.append(':115200:8:n:1:off#ubx -out ')
   args.append(config.get('server','con1'))
   args=''.join(args)
   args=shlex.split(args)
   os.system("systemctl stop serial-getty@" + Serial + ".service")
   strproc = subprocess.Popen(args, stdout=devnull, stderr=devnull)
   os.system("systemctl restart gpsd.service")

   # PHASE 2 - Scan for coordinates

   def getPositionData(gps):
      global lat, lng, epv, alt
      nx = gpsd.next()
      # For a list of all supported classes and fields refer to:
      # https://gpsd.gitlab.io/gpsd/gpsd_json.html
      if nx['class'] == 'TPV':
         lat = getattr(nx,'lat', 0)
         lng = getattr(nx,'lon', 0)
         alt = getattr(nx,'alt', 0)
         epv = getattr(nx,'lon', 0)

   gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)

   print "INFO: Waiting for position ..."
   Countdown=0
   while Countdown < 60 :
      getPositionData(gpsd)
      time.sleep(1.0)
      Countdown+=1
      if epv > 0 and epv < 15 : break
      
   strproc.terminate()
   print "INFO: Receiver position: lon = " + str(lng) + ", lat = " + str(lat) + ", alt = " + str(alt) + ", epv = " + str(epv)
else:
   lat=args.lat
   lng=args.lng
   alt=args.alt
   
# PHASE 3 START Stream Server
args=[]
args.append('/opt/RTKLIB/app/str2str/gcc/str2str')
args.append(' -in serial://')
args.append(Serial)
args.append(':115200:8:n:1:off#ubx -out ')
args.append(config.get('server','con1'))
args.append(' -out ')
args.append(config.get('server','con2'))
args.append(' -out ')
args.append(config.get('server','con3'))
args.append(' -out ')
args.append(config.get('server','con4'))
args.append(' -msg ')
args.append(config.get('server','msg'))
args.append(' -opt ')
args.append(config.get('server','opt'))
args.append(' -p ')
args.append(str(lat) + " " + str(lng) + " " + str(alt))
args=''.join(args)
args=shlex.split(args)
print "INFO: Starting Continuously Operating Mobile GNSS Reference Station"
stop_server()
os.system("systemctl stop serial-getty@" + Serial + ".service")
strproc = subprocess.Popen(args, stdout=devnull, stderr=devnull)
os.system("systemctl restart gpsd.service")

sys.exit(0)
# PHASE 4 Log Observations -> rtksvr.py
DataFields=['Timestamp', 'az', 'el', 'cno', 'PRN']
if not os.path.isfile(CDATAFILE):
    with open(CDATAFILE, 'a') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=DataFields)
        writer.writeheader()
        csvfile.close()

gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)

print "INFO: Observing Satellites and if necessary I could watch out for a receiver position update ..."
while True :
    nx = gpsd.next()
    # For a list of all supported classes and fields refer to:
    # https://gpsd.gitlab.io/gpsd/gpsd_json.html
    if nx['class'] == 'SKY':
        skyview = getattr(nx,'satellites')
        for sat in skyview:
          if int(getattr(sat,'ss')) > 0:
             with open(CDATAFILE, 'a') as csvfile:
                 writer = csv.DictWriter(csvfile, fieldnames=DataFields)
                 writer.writerow({'Timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S"), 'az': getattr(sat,'az'), 'el': getattr(sat,'el'), 'cno': getattr(sat,'ss'), 'PRN': getattr(sat,'PRN')})
             csvfile.close
        time.sleep(30.0)

sys.exit(0)

