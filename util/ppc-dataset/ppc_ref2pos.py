# Files
REF_FILE = r"C:\gps\data\PPC\PPC-Dataset\nagoya\run3\reference.csv"
OUT_FILE = r"C:\gps\data\PPC\PPC-Dataset\nagoya\run3\reference.pos"
#REF_FILE = r"C:\gps\data\PPC\PPC-Dataset\gnssplusplusresults\ppc_tokyo_run3_rtk.pos"
#OUT_FILE = r"C:\gps\data\PPC\PPC-Dataset\gnssplusplusresults\ppc_tokyo_run3_rtklib.pos"

import csv

with open(OUT_FILE, "w", newline="") as fo:
    fo.write("%  GPST          latitude(deg) longitude(deg)  height(m)   Q  ns   "
             "sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio\n")

    with open(REF_FILE, newline="") as fi:
        for r in csv.DictReader(fi):
            fo.write(f"{int(r['GPS Week']):4d} {float(r['GPS TOW (s)']):10.3f} "
                     f"{float(r['Latitude (deg)']):15.9f} "
                     f"{float(r['Longitude (deg)']):15.9f} "
                     f"{float(r['Ellipsoid Height (m)']):10.4f} "
                     f"{1:3d} {0:3d} "
                     f"{0.0000:8.4f} {0.0000:8.4f} {0.0000:8.4f} "
                     f"{0.0000:8.4f} {0.0000:8.4f} {0.0000:8.4f} "
                     f"{0.00:6.2f} {999.9:6.1f}\n")