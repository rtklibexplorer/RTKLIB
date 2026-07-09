from compare_ppc import parse_nmea_full_ts

# Files
NMEA_FILE = r"C:\gps\data\PPC\PPC-Dataset\mrtklib_results\nagoya_run3_rtk.nmea"
OUT_FILE  = r"C:\gps\data\PPC\PPC-Dataset\mrtklib_results\mrtk_nagoya_run3_rtklib.pos"

# Set these for the PPC run date
GPS_WEEK = 2324
UTC_TO_GPST_OFFSET = 18.0   # seconds, valid for recent PPC data


with open(OUT_FILE, "w", newline="") as fo:
    fo.write("%  GPST          latitude(deg) longitude(deg)  height(m)   Q  ns   "
             "sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio\n")

    for week, tow, utc_sod, lat, lon, height, q, ns in parse_nmea_full_ts(NMEA_FILE):
        ratio = 0.0

        fo.write(f"{week:4d} {tow:10.3f} "
                 f"{lat:15.9f} "
                 f"{lon:15.9f} "
                 f"{height:10.4f} "
                 f"{q:3d} {ns:3d} "
                 f"{0.0000:8.4f} {0.0000:8.4f} {0.0000:8.4f} "
                 f"{0.0000:8.4f} {0.0000:8.4f} {0.0000:8.4f} "
                 f"{0.00:6.2f} {ratio:6.1f}\n")