# Files
REF_FILE = r"C:\gps\data\PPC\PPC-Dataset\gnssplusplusresults\ppc_nagoya_run3_rtk.pos"
OUT_FILE = r"C:\gps\data\PPC\PPC-Dataset\gnssplusplusresults\ppc_nagoya_run3_rtklib.pos"

with open(OUT_FILE, "w", newline="") as fo:
    fo.write("%  GPST          latitude(deg) longitude(deg)  height(m)   Q  ns   "
             "sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio\n")

    with open(REF_FILE, newline="") as fi:
        for line in fi:
            if not line.strip() or line.startswith("%"):
                continue

            f = line.split()

            week   = int(f[0])
            tow    = float(f[1])
            lat    = float(f[5])
            lon    = float(f[6])
            height = float(f[7])
            q      = int(f[8])     # Status
            ns     = int(f[9])     # NumSat
            ratio  = float(f[11])  # Ratio

            fo.write(f"{week:4d} {tow:10.3f} "
                     f"{lat:15.9f} "
                     f"{lon:15.9f} "
                     f"{height:10.4f} "
                     f"{q:3d} {ns:3d} "
                     f"{0.0000:8.4f} {0.0000:8.4f} {0.0000:8.4f} "
                     f"{0.0000:8.4f} {0.0000:8.4f} {0.0000:8.4f} "
                     f"{0.00:6.2f} {ratio:6.1f}\n")