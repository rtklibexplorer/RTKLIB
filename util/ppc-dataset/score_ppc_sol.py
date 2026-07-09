#!/usr/bin/env python3
# Score an RTKLIB .pos solution using PPC GNSS competition rules.

dataDir = r"C:\gps\data\PPC\PPC-Dataset"
REF_FILE = [
    r"\nagoya\run1\reference.csv",
    r"\nagoya\run2\reference.csv",
    r"\nagoya\run3\reference.csv",
    r"\tokyo\run1\reference.csv",
    r"\tokyo\run2\reference.csv",
    r"\tokyo\run3\reference.csv"]

# GNSSplusplus
# SOL_FILE = [
#     r"\gnssplusplusresults\ppc_nagoya_run1_rtklib.pos",
#     r"\gnssplusplusresults\ppc_nagoya_run2_rtklib.pos",
#     r"\gnssplusplusresults\ppc_nagoya_run3_rtklib.pos",
#     r"\gnssplusplusresults\ppc_tokyo_run1_rtklib.pos",
#     r"\gnssplusplusresults\ppc_tokyo_run2_rtklib.pos",
#     r"\gnssplusplusresults\ppc_tokyo_run3_rtklib.pos"]

# MRTKLIB
# SOL_FILE = [
#     r"\mrtklib_results\mrtk_nagoya_run1_rtklib.pos",
#     r"\mrtklib_results\mrtk_nagoya_run2_rtklib.pos",
#     r"\mrtklib_results\mrtk_nagoya_run3_rtklib.pos",
#     r"\mrtklib_results\mrtk_tokyo_run1_rtklib.pos",
#     r"\mrtklib_results\mrtk_tokyo_run2_rtklib.pos",
#     r"\mrtklib_results\mrtk_tokyo_run3_rtklib.pos"]

# RTKLIB_EX 2.5.1
#file = r'\0621b_comb_.pos'
file = r'\f9p_ppk_.pos'
#file = r'\b243_b.pos'
SOL_FILE = [
    r"\nagoya\run1"+ file,
    r"\nagoya\run2" + file,
    r"\nagoya\run3" + file,
    r"\tokyo\run1" + file,
    r"\tokyo\run2" + file,
    r"\tokyo\run3" + file]


MAX_ERR_M = 0.50       # PPC threshold: 3D error <= 50 cm
USE_DISTANCE_WEIGHTING = True   # True = PPC distance score, False = epoch percentage

import csv
import math
import matplotlib.pyplot as plt

def llh_to_ecef(lat_deg, lon_deg, h):
    a = 6378137.0
    f = 1.0 / 298.257223563
    e2 = f * (2.0 - f)
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    s = math.sin(lat)
    n = a / math.sqrt(1.0 - e2 * s * s)
    x = (n + h) * math.cos(lat) * math.cos(lon)
    y = (n + h) * math.cos(lat) * math.sin(lon)
    z = (n * (1.0 - e2) + h) * math.sin(lat)
    return (x, y, z)


def norm3(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

def read_rtklib_pos(path):
    sol = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if not line.strip() or line[0] == "%":
                continue

            p = line.split()
            if len(p) < 5:
                continue

            try:
                t = float(p[1])
                lat      = float(p[2])
                lon      = float(p[3])
                h        = float(p[4])
                sol.append((t, llh_to_ecef(lat, lon, h)))
            except ValueError:
                pass
    return sorted(sol)


def read_reference_csv(path):
    ref = []
    with open(path, newline="", encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            #t = float(row["GPS Week"]) * 604800.0 + float(row["GPS TOW (s)"])
            t = float(row["GPS TOW (s)"])
            lat = float(row["Latitude (deg)"])
            lon = float(row["Longitude (deg)"])
            h   = float(row["Ellipsoid Height (m)"])
            ref.append((t, llh_to_ecef(lat, lon, h)))
    return sorted(ref)

def calc_score(ref, sol, distance_weighted=False):
    j = 0
    last_sol = None

    good = 0.0
    total = 0.0
    good_epochs = 0
    total_epochs = 0

    t0 = ref[0][0]
    times = []
    errors = []

    for i in range(len(ref)):
        t, ref_xyz = ref[i]

        while j < len(sol) and sol[j][0] <= t:
            last_sol = sol[j]
            j += 1

        err = float("inf") if last_sol is None else norm3(last_sol[1], ref_xyz)

        times.append((t - t0) / 60.0)     # minutes
        errors.append(err)

        if distance_weighted:
            if i == 0:
                continue
            weight = norm3(ref[i][1], ref[i - 1][1])
        else:
            weight = 1.0

        total += weight
        total_epochs += 1

        if err <= MAX_ERR_M:
            good += weight
            good_epochs += 1

    score = 100.0 * good / total if total > 0 else 0.0

    return score, good, total, good_epochs, total_epochs, times, errors

# main code
total_good_ep = 0
total_ep = 0
route_scores = []
nfiles = len(REF_FILE)
for i in range(nfiles):
    ref = read_reference_csv(dataDir + REF_FILE[i])
    sol = read_rtklib_pos(dataDir + SOL_FILE[i])
    score, good, total, good_ep, total_ep_i, times, errors = calc_score(
    ref, sol, distance_weighted=USE_DISTANCE_WEIGHTING)
    
    route_scores.append(score)
    total_good_ep += good_ep
    total_ep += total_ep_i
    
    ref_file = REF_FILE[i].split('\\')
    print('%14s: %5.2f%%  (%d/%d, missing=%d)' % (ref_file[-3] + ' ' + ref_file[-2], score, good_ep, total_ep_i, len(ref)-len(sol)))
    
    plt.figure(figsize=(12,4))
    plt.plot(times, errors, '.', markersize=1)
    plt.axhline(0.5, color='r', linestyle='--')
    plt.xlabel('Time (minutes)')
    plt.ylabel('3D Error (m)')
    plt.title(ref_file[-3] + ' ' + ref_file[-2])
    plt.grid(True)
    plt.yscale('log')     # optional but usually helpful
    plt.show()



print()

if USE_DISTANCE_WEIGHTING:
    # PPC wording says route percentages are averaged.
    overall = sum(route_scores) / len(route_scores) if route_scores else 0.0
    print("Average route score: %6.2f%%" % overall)

else:
    overall = 100.0 * total_good_ep / total_ep if total_ep else 0.0
    print("Overall epoch score: %6.2f%%  (%d/%d epochs)" %
          (overall, total_good_ep, total_ep))