#!/bin/bash
# Software-in-the-loop test of the Crazyflie app logic on the desktop (see sil_test.c), followed by
# the flight analysis (host/analyze_flight.py) on the simulated logs, which use the flight-log format.
set -e
cd "$(dirname "$0")"
CXX=${CXX:-c++}; CC=${CC:-cc}
$CXX -std=c++17 -O2 -Wall -I../include -Istubs -DTSDP_MAX_OBS=4 -DTSDP_MAX_WORLD=8 -DTSDP_MEMORY=0 -c ../src/tsdp_planner.cpp -o /tmp/tsdp_planner.o
$CC -std=gnu11 -O2 -Wall -Istubs -I../src -c sil_test.c -o /tmp/tsdp_sil_test.o
$CXX /tmp/tsdp_sil_test.o /tmp/tsdp_planner.o -o /tmp/tsdp_sil_test
rm -rf /tmp/tsdp_sil_csv && mkdir -p /tmp/tsdp_sil_csv
SIL_CSV_DIR=/tmp/tsdp_sil_csv /tmp/tsdp_sil_test
for f in /tmp/tsdp_sil_csv/sil_*.csv; do python3 ../host/analyze_flight.py "$f" --json "${f%.csv}.json" > /dev/null; done
python3 - <<'PY'
import glob, json, sys
bad = []
for f in sorted(glob.glob("/tmp/tsdp_sil_csv/sil_*.json")):
    s = json.load(open(f))
    tc, dr = s["true_clearance_m"], s["delta_within_rho"]
    ok = s["frames"] > 0 and (tc["min"] is None or tc["samples_in_contact"] == 0) and (dr["fraction"] in (None, 1.0))
    if not ok: bad.append(f)
    rmin = s["rho_m"].get("min")
    print(f"  analysis {f.split('/')[-1]:30s} frames {s['frames']:4d}  min clearance {'n/a' if tc['min'] is None else format(tc['min'], '+.3f')} m"
          f"  rho min {'n/a' if rmin is None else format(rmin, '.3f')} m  delta<=rho {dr['fraction']}")
print("ANALYSIS PASS" if not bad else f"ANALYSIS FAIL: {bad}")
sys.exit(1 if bad else 0)
PY
