#!/usr/bin/env python3
"""Summarise tools/run_seed_suite.sh results:  python3 tools/suite_report.py <out_dir>

A seed PASSES only if, by Gazebo ground truth: the mission state machine
reached COMPLETE, every leg it reported as arrived was really in its zone
(legs_gt_verified; '?' for summaries written before that check existed), no
footprint-wall contact, no footprint-boulder contact and no wheel ever inside a
crater bowl.
"""
import glob
import json
import os
import sys

out = sys.argv[1]
rows = []
for f in sorted(glob.glob(os.path.join(out, 'seed_*_summary.json')), key=lambda p: int(p.split('_')[-2])):
    s = json.load(open(f))
    legs = s.get('legs_gt_verified')
    ok = (s['final_state'] == 'COMPLETE' and legs is not False and not s['wall_contact']
          and not s['boulder_contact'] and s['crater_wheel_samples'] == 0)
    reason = s.get('mission_status', {}).get('fail_reason', '') if s['final_state'] != 'COMPLETE' else ''
    rows.append((s['seed'], 'PASS' if ok else 'FAIL', s['final_state'], s['duration_sec'], s['loc_err_mean'],
                 s['loc_err_max'], s['min_wall_dist'], s['min_boulder_dist'], s['crater_wheel_samples'],
                 s.get('crater_false_cells'), '?' if legs is None else ('yes' if legs else 'NO'), reason[:70]))
hdr = ('seed', 'result', 'final', 'time_s', 'loc_mean', 'loc_max', 'wall_min', 'boulder_min', 'crater_wheel',
       'false_cells', 'legs_gt', 'reason')
print(' | '.join(hdr))
for r in rows:
    print(' | '.join('-' if v is None else str(v) for v in r))
n = len(rows)
print(f'\n{sum(r[1] == "PASS" for r in rows)}/{n} seeds passed; '
      f'{sum(r[2] == "COMPLETE" for r in rows)}/{n} reached COMPLETE; '
      f'{sum(r[8] > 0 for r in rows)}/{n} had a wheel in a crater; '
      f'{sum(r[6] is not None and r[6] <= 0.02 for r in rows)}/{n} touched a wall.')
