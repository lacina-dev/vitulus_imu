#!/usr/bin/env python3
"""Replays the mag_heading logic over a bag and compares with the RTK heading.

  tools/mag_replay.py BAG [--mag-topic /bno085/mag --source cal] [--two-pass]
                          [--cal FILE] [--save FILE] [--fit-interval 45]

Same code as the node (vitulus_imu.mag_heading), fed in bag order with header
stamps.  Single pass: starts uncalibrated and self-calibrates from RTK as the
node does on the robot.  --two-pass: the calibration found at the end of pass 1
is frozen for pass 2 (shows what a robot with a stored calibration would give).
Old bags have only /bno085/mag (calibrated report): --mag-topic /bno085/mag
--source cal (default when the bag has no /bno085/mag_uncal).
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'src'))

import numpy as np  # noqa: E402
import rosbag  # noqa: E402

from vitulus_imu.mag_heading import (DEG, MagHeadingEstimator, load_calibration,  # noqa: E402
                                     save_calibration, summary)


def run(bag_path, mag_topic, imu_topic, rtk_topic, est, fit_interval, verbose):
    states = {}
    fits = []
    t_fit = None
    n_mag = 0
    for topic, m, bt in rosbag.Bag(bag_path).read_messages(topics=[mag_topic, imu_topic, rtk_topic]):
        t = m.header.stamp.to_sec() or bt.to_sec()
        if topic == imu_topic:
            o = m.orientation
            est.on_imu(t, o.x, o.y, o.z, o.w)
        elif topic == rtk_topic:
            o = m.orientation
            est.on_rtk(t, o.x, o.y, o.z, o.w, m.orientation_covariance[8])
        else:
            n_mag += 1
            f = m.magnetic_field
            out = est.on_mag(t, f.x * 1e6, f.y * 1e6, f.z * 1e6)
            if out is not None:
                states[out['state']] = states.get(out['state'], 0) + 1
            if t_fit is None:
                t_fit = t
            if est.p['self_calibrate'] and t - t_fit >= fit_interval:
                t_fit = t
                res = est.fit(t)
                fits.append((t, res))
                if verbose:
                    print('  fit t=%7.1f  %-40s cover %2d  n %4d  rej %3d  p50 %s  p90 %s  current p90 %s' % (
                        t, res['reason'], res['coverage'], res['n'], res['rejected'], res['p50_deg'],
                        res['p90_deg'], res['current_p90_deg']))
    return states, fits, n_mag


def report(est, states, title):
    total = sum(states.values())
    print('\n== %s' % title)
    print('  output states: %s' % ', '.join('%s %.1f %%' % (k, 100.0 * v / total)
                                            for k, v in sorted(states.items())) if total else '  no output')
    if est.cal is not None:
        c = est.cal.to_dict()
        print('  calibration: centre %s uT, semi-axes %s uT, offset %.1f deg, Fref %.1f uT, Iref %.1f deg, '
              'residual p50/p90 %s/%s deg, %d bins' % (c['center_uT'], c['radii_uT'], c['offset_deg'],
                                                       c['Fref_uT'], c['Iref_deg'], c['resid_p50_deg'],
                                                       c['resid_p90_deg'], c['bins']))
    if not est.errors:
        print('  no comparison with RTK')
        return None
    e = np.array([x[1] for x in est.errors])
    ok = np.array([x[2] for x in est.errors])
    res = {'all': summary(e), 'ok': summary(e[ok]), 'ok_share': round(float(ok.mean()), 3)}
    for k in ('all', 'ok'):
        s = res[k]
        if s:
            print('  |err| vs RTK %-4s n=%5d  median %4.1f  p90 %4.1f  p99 %5.1f  max %5.1f deg' % (
                k, s['n'], s['median'], s['p90'], s['p99'], s['max']))
    print('  ok share (while RTK valid): %.1f %%' % (100 * res['ok_share']))
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--mag-topic')
    ap.add_argument('--source', help='uncal or cal (default by topic)')
    ap.add_argument('--imu-topic', default='/bno085/imu')
    ap.add_argument('--rtk-topic', default='/gnss_heading/navheading')
    ap.add_argument('--cal', help='start from this calibration file')
    ap.add_argument('--save', help='write the final calibration here')
    ap.add_argument('--no-selfcal', action='store_true')
    ap.add_argument('--two-pass', action='store_true')
    ap.add_argument('--fit-interval', type=float, default=45.0)
    ap.add_argument('--hard-xy', action='store_true',
                    help='subtract the horizontal hard iron before levelling too (node default: off)')
    ap.add_argument('--json', action='store_true', help='print the results as JSON at the end')
    ap.add_argument('-q', '--quiet', action='store_true')
    a = ap.parse_args()

    if a.mag_topic is None:
        topics = rosbag.Bag(a.bag).get_type_and_topic_info().topics
        a.mag_topic = '/bno085/mag_uncal' if '/bno085/mag_uncal' in topics else '/bno085/mag'
    if a.source is None:
        a.source = 'uncal' if a.mag_topic.endswith('uncal') else 'cal'
    print('bag %s, mag %s (source %s)' % (a.bag, a.mag_topic, a.source))

    params = dict(source=a.source, self_calibrate=not a.no_selfcal, stats_window_s=1e9,
                  hard_xy_pre_level=a.hard_xy)
    cal = None
    if a.cal:
        cal, note = load_calibration(a.cal, a.source)
        print(note)
    est = MagHeadingEstimator(cal, **params)
    states, fits, n_mag = run(a.bag, a.mag_topic, a.imu_topic, a.rtk_topic, est, a.fit_interval, not a.quiet)
    acc = sum(1 for _, r in fits if r['accepted'])
    print('magnetometer samples %d, fits %d, accepted %d, RTK used %d, RTK glitches %d' % (
        n_mag, len(fits), acc, est.rtk_used, est.rtk_glitches))
    results = {'pass1': report(est, states, 'single pass (self-calibration as on the robot)')}
    if a.two_pass:
        if est.cal is None:
            print('no calibration after pass 1, no pass 2')
        else:
            est2 = MagHeadingEstimator(est.cal, **dict(params, self_calibrate=False))
            states2, _, _ = run(a.bag, a.mag_topic, a.imu_topic, a.rtk_topic, est2, a.fit_interval, False)
            results['pass2'] = report(est2, states2, 'pass 2 (calibration of pass 1 frozen)')
    if a.save and est.cal is not None:
        save_calibration(a.save, est.cal)
        print('calibration saved to', a.save)
    if a.json:
        print(json.dumps(results))
    return 0


if __name__ == '__main__':
    sys.exit(main())
