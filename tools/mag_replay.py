#!/usr/bin/env python3
"""Replays the mag_heading logic over a bag and compares with the RTK heading.

  tools/mag_replay.py BAG [--mag-topic /bno085/mag --source cal] [--two-pass]
                          [--cal FILE | --cal-bag BAG2] [--save FILE]
                          [--fit-interval 45] [--set KEY=VALUE ...] [--dump FILE.npz]

Same code as the node (vitulus_imu.mag_heading), fed in bag order with header
stamps (NAV-RELPOSNED has none: bag time, as the node uses the receive time).
Single pass: starts uncalibrated and self-calibrates from RTK as the node does
on the robot.  --two-pass: the calibration found at the end of pass 1 is frozen
for pass 2 (shows what a robot with a stored calibration would give).
--cal-bag BAG2: cross test, the calibration is self-calibrated on BAG2 and
frozen for BAG (same magnetometer topic and source in both).
Old bags have only /bno085/mag (calibrated report): --mag-topic /bno085/mag
--source cal (default when the bag has no /bno085/mag_uncal).  A bag without
/gnss_heading/navrelposned cannot show the carrier fix: the RTK heading is then
taken by its variance band alone (printed; --no-fixed-gate forces that).
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


def run(bag_path, topics, est, fit_interval, verbose, rows=None):
    """topics: dict mag, imu, rtk, relpos, motion.  rows: list filled with
    (t, yaw, ok, moving (1/0/-1), frozen, rtk error deg or nan, tau_eff) per output."""
    states = {}
    reasons = {}
    fits = []
    t_fit = None
    n_mag = 0
    wanted = [v for v in topics.values() if v]
    def taken(t, out):
        if out is None:
            return
        states[out['state']] = states.get(out['state'], 0) + 1
        for r in out['reasons']:
            reasons[r] = reasons.get(r, 0) + 1
        if rows is not None and out.get('yaw') is not None:
            mv = out['moving']
            rows.append((out['t'], out['yaw'], out['ok'], -1 if mv is None else int(mv), out['frozen'],
                         out.get('rtk_err_deg', float('nan')), out['fuse_tau_eff_s']))

    for topic, m, bt in rosbag.Bag(bag_path).read_messages(topics=wanted):
        if topic == topics['relpos']:
            est.on_relpos(bt.to_sec(), m.flags)
            continue
        t = m.header.stamp.to_sec() or bt.to_sec()
        if topic == topics['imu']:
            o = m.orientation
            taken(t, est.on_imu(t, o.x, o.y, o.z, o.w, m.angular_velocity.z))
        elif topic == topics['rtk']:
            o = m.orientation
            est.on_rtk(t, o.x, o.y, o.z, o.w, m.orientation_covariance[8], t_rx=bt.to_sec())
        elif topic == topics['motion']:
            est.on_motion(t, m.twist.twist.linear.x)
        else:
            n_mag += 1
            f = m.magnetic_field
            taken(t, est.on_mag(t, f.x * 1e6, f.y * 1e6, f.z * 1e6))
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
    return states, fits, n_mag, reasons


def report(est, states, title, reasons=None):
    total = sum(states.values())
    print('\n== %s' % title)
    print('  output states: %s' % ', '.join('%s %.1f %%' % (k, 100.0 * v / total)
                                            for k, v in sorted(states.items())) if total else '  no output')
    if reasons:
        print('  reasons (share of outputs): %s' % ', '.join(
            '%s %.1f %%' % (k, 100.0 * v / total) for k, v in sorted(reasons.items())))
    if est.cal is not None:
        c = est.cal.to_dict()
        print('  calibration: centre %s uT, semi-axes %s uT, offset %.1f deg, Fref %.1f uT, Iref %.1f deg, '
              'residual p50/p90 %s/%s deg, %d bins' % (c['center_uT'], c['radii_uT'], c['offset_deg'],
                                                       c['Fref_uT'], c['Iref_deg'], c['resid_p50_deg'],
                                                       c['resid_p90_deg'], c['bins']))
        print('  axis weights (information, body axes): %s' % (
            'none (both axes the same)' if not est.weighted else np.round(est.Q, 3).tolist()))
    print('  filter restarts %d%s; RTK reference at the end: %s' % (
        est.resets, ' (last: %s)' % est.reset_reason if est.resets else '', est.rtk_reason))
    if not est.errors:
        print('  no comparison with RTK')
        return None
    e = np.array([x[1] for x in est.errors])
    ok = np.array([x[2] for x in est.errors], bool)
    mov = np.array([{True: 1, False: 0, None: -1}[x[3]] for x in est.errors])
    res = {'all': summary(e), 'ok': summary(e[ok]), 'ok_share': round(float(ok.mean()), 3),
           'moving': summary(e[mov == 1]), 'standing': summary(e[mov == 0]),
           'ok_moving': summary(e[ok & (mov == 1)]), 'ok_standing': summary(e[ok & (mov == 0)])}
    for k in ('all', 'ok', 'moving', 'standing', 'ok_moving', 'ok_standing'):
        s = res[k]
        if s:
            print('  |err| vs RTK %-11s n=%5d  median %4.1f  p90 %4.1f  p99 %5.1f  max %5.1f deg' % (
                k, s['n'], s['median'], s['p90'], s['p99'], s['max']))
    print('  ok share (while RTK valid): %.1f %%' % (100 * res['ok_share']))
    return res


def bag_topics(path):
    return rosbag.Bag(path).get_type_and_topic_info().topics


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--mag-topic')
    ap.add_argument('--source', help='uncal or cal (default by topic)')
    ap.add_argument('--imu-topic', default='/bno085/imu')
    ap.add_argument('--rtk-topic', default='/gnss_heading/navheading')
    ap.add_argument('--relpos-topic', default='/gnss_heading/navrelposned')
    ap.add_argument('--motion-topic', default='/mobile_base_controller/odom',
                    help="wheel odometry; '' replays without it (no standing freeze)")
    ap.add_argument('--cal', help='start from this calibration file')
    ap.add_argument('--cal-bag', help='cross test: self-calibrate on this bag, freeze, replay BAG')
    ap.add_argument('--save', help='write the final calibration here')
    ap.add_argument('--no-selfcal', action='store_true')
    ap.add_argument('--two-pass', action='store_true')
    ap.add_argument('--fit-interval', type=float, default=45.0)
    ap.add_argument('--hard-xy', action='store_true',
                    help='subtract the horizontal hard iron before levelling too (node default: off)')
    ap.add_argument('--no-fixed-gate', action='store_true',
                    help='take the RTK heading by its variance band alone (no NAV-RELPOSNED)')
    ap.add_argument('--set', action='append', default=[], metavar='KEY=VALUE',
                    help='estimator parameter (MagHeadingEstimator.DEFAULTS), repeatable')
    ap.add_argument('--dump', help='write the outputs of the last pass to this .npz '
                                   '(t, yaw, ok, moving, frozen, rtk_err_deg, tau_eff)')
    ap.add_argument('--json', action='store_true', help='print the results as JSON at the end')
    ap.add_argument('-q', '--quiet', action='store_true')
    a = ap.parse_args()

    have = bag_topics(a.bag)
    if a.mag_topic is None:
        a.mag_topic = '/bno085/mag_uncal' if '/bno085/mag_uncal' in have else '/bno085/mag'
    if a.source is None:
        a.source = 'uncal' if a.mag_topic.endswith('uncal') else 'cal'
    print('bag %s, mag %s (source %s)' % (a.bag, a.mag_topic, a.source))

    params = dict(source=a.source, self_calibrate=not a.no_selfcal, stats_window_s=1e9,
                  hard_xy_pre_level=a.hard_xy)
    for kv in a.set:
        k, _, v = kv.partition('=')
        d = MagHeadingEstimator.DEFAULTS[k]
        params[k] = (v.lower() in ('1', 'true', 'yes')) if isinstance(d, bool) else type(d)(v)

    def setup(path):
        """(topics, parameters) for one bag: the gates follow what the bag has."""
        there = have if path == a.bag else bag_topics(path)
        tp = {'mag': a.mag_topic, 'imu': a.imu_topic, 'rtk': a.rtk_topic,
              'relpos': a.relpos_topic, 'motion': a.motion_topic}
        pr = dict(params)
        if a.no_fixed_gate or tp['relpos'] not in there:
            if not a.no_fixed_gate:
                print('NOTE %s has no %s: RTK heading taken by its variance band alone' % (
                    os.path.basename(path), tp['relpos']))
            pr['rtk_require_fixed'] = False
            tp['relpos'] = None
        if tp['motion'] and tp['motion'] not in there:
            print('NOTE %s has no %s: no standing freeze' % (os.path.basename(path), tp['motion']))
            tp['motion'] = None
        return tp, pr

    cal = None
    if a.cal:
        cal, note = load_calibration(a.cal, a.source)
        print(note)
    results = {}
    rows = []
    if a.cal_bag:
        tp, pr = setup(a.cal_bag)
        est0 = MagHeadingEstimator(cal, **dict(pr, self_calibrate=True))
        states, fits, n_mag, reasons = run(a.cal_bag, tp, est0, a.fit_interval, not a.quiet)
        print('calibration bag %s: fits %d, accepted %d' % (
            a.cal_bag, len(fits), sum(1 for _, r in fits if r['accepted'])))
        if est0.cal is None:
            print('no calibration from the calibration bag')
            return 1
        tp, pr = setup(a.bag)
        est = MagHeadingEstimator(est0.cal, **dict(pr, self_calibrate=False))
        states, _, n_mag, reasons = run(a.bag, tp, est, a.fit_interval, False, rows)
        results['cross'] = report(est, states, 'cross: calibration of %s frozen' % os.path.basename(a.cal_bag),
                                  reasons)
    else:
        tp, pr = setup(a.bag)
        est = MagHeadingEstimator(cal, **pr)
        states, fits, n_mag, reasons = run(a.bag, tp, est, a.fit_interval, not a.quiet, rows)
        acc = sum(1 for _, r in fits if r['accepted'])
        print('magnetometer samples %d, fits %d, accepted %d, RTK used %d, RTK glitches %d' % (
            n_mag, len(fits), acc, est.rtk_used, est.rtk_glitches))
        results['pass1'] = report(est, states, 'single pass (self-calibration as on the robot)', reasons)
        if a.two_pass:
            if est.cal is None:
                print('no calibration after pass 1, no pass 2')
            else:
                rows = []
                est2 = MagHeadingEstimator(est.cal, **dict(pr, self_calibrate=False))
                states2, _, _, reasons2 = run(a.bag, tp, est2, a.fit_interval, False, rows)
                results['pass2'] = report(est2, states2, 'pass 2 (calibration of pass 1 frozen)', reasons2)
    if a.save and est.cal is not None:
        save_calibration(a.save, est.cal)
        print('calibration saved to', a.save)
    if a.dump:
        r = np.array(rows, float).reshape(-1, 7)
        np.savez_compressed(a.dump, t=r[:, 0], yaw=r[:, 1], ok=r[:, 2], moving=r[:, 3], frozen=r[:, 4],
                            rtk_err_deg=r[:, 5], tau_eff=r[:, 6])
        print('outputs written to', a.dump)
    if a.json:
        print(json.dumps(results))
    return 0


if __name__ == '__main__':
    sys.exit(main())
