"""vitulus_imu.mag_heading on synthetic data (no ROS).

  nice -n 10 python3 -m pytest test/test_mag_heading.py -q
"""
import json
import math
import os
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'src'))

from vitulus_imu import mag_heading as mh  # noqa: E402

D = math.radians
H_EARTH, V_EARTH, AZ = 20.0, 45.0, 1.0           # uT, uT, rad (offset expected = AZ)
HARD = np.array([28.0, 32.0, 0.0])
SOFT = np.array([[1.15, 0.10, 0.0], [0.10, 0.87, 0.0], [0.0, 0.0, 1.0]])
FIXED = 0x100 | 0x10 | 0x07                      # NAV-RELPOSNED: heading valid, carrSoln FIXED
FLOAT = 0x100 | 0x08 | 0x07


def simulate(est, t0, duration, yaw_fn, tilt_deg=(3.0, 4.0), rtk=True, gyro_drift=0.0, extra=None,
             noise=0.3, rtk_noise_deg=0.5, seed=1, game_offset=0.4, tell_tilt=True, hard=HARD,
             relpos=FIXED, speed=None):
    """Feeds est like the robot: IMU 100 Hz, mag 20 Hz, RTK 3 Hz (each heading
    preceded by NAV-RELPOSNED with the flags relpos, None: not sent), wheel
    odometry 50 Hz when speed (m/s, number or function of t) is given.  Returns
    a list of (t, true yaw, output dict) for every output."""
    rng = np.random.default_rng(seed)
    t = t0 + np.arange(0.0, duration, 0.01)
    yaw = np.array([yaw_fn(x) for x in t])
    roll = D(tilt_deg[0]) * np.sin(0.7 * t)
    pitch = D(tilt_deg[1]) * np.sin(0.45 * t + 1.0)
    rot = Rotation.from_euler('ZYX', np.c_[yaw, pitch, roll])
    tilt = np.c_[pitch, roll] if tell_tilt else np.zeros((len(t), 2))
    game = Rotation.from_euler('ZYX', np.c_[yaw + game_offset + gyro_drift * (t - t0), tilt]).as_quat()
    b_w = np.array([H_EARTH * math.cos(AZ), H_EARTH * math.sin(AZ), -V_EARTH])
    b_s = rot.inv().apply(b_w)                        # field in the sensor frame
    meas = b_s @ SOFT.T + np.asarray(hard) + rng.normal(0, noise, b_s.shape)
    outs = []
    for i, ti in enumerate(t):
        q = game[i]
        est.on_imu(ti, q[0], q[1], q[2], q[3])
        if i % 5 == 0:
            b = meas[i] + (extra(ti) if extra is not None else 0.0)
            out = est.on_mag(ti, float(b[0]), float(b[1]), float(b[2]))
            if out is not None:
                outs.append((ti, yaw[i], out))
        if speed is not None and i % 2 == 0:
            est.on_motion(ti, speed(ti) if callable(speed) else speed)
        if rtk and i % 33 == 0:
            if relpos is not None:
                est.on_relpos(ti, relpos)
            r = Rotation.from_euler('Z', yaw[i] + D(rng.normal(0, rtk_noise_deg))).as_quat()
            est.on_rtk(ti, r[0], r[1], r[2], r[3], 0.0005)
    return outs


def spin(rate=0.3):
    return lambda t: rate * t


def errors(outs, only_ok=False):
    e = [abs(mh.wrap(o['yaw'] - y)) * mh.DEG for _, y, o in outs
         if o.get('yaw') is not None and (o['ok'] or not only_ok)]
    return np.array(e)


@pytest.fixture(scope='module')
def calibrated():
    """An estimator self-calibrated from RTK on a spinning, tilting robot."""
    est = mh.MagHeadingEstimator(None, source='uncal')
    simulate(est, 1000.0, 80.0, spin())
    res = est.fit(1080.0)
    return est, res


# -- geometry --------------------------------------------------------------------

def test_quat_level_matches_scipy():
    rng = np.random.default_rng(3)
    for _ in range(50):
        e = rng.uniform([-math.pi, -0.4, -0.4], [math.pi, 0.4, 0.4])
        q = Rotation.from_euler('ZYX', e).as_quat()
        yaw, L = mh.quat_level(*q)
        ref = Rotation.from_euler('ZYX', [0.0, e[1], e[2]]).as_matrix()
        assert abs(mh.wrap(yaw - e[0])) < 1e-9
        assert np.allclose(np.array(L), ref, atol=1e-9)
        assert abs(mh.quat_yaw(*q) - Rotation.from_quat(q).as_euler('ZYX')[0]) < 1e-9
    assert mh.quat_level(0, 0, 0, 0) is None


def test_fit_ellipse_recovers_centre_and_axes():
    a = np.linspace(0, 2 * math.pi, 36, endpoint=False)
    c = np.array([28.0, 32.0])
    pts = np.c_[np.cos(a), np.sin(a)] @ (SOFT[:2, :2] * 20.0).T + c
    fit = mh.fit_ellipse(pts)
    assert fit is not None
    centre, W, radii = fit
    assert np.allclose(centre, c, atol=1e-6)
    hc = (pts - centre) @ W.T
    assert np.allclose(np.hypot(hc[:, 0], hc[:, 1]), 1.0, atol=1e-6)
    assert mh.fit_ellipse(pts[:4]) is None
    line = np.c_[np.linspace(0, 1, 20), np.linspace(0, 1, 20)]
    assert mh.fit_ellipse(line) is None


# -- calibration and heading -----------------------------------------------------

def test_self_calibration_and_heading_within_one_degree(calibrated):
    est, res = calibrated
    assert res['accepted'], res
    cal = est.cal
    assert cal.source == 'uncal'
    assert np.allclose(cal.hard[:2] + cal.center, HARD[:2], atol=1.0)
    assert abs(cal.hard[2]) < 5.0
    assert abs(math.degrees(mh.wrap(cal.offset - AZ))) < 1.0
    assert res['p90_deg'] < 2.0
    outs = simulate(est, 1080.0, 40.0, spin(), seed=2)
    after = [o for o in outs if o[0] > 1092.0]        # past the 10 s warmup
    e = errors(after)
    # +-3/4 deg tilt: the 2D model leaves ~1 deg (soft iron couples the vertical field).
    assert np.median(e) < 1.2 and np.percentile(e, 99) < 3.0, (np.median(e), np.percentile(e, 99))
    assert all(o['ok'] for _, _, o in after), set(tuple(o['reasons']) for _, _, o in after)
    assert after[-1][2]['sigma_deg'] == 8.0


def test_level_ground_heading_within_half_degree():
    est = mh.MagHeadingEstimator(None, source='uncal')
    simulate(est, 0.0, 80.0, spin(), tilt_deg=(0.0, 0.0))
    assert est.fit(80.0)['accepted']
    outs = simulate(est, 80.0, 40.0, spin(), tilt_deg=(0.0, 0.0), seed=2)
    e = errors([o for o in outs if o[0] > 92.0])
    assert np.median(e) < 0.3 and np.percentile(e, 99) < 1.0, (np.median(e), np.percentile(e, 99))


def test_levelling_matters():
    # 10 deg roll: ~23 deg p99 without levelling, ~6 deg with it (2D soft-iron model limit).
    est = mh.MagHeadingEstimator(None, source='uncal')
    simulate(est, 0.0, 80.0, spin(), tilt_deg=(1.0, 1.0))
    assert est.fit(80.0)['accepted']
    levelled, raw = [], []
    for tell, out in ((True, levelled), (False, raw)):
        e2 = mh.MagHeadingEstimator(est.cal, source='uncal')
        outs = simulate(e2, 80.0, 30.0, lambda t: 0.5, tilt_deg=(10.0, 0.0), rtk=False, seed=3,
                        tell_tilt=tell)
        out.extend(errors([o for o in outs if o[0] > 92.0]))
    assert np.percentile(levelled, 99) < 0.35 * np.percentile(raw, 99), (
        np.percentile(levelled, 99), np.percentile(raw, 99))
    assert np.median(levelled) < 5.0


def test_heading_without_rtk_while_driving_straight(calibrated):
    est, _ = calibrated
    est2 = mh.MagHeadingEstimator(est.cal, source='uncal')
    outs = simulate(est2, 2000.0, 30.0, lambda t: 2.5 + 0.05 * math.sin(0.2 * t), rtk=False, seed=4)
    after = [o for o in outs if o[0] > 2012.0]
    e = errors(after)
    assert np.median(e) < 1.0 and e.max() < 2.5
    assert all(o['ok'] for _, _, o in after)
    assert est2.status(2030.0)['rtk_compare'].get('all') is None


def test_uncalibrated_without_calibration():
    est = mh.MagHeadingEstimator(None, source='uncal', self_calibrate=False)
    outs = simulate(est, 0.0, 5.0, spin(), rtk=False)
    assert outs and all(o['state'] == 'uncalibrated' and o['yaw'] is None for _, _, o in outs)
    s = est.status(5.0)
    assert s['state'] == 'uncalibrated' and s['yaw_deg'] is None


def test_stale_without_data(calibrated):
    est, _ = calibrated
    s = est.status(est.t_mag + 5.0)
    assert s['state'] == 'stale'


# -- gate ------------------------------------------------------------------------

def gated(est, extra=None, gyro_drift=0.0, start=3000.0):
    e = mh.MagHeadingEstimator(est.cal, source='uncal')
    outs = simulate(e, start, 30.0, lambda t: 0.7 + 0.1 * math.sin(0.3 * t), rtk=False, seed=5,
                    extra=extra, gyro_drift=gyro_drift)
    return e, outs


def test_gate_ok_when_clean(calibrated):
    _, outs = gated(calibrated[0])
    assert all(o['ok'] for t, _, o in outs if t > 3012.0)


def test_gate_field_strength_disturbed(calibrated):
    # A steady 10 uT along z from t=3015 (a steel lid, the dock): F changes ~20 %.
    _, outs = gated(calibrated[0], extra=lambda t: np.array([0.0, 0.0, 10.0]) if t > 3015.0 else 0.0)
    late = [o for t, _, o in outs if t > 3017.0]
    assert late and all(o['state'] == 'disturbed' and 'field' in o['reasons'] for o in late)
    s = mh.MagHeadingEstimator.DEFAULTS['max_dF_pct']
    assert all(o['dF_pct'] >= s for o in late)


def test_gate_gyro_disagreement_disturbed(calibrated):
    # Gyro (game yaw) runs away 1.5 deg/s from the magnetometer.
    _, outs = gated(calibrated[0], gyro_drift=D(1.5))
    late = [o for t, _, o in outs if t > 3015.0]
    assert late and all(not o['ok'] for o in late)
    assert all('gyro_drift' in o['reasons'] for o in late)


def test_gate_noisy_heading_disturbed(calibrated):
    # A horizontal disturbance that swings: heading noise against the gyro.
    # 20 uT (std ~20 deg): the 8 uT of the first version gives 4.6..6.9 deg,
    # which is the per-sample noise of normal driving (3..7.5 deg on the
    # 2026-11-08 bags) and averages out in the slow fusion.
    _, outs = gated(calibrated[0], extra=lambda t: np.array([20.0 * math.sin(3.0 * t), 0.0, 0.0])
                    if t > 3012.0 else 0.0)
    late = [o for t, _, o in outs if t > 3018.0]
    assert late and all(not o['ok'] and 'gyro_std' in o['reasons'] for o in late)


def test_warmup_is_not_ok(calibrated):
    _, outs = gated(calibrated[0])
    first = [o for t, _, o in outs if t < 3004.0]
    assert first and all(not o['ok'] and 'warmup' in o['reasons'] for o in first)


# -- self-calibration: coverage, refusal, storage --------------------------------

def test_no_fit_without_coverage():
    est = mh.MagHeadingEstimator(None, source='uncal')
    simulate(est, 0.0, 40.0, lambda t: 0.3 * math.sin(0.2 * t))   # +-17 deg only
    res = est.fit(40.0)
    assert not res['accepted'] and res['reason'].startswith('coverage')
    assert est.cal is None
    s = est.status(40.0)
    assert s['self_cal']['bins_covered'] < s['self_cal']['bins_needed']


def test_no_samples_before_rtk_settles():
    est = mh.MagHeadingEstimator(None, source='uncal', rtk_settle_s=5.0)
    simulate(est, 0.0, 4.0, spin())
    assert est.bins.total() == 0
    simulate(est, 4.0, 4.0, spin())
    assert est.bins.total() > 0


def test_fit_refused_with_bad_reference():
    # RTK heading off by a direction-dependent 15 deg: residual too large.
    est = mh.MagHeadingEstimator(None, source='uncal', rtk_gyro_tol_deg=90.0)
    rng = np.random.default_rng(7)
    simulate(est, 0.0, 80.0, spin())
    snap = est.fit_snapshot()
    snap['ref'] = mh.wrap(snap['ref'] + D(15.0) * rng.choice([-1.0, 1.0], len(snap['ref'])))
    res = est.fit_from(snap, 80.0)
    est.adopt(res, 80.0)
    assert not res['accepted'] and 'residual' in res['reason'], res['reason']
    assert est.cal is None


def test_fit_refused_when_worse_than_current(calibrated):
    est, _ = calibrated
    snap = est.fit_snapshot()
    res = mh.fit_calibration(snap, 'uncal', current=est.cal, max_worse_deg=-0.5)
    assert not res['accepted'] and res['reason'].startswith('worse than current')
    res = mh.fit_calibration(snap, 'uncal', current=est.cal)
    assert res['accepted'] and res['current_p90_deg'] is not None


def test_rtk_glitch_ignored(calibrated):
    est, _ = calibrated
    t = est.t_imu
    q = Rotation.from_euler('Z', est.rtk_yaw(t) + D(90.0)).as_quat()
    used = est.rtk_used
    assert est.on_rtk(t, q[0], q[1], q[2], q[3], 0.0005) is False
    assert est.rtk_used == used
    q = Rotation.from_euler('Z', est.rtk_yaw(t)).as_quat()
    assert est.on_rtk(t, q[0], q[1], q[2], q[3], 5.0) is False     # covariance too large


def test_save_load_and_source_mismatch(tmp_path, calibrated):
    est, _ = calibrated
    path = str(tmp_path / 'sub' / 'mag_calibration.yaml')
    mh.save_calibration(path, est.cal)
    assert [p for p in os.listdir(str(tmp_path / 'sub'))] == ['mag_calibration.yaml']
    cal, note = mh.load_calibration(path, 'uncal')
    assert cal is not None, note
    assert np.allclose(cal.center, est.cal.center, atol=1e-3)
    assert np.allclose(cal.W, est.cal.W, atol=1e-5)
    assert abs(math.degrees(cal.offset - est.cal.offset)) < 1e-3
    assert abs(cal.Fref - est.cal.Fref) < 1e-3
    bl = np.array([[30.0, 20.0, -40.0], [10.0, 45.0, -44.0]])
    assert np.allclose(cal.apply(bl)[0], est.cal.apply(bl)[0], atol=1e-4)
    cal2, note = mh.load_calibration(path, 'cal')
    assert cal2 is None and 'source' in note
    e = mh.MagHeadingEstimator(None, source='cal')
    assert e.set_calibration(cal) is False and e.cal is None
    with open(path, 'w') as fh:
        fh.write('::: not yaml [')
    assert mh.load_calibration(path, 'uncal')[0] is None
    assert mh.load_calibration(str(tmp_path / 'none.yaml'))[0] is None


def test_status_json(calibrated):
    est, _ = calibrated
    outs = simulate(est, 1200.0, 15.0, spin(), seed=9)
    s = est.status(outs[-1][0])
    json.dumps(s)
    for key in ('state', 'reasons', 'yaw_deg', 'sigma_deg', 'dF_pct', 'dIncl_deg', 'gyro_std_deg',
                'gyro_drift_deg', 'calibration', 'self_cal', 'rtk_compare'):
        assert key in s, key
    for key in ('age_s', 'center_uT', 'radii_uT', 'Fref_uT', 'Iref_deg', 'resid_p50_deg', 'resid_p90_deg'):
        assert key in s['calibration'], key
    cmp = s['rtk_compare']
    assert cmp['rtk_valid'] and 'err_deg' in cmp
    for key in ('median', 'p90', 'p99', 'max'):
        assert key in cmp['all'] and key in cmp['ok']


def test_unknown_parameter_refused():
    with pytest.raises(ValueError):
        mh.MagHeadingEstimator(None, sorce='uncal')


# -- vertical hard iron (sensor z) on a slope ------------------------------------

@pytest.mark.parametrize('hz', [30.0, -40.0])
def test_vertical_hard_iron_on_tilt(hz):
    """Hard iron along sensor z leaks into the levelled horizontal as hz sin(tilt);
    it is estimated in 3D before levelling, so the error stays as with hz = 0."""
    hard = np.array([28.0, 32.0, hz])
    res = {}
    for name, h in (('hz', hard), ('zero', HARD)):
        est = mh.MagHeadingEstimator(None, source='uncal')
        simulate(est, 0.0, 80.0, spin(), tilt_deg=(4.0, 3.0), hard=h)
        fit = est.fit(80.0)
        assert fit['accepted'], fit
        if name == 'hz':
            assert abs(est.cal.hard[2] - hz) < 5.0, est.cal.hard
        # A slope of 8 deg, driving straight in several directions, no RTK.
        e_all = []
        for k, yaw0 in enumerate((0.3, 1.9, 3.5, 5.1)):
            e2 = mh.MagHeadingEstimator(est.cal, source='uncal')
            outs = simulate(e2, 100.0 * (k + 1), 25.0, lambda t, y=yaw0: y, tilt_deg=(8.0, 3.0),
                            rtk=False, seed=11 + k, hard=h)
            e_all.extend(errors([o for o in outs if o[0] > 100.0 * (k + 1) + 12.0]))
        res[name] = np.array(e_all)
    e, e0 = res['hz'], res['zero']
    assert np.percentile(e, 90) < np.percentile(e0, 90) + 1.0, (np.percentile(e, 90), np.percentile(e0, 90))
    assert e.max() < e0.max() + 1.5, (e.max(), e0.max())


def test_vertical_hard_iron_flat_ground_falls_back_to_prior():
    # No tilt in the samples: hz cannot be seen and stays at the prior (0).
    est = mh.MagHeadingEstimator(None, source='uncal')
    simulate(est, 0.0, 80.0, spin(), tilt_deg=(0.0, 0.0), hard=np.array([28.0, 32.0, 30.0]))
    assert est.fit(80.0)['accepted']
    assert abs(est.cal.hard[2]) < 1.0


def test_old_calibration_file_without_hard(tmp_path, calibrated):
    est, _ = calibrated
    d = est.cal.to_dict()
    del d['hard_uT'], d['best_p90_deg']
    import yaml
    path = str(tmp_path / 'old.yaml')
    with open(path, 'w') as fh:
        yaml.safe_dump(d, fh)
    cal, note = mh.load_calibration(path, 'uncal')
    assert cal is not None, note
    assert np.allclose(cal.hard, 0.0) and cal.best_p90_deg == cal.resid_p90_deg


@pytest.mark.parametrize('field,value', [('offset_deg', float('nan')), ('Fref_uT', float('inf')),
                                         ('center_uT', [float('nan'), 1.0]), ('hard_uT', [0, 0, float('inf')]),
                                         ('W', [[1.0, 0.0], [0.0, -1.0]])])
def test_non_finite_calibration_refused(tmp_path, calibrated, field, value):
    est, _ = calibrated
    d = est.cal.to_dict()
    d[field] = value
    import yaml
    path = str(tmp_path / 'bad.yaml')
    with open(path, 'w') as fh:
        yaml.safe_dump(d, fh)
    cal, note = mh.load_calibration(path, 'uncal')
    assert cal is None and 'unreadable' in note


def test_non_finite_input_not_published(calibrated):
    import copy
    est, _ = calibrated
    e = mh.MagHeadingEstimator(copy.deepcopy(est.cal), source='uncal')
    outs = simulate(e, 5000.0, 15.0, spin(), rtk=False)
    assert outs[-1][2]['ok']
    assert e.on_mag(5015.5, float('nan'), 1.0, 1.0) is None
    e.on_imu(5015.6, float('nan'), 0.0, 0.0, 1.0)            # ignored, level kept
    assert e.level is not None
    # A calibration that turns non-finite at run time (corrupted in memory).
    e.cal.offset = float('nan')
    out = e._output(5015.7)
    assert out['yaw'] is None and not out['ok'] and out['reasons'] == ['invalid']


def test_ratchet_no_creeping_worse(calibrated):
    """Each fit may be up to max_worse worse than the current one; the best so far
    stops a slow creep."""
    est, _ = calibrated
    snap = est.fit_snapshot()
    cal = est.cal
    cal.best_p90_deg = cal.resid_p90_deg - 2.0          # an earlier, clearly better calibration
    try:
        res = mh.fit_calibration(snap, 'uncal', current=cal)
        assert not res['accepted'] and res['reason'].startswith('worse than the best'), res
    finally:
        cal.best_p90_deg = cal.resid_p90_deg
    res = mh.fit_calibration(snap, 'uncal', current=cal)
    assert res['accepted'] and res['cal'].best_p90_deg <= cal.best_p90_deg


def test_samples_expire():
    est = mh.MagHeadingEstimator(None, source='uncal', sample_max_age_s=30.0)
    simulate(est, 0.0, 80.0, spin())
    assert len(est.fit_snapshot()['ref']) < est.bins.total()
    assert est.fit_snapshot()['t'].min() >= est.t_mag - 30.0


def test_hard_xy_pre_level_option():
    est = mh.MagHeadingEstimator(None, source='uncal', hard_xy_pre_level=True)
    simulate(est, 0.0, 80.0, spin(), tilt_deg=(4.0, 3.0), hard=np.array([28.0, 32.0, 30.0]))
    assert est.fit(80.0)['accepted']
    assert np.allclose(est.cal.hard, [28.0, 32.0, 30.0], atol=3.0)
    assert np.allclose(est.cal.center, 0.0, atol=1.0)


# -- slow fusion with the gyro ----------------------------------------------------

def fused(est, t0, duration, yaw_fn=lambda t: 0.7, seed=21, **kw):
    e = mh.MagHeadingEstimator(est.cal, source='uncal', self_calibrate=False)
    return e, simulate(e, t0, duration, yaw_fn, rtk=False, seed=seed, **kw)


def test_fusion_window_grows_then_settles(calibrated):
    e, outs = fused(calibrated[0], 6000.0, 100.0)
    w = {round(t - 6000.0): o['fuse_tau_eff_s'] for t, _, o in outs}
    # Effective window ~ min(elapsed, tau): not deaf for a minute after the start.
    assert 1.4 < w[2] < 2.6 and 8.0 < w[10] < 10.6 and 22.0 < w[30] < 30.6
    assert 45.0 < w[99] <= 60.0
    assert all('warmup' in o['reasons'] for t, _, o in outs if t < 6004.0)
    late = [x for x in outs if x[0] > 6006.0]
    assert all(o['ok'] for _, _, o in late)
    assert errors([x for x in outs if 6001.0 < x[0] < 6006.0]).max() < 3.0     # usable while growing
    assert errors(late).max() < 1.5
    sig = {round(t - 6000.0): o['sigma_deg'] for t, _, o in late}
    assert sig[20] == 8.0 and sig[99] == 4.0                 # sigma_start_deg, then sigma_ok_deg
    s = e.status(outs[-1][0])
    assert s['fusion']['tau_s'] == 60.0 and 45.0 < s['fusion']['tau_eff_s'] <= 60.0
    assert s['fusion']['frozen'] is False and s['fusion']['motion'] == 'unknown'
    assert 'note' in s['fusion']                             # no odometry: says it runs without the freeze
    assert s['calibration']['version'] == mh.CAL_VERSION


def test_fusion_ignores_short_disturbance(calibrated):
    """5 s of a 6 uT horizontal field (drive current): the 1 s window of the first
    version followed it (~15 deg), the fused heading moves by a fraction."""
    bump = lambda t: np.array([6.0, 0.0, 0.0]) if 6260.0 < t < 6265.0 else 0.0   # noqa: E731
    _, outs = fused(calibrated[0], 6200.0, 80.0, extra=bump)
    assert errors([x for x in outs if 6259.0 < x[0] < 6270.0]).max() < 2.5


def test_game_yaw_step_restarts_fusion(calibrated):
    """The IMU node restarts: a gap and a new origin of the game yaw."""
    e, outs = fused(calibrated[0], 6400.0, 60.0)
    assert outs[-1][2]['ok'] and e.resets == 0
    outs = simulate(e, 6463.0, 30.0, lambda t: 0.7, rtk=False, seed=22, game_offset=2.4)   # +115 deg
    assert e.resets == 1 and 'gap' in e.reset_reason
    assert 'warmup' in outs[0][2]['reasons'] and outs[0][2]['fuse_tau_eff_s'] < 1.0
    late = [x for x in outs if x[0] > 6470.0]
    assert all(o['ok'] for _, _, o in late) and errors(late).max() < 1.5
    assert e.status(outs[-1][0])['fusion']['resets'] == 1


def test_game_yaw_step_without_gap_restarts_fusion(calibrated):
    e, _ = fused(calibrated[0], 6600.0, 60.0)
    # Same stamps go on, the origin jumps by 115 deg at once.
    outs = simulate(e, 6660.0, 20.0, lambda t: 0.7, rtk=False, seed=23, game_offset=2.4)
    assert e.resets == 1 and 'step' in e.reset_reason
    assert errors([x for x in outs if x[0] > 6667.0]).max() < 1.5


def test_slow_game_yaw_offset_restarts_by_innovation(calibrated):
    """No gap, no visible step (the samples around it are lost): the magnetometer
    and the gyro stay 40 deg apart, the filter starts again after fuse_reset_s."""
    e, _ = fused(calibrated[0], 6800.0, 60.0)
    e.yaw_u += D(40.0)                      # as if the origin moved between two samples
    e.imu.clear()
    outs = simulate(e, 6860.0, 20.0, lambda t: 0.7, rtk=False, seed=24, game_offset=0.4 + D(40.0))
    assert e.resets == 1 and 'apart' in e.reset_reason
    assert errors([x for x in outs if x[0] > 6870.0]).max() < 1.5


def test_standing_freezes_correction(calibrated):
    """Standing with a holding torque: a steady 4 uT in the horizontal.  With
    the wheel odometry the correction is frozen and the gyro carries the
    heading; without it the heading creeps after the magnetometer."""
    hold = lambda t: np.array([4.0, 0.0, 0.0]) if t > 7040.0 else 0.0     # noqa: E731
    speed = lambda t: 0.3 if t < 7040.0 else 0.0                         # noqa: E731
    e, outs = fused(calibrated[0], 7000.0, 100.0, extra=hold, speed=speed)
    stand = [x for x in outs if x[0] > 7042.0]
    assert all(o['frozen'] and o['moving'] is False for _, _, o in stand)
    assert not any(set(o['reasons']) - {'field'} for _, _, o in stand)     # 4 uT is at the field gate
    assert errors(stand).max() < 1.0
    s = e.status(outs[-1][0])
    assert s['fusion']['frozen'] is True and s['fusion']['motion'] == 'standing' and 'note' not in s['fusion']
    # The same without odometry, and with the freeze switched off.
    _, free = fused(calibrated[0], 7000.0, 100.0, extra=hold)
    assert not any(o['frozen'] for _, _, o in free)
    assert errors([x for x in free if x[0] > 7090.0]).max() > 3.0
    e3 = mh.MagHeadingEstimator(calibrated[0].cal, source='uncal', self_calibrate=False, freeze_standing=False)
    off = simulate(e3, 7000.0, 100.0, lambda t: 0.7, rtk=False, seed=21, extra=hold, speed=speed)
    assert not any(o['frozen'] for _, _, o in off) and errors([x for x in off if x[0] > 7090.0]).max() > 3.0
    assert e3.status(off[-1][0])['fusion']['freeze_standing'] is False


def test_turning_on_the_spot_is_moving(calibrated):
    e, outs = fused(calibrated[0], 7200.0, 20.0, yaw_fn=spin(0.5), speed=0.0)
    assert all(o['moving'] is True and not o['frozen'] for t, _, o in outs if t > 7202.0)


def test_standing_from_the_start_still_gets_a_heading(calibrated):
    """No motion since the start: the first fuse_init_s are taken from the
    standing magnetometer, then the correction freezes."""
    e, outs = fused(calibrated[0], 7400.0, 30.0, speed=0.0)
    late = [x for x in outs if x[0] > 7407.0]
    assert all(o['ok'] for _, _, o in late) and errors(late).max() < 2.0
    assert np.mean([o['frozen'] for _, _, o in late]) > 0.9       # tops the window up as it ages
    assert 4.5 <= late[-1][2]['fuse_tau_eff_s'] <= 6.0


def test_stale_odometry_means_no_freeze(calibrated):
    e, _ = fused(calibrated[0], 7600.0, 20.0, speed=0.0)
    assert e.frozen
    outs = simulate(e, 7620.0, 5.0, lambda t: 0.7, rtk=False, seed=25)          # odometry stops
    assert not outs[-1][2]['frozen'] and outs[-1][2]['moving'] is None
    assert 'note' in e.status(outs[-1][0])['fusion']


def test_bad_field_is_not_fused(calibrated):
    """A minute in a disturbed field (dock: +10 uT down, heading 25 deg off): the
    state is disturbed, and the heading is right as soon as the field is clean."""
    dock = lambda t: np.array([8.0, 0.0, 10.0]) if 7830.0 < t < 7890.0 else 0.0   # noqa: E731
    _, outs = fused(calibrated[0], 7800.0, 110.0, extra=dock)
    inside = [o for t, _, o in outs if 7832.0 < t < 7889.0]
    assert all(o['state'] == 'disturbed' and {'field', 'inclination'} & set(o['reasons']) for o in inside)
    after = [x for x in outs if x[0] > 7892.0]
    assert all(o['ok'] for _, _, o in after) and errors(after).max() < 1.5


def test_mag_sample_waits_for_its_imu_message(calibrated):
    """Turning at 1 rad/s, the magnetometer sample arrives 80 ms before the IMU
    message of its time: its heading uses the game yaw of its own time."""
    e = mh.MagHeadingEstimator(calibrated[0].cal, source='uncal', self_calibrate=False)
    simulate(e, 8000.0, 20.0, spin(1.0), rtk=False, seed=26)
    yaw = lambda t: 1.0 * t                                     # noqa: E731
    b_w = np.array([H_EARTH * math.cos(AZ), H_EARTH * math.sin(AZ), -V_EARTH])
    t = 8020.08
    b = Rotation.from_euler('Z', yaw(t)).inv().apply(b_w) @ SOFT.T + HARD
    e.last_out_t = -1e18
    assert e.on_mag(t, *b) is None                              # no IMU at 8020.08 yet
    out = None
    for ti in (8020.0, 8020.04, 8020.08):
        q = Rotation.from_euler('Z', yaw(ti) + 0.4).as_quat()
        out = e.on_imu(ti, *q) or out
    assert out is not None and out['t'] == t
    assert abs(mh.wrap(out['yaw'] - yaw(t))) * mh.DEG < 1.0      # 80 ms late would be 4.6 deg


# -- axis weighting and the calibration format ------------------------------------

def test_axis_information():
    Q, ok = mh.axis_information([[4.0, 0.0], [0.0, 1.0]])
    assert ok and np.allclose(Q, [[0.4, 0.0], [0.0, 1.6]])       # mean eigenvalue 1, noisy axis counts less
    Q, ok = mh.axis_information([[1e6, 0.0], [0.0, 1.0]])
    assert ok and np.isclose(Q[1, 1] / Q[0, 0], 25.0)            # ratio limited
    eye = np.eye(2)
    for bad in (None, [[float('nan'), 0.0], [0.0, 1.0]], [[float('inf'), 0.0], [0.0, 1.0]],
                [[0.0, 0.0], [0.0, 0.0]], [[1.0, 0.0], [0.0, -1.0]], [1.0, 2.0, 3.0], 'x'):
        Q, ok = mh.axis_information(bad)
        assert not ok and np.array_equal(Q, eye), bad


def test_fit_estimates_residual_covariance():
    """Noise along body x only (drive current): the fit finds it and the fusion
    leans on the clean axis."""
    rng = np.random.default_rng(5)
    state = {'v': 0.0, 't': None}

    def current(t):                       # slow random walk along sensor x, ~2 uT
        if state['t'] is None or t - state['t'] >= 0.5:
            state['v'] = 0.8 * state['v'] + rng.normal(0.0, 1.5)
            state['t'] = t
        return np.array([state['v'], 0.0, 0.0])

    est = mh.MagHeadingEstimator(None, source='uncal')
    simulate(est, 0.0, 160.0, spin(), tilt_deg=(0.0, 0.0), extra=current)
    res = est.fit(160.0)
    assert res['accepted'], res
    C = est.cal.resid_cov
    assert C is not None and est.weighted
    ev, V = np.linalg.eigh(C)
    assert ev[1] > 4.0 * ev[0] and abs(V[0, 1]) > 0.9            # the noisy direction is body x
    assert est.Q[1, 1] > 2.0 * est.Q[0, 0]
    d = est.cal.to_dict()
    assert d['version'] == 2 and np.allclose(d['resid_cov'], C, atol=1e-6)


def test_old_calibration_format_loads_with_equal_axes(tmp_path, calibrated):
    """Version 1 file (no resid_cov): loads, both axes weigh the same; the first
    fit that still agrees adds the matrix without replacing the calibration."""
    import copy
    import yaml
    est, _ = calibrated
    d = est.cal.to_dict()
    d['version'] = 1
    del d['resid_cov']
    path = str(tmp_path / 'v1.yaml')
    with open(path, 'w') as fh:
        yaml.safe_dump(d, fh)
    cal, note = mh.load_calibration(path, 'uncal')
    assert cal is not None, note
    assert cal.resid_cov is None and cal.version == 1
    e = mh.MagHeadingEstimator(cal, source='uncal')
    assert not e.weighted and np.array_equal(e.Q, np.eye(2))
    outs = simulate(e, 9000.0, 30.0, lambda t: 0.7, rtk=False, seed=31)
    assert errors([x for x in outs if x[0] > 9007.0]).max() < 1.5
    s = e.status(outs[-1][0])
    assert s['calibration']['version'] == 1 and s['fusion']['axis_weighting'] is False
    # A fit refused by the ratchet still hands the matrix to the old calibration.
    cal.best_p90_deg = cal.resid_p90_deg - 2.0
    snap = est.fit_snapshot()
    res = mh.fit_calibration(snap, 'uncal', current=cal)
    assert not res['accepted'] and res.get('current_cov') is not None
    before = copy.deepcopy(cal.center)
    assert e.adopt(res, 9030.0) is True
    assert e.cal is cal and np.array_equal(cal.center, before)
    assert cal.resid_cov is not None and cal.version == 2 and e.weighted
    mh.save_calibration(path, cal)
    cal2, _ = mh.load_calibration(path, 'uncal')
    assert cal2.version == 2 and np.allclose(cal2.resid_cov, cal.resid_cov, atol=1e-6)


@pytest.mark.parametrize('cov', [[[float('nan'), 0.0], [0.0, 1.0]], [[0.0, 0.0], [0.0, 0.0]],
                                 [[float('inf'), 0.0], [0.0, 1.0]], [[1.0, 2.0], [2.0, 1.0]], [1.0], 'bad'])
def test_degenerate_residual_covariance_means_equal_axes(tmp_path, calibrated, cov):
    """A matrix that cannot be used does not refuse the calibration."""
    import yaml
    est, _ = calibrated
    d = est.cal.to_dict()
    d['resid_cov'] = cov
    path = str(tmp_path / 'deg.yaml')
    with open(path, 'w') as fh:
        yaml.safe_dump(d, fh)
    cal, note = mh.load_calibration(path, 'uncal')
    assert cal is not None, note
    assert cal.resid_cov is None
    e = mh.MagHeadingEstimator(cal, source='uncal')
    assert not e.weighted and np.array_equal(e.Q, np.eye(2))
    outs = simulate(e, 9100.0, 15.0, lambda t: 0.7, rtk=False, seed=32)
    assert outs[-1][2]['ok'] and errors(outs[-50:]).max() < 1.5


def test_unknown_calibration_version_refused(tmp_path, calibrated):
    import yaml
    d = calibrated[0].cal.to_dict()
    d['version'] = 3
    path = str(tmp_path / 'v3.yaml')
    with open(path, 'w') as fh:
        yaml.safe_dump(d, fh)
    cal, note = mh.load_calibration(path, 'uncal')
    assert cal is None and 'version' in note


# -- RTK reference only with a full carrier fix -----------------------------------

def rtk_fed(relpos, cov=0.0005, **params):
    est = mh.MagHeadingEstimator(None, source='uncal', **params)
    rng = np.random.default_rng(1)
    used = 0
    for i in range(3000):
        ti = 100.0 + 0.01 * i
        q = Rotation.from_euler('Z', 0.3 * ti + 0.4).as_quat()
        est.on_imu(ti, *q)
        if i % 5 == 0:
            est.on_mag(ti, 30.0 + rng.normal(), 20.0, -40.0)
        if i % 33 == 0:
            if relpos is not None:
                est.on_relpos(ti, relpos)
            r = Rotation.from_euler('Z', 0.3 * ti).as_quat()
            used += bool(est.on_rtk(ti, r[0], r[1], r[2], r[3], cov))
    return est, used


def test_rtk_without_full_fix_is_not_a_reference():
    est, used = rtk_fed(FIXED)
    assert used > 80 and est.bins.total() > 0 and est.rtk_state(est.t_imu) == (True, 'ok')
    for flags, word in ((FLOAT, 'carrSoln 1'), (0x100 | 0x07, 'carrSoln 0'), (0x10 | 0x07, 'not valid')):
        est, used = rtk_fed(flags)
        assert used == 0 and est.rtk_used == 0 and est.bins.total() == 0
        valid, why = est.rtk_state(est.t_imu)
        assert valid is False and word in why, why
        s = est.status(est.t_imu)
        assert s['rtk_compare']['rtk_valid'] is False and word in s['rtk_compare']['rtk_reason']
        assert s['rtk_compare'].get('all') is None


def test_rtk_without_relposned_is_not_a_reference():
    est, used = rtk_fed(None)
    assert used == 0 and est.bins.total() == 0
    s = est.status(est.t_imu)
    assert s['rtk_compare']['rtk_valid'] is False and 'NAV-RELPOSNED' in s['rtk_compare']['rtk_reason']
    assert s['rtk_compare']['rtk_require_fixed'] is True
    # Switched off (a bag without the topic): the variance band alone.
    est, used = rtk_fed(None, rtk_require_fixed=False)
    assert used > 80 and est.bins.total() > 0


def test_rtk_stale_relposned_is_not_a_reference():
    est, _ = rtk_fed(FIXED)
    t = est.t_imu
    r = Rotation.from_euler('Z', 0.3 * t).as_quat()
    assert est.on_rtk(t, r[0], r[1], r[2], r[3], 0.0005, t_rx=t + 1.0) is False     # RELPOSNED 1 s older
    assert 'stale' in est.rtk_reason
    est.on_relpos(t + 1.0, FIXED)
    assert est.on_rtk(t, r[0], r[1], r[2], r[3], 0.0005, t_rx=t + 1.0) is True


@pytest.mark.parametrize('cov', [0.0001, 0.002, 1000.0, float('nan')])
def test_rtk_variance_outside_the_band_is_not_a_reference(cov):
    """Fixed, but the yaw variance is off the healthy band (navi_transform:
    0.00043..0.00079): too tight is a degenerate solution, too wide is poor."""
    est, used = rtk_fed(FIXED, cov=cov)
    assert used == 0 and 'variance' in est.rtk_reason


# -- sigma covers what the gate cannot see ----------------------------------------

@pytest.mark.parametrize('amp', [3.0, 5.0])
def test_long_steady_disturbance_is_covered_by_sigma(calibrated, amp):
    """A steady horizontal field within the field gate for 3 min (a long car, a
    fence): the correction follows it by 8 / 12 deg.  Every output is either
    disturbed or ok with error <= 2 sigma; after the field is clean again the
    heading comes back within two minutes."""
    e = mh.MagHeadingEstimator(calibrated[0].cal, source='uncal', self_calibrate=False)
    outs = simulate(e, 3000.0, 400.0, lambda t: 0.7 + 0.1 * math.sin(0.3 * t), rtk=False, seed=5,
                    extra=lambda t: np.array([0.0, amp, 0.0]) if 3060.0 < t < 3240.0 else 0.0)
    worst = 0.0
    for t, y, o in outs:
        if o['ok']:
            worst = max(worst, abs(mh.wrap(o['yaw'] - y)) * mh.DEG / o['sigma_deg'])
    assert worst <= 2.0, worst
    inside = [x for x in outs if 3200.0 < x[0] < 3240.0]
    assert errors(inside).max() > 6.0                              # the heading did follow the field
    ok_in = [o for _, _, o in inside if o['ok']]
    assert all(o['sigma_deg'] > 4.2 for o in ok_in)                # and sigma says so
    assert errors([x for x in outs if x[0] > 3360.0]).max() < 3.0  # back 2 min after the end
    assert outs[-1][2]['correction_range_deg'] > 6.0               # still remembered (sigma_window_s)
    # Clean run of the same length: sigma stays at sigma_ok_deg.
    e2 = mh.MagHeadingEstimator(calibrated[0].cal, source='uncal', self_calibrate=False)
    clean = simulate(e2, 3000.0, 150.0, lambda t: 0.7 + 0.1 * math.sin(0.3 * t), rtk=False, seed=5)
    assert all(o['sigma_deg'] == 4.0 for t, _, o in clean if t > 3060.0)


def test_long_standing_holds_then_refreshes(calibrated):
    """Standing with a holding-torque field and a drifting gyro: the correction
    of the drive is held for freeze_max_s (window unchanged, sigma_ok_deg),
    then the window is cut and the standing magnetometer is let in (sigma_start_deg)."""
    e = mh.MagHeadingEstimator(calibrated[0].cal, source='uncal', self_calibrate=False, freeze_max_s=200.0)
    outs = simulate(e, 5000.0, 330.0, lambda t: 0.7, rtk=False, seed=7, tilt_deg=(0.2, 0.2),
                    gyro_drift=D(0.5) / 60.0, speed=lambda t: 0.3 if t < 5060.0 else 0.0,
                    extra=lambda t: np.array([0.0, 3.0, 0.0]) if t > 5060.0 else 0.0)
    held = [x for x in outs if 5075.0 < x[0] < 5255.0]
    w = [o['fuse_tau_eff_s'] for _, _, o in held]
    assert all(o['frozen'] and o['ok'] and o['sigma_deg'] == 4.0 for _, _, o in held)
    assert max(w) - min(w) < 0.01 and min(w) > 30.0               # the window does not shrink while held
    assert errors(held).max() < 2.5                               # gyro drift only (0.5 deg/min here)
    assert 150.0 < held[-1][2]['hold_age_s'] < 200.0
    late = [x for x in outs if x[0] > 5275.0]
    assert all(o['ok'] and o['sigma_deg'] == 8.0 for _, _, o in late)
    assert all(2.0 < o['fuse_tau_eff_s'] < 6.0 for _, _, o in late)
    e_late = errors(late)
    assert 1.5 < e_late.max() < 16.0                              # the standing magnetometer came in, within 2 sigma


def test_long_hold_grows_sigma_with_gyro_drift(calibrated):
    e, outs = fused(calibrated[0], 5500.0, 60.0, speed=lambda t: 0.3)
    assert outs[-1][2]['sigma_deg'] == 4.0
    e.f_used_t -= 3600.0                                           # as if frozen for an hour
    assert 7.2 <= e._sigma(e.t_mag, 0.0)[0] < 8.2      # sigma_k x (0.2 deg/min x 60 min + range of the correction)


# -- IMU outage, queue of waiting samples -----------------------------------------

def test_imu_outage_queue_is_bounded_and_fusion_restarts(calibrated):
    e, outs = fused(calibrated[0], 9300.0, 30.0)
    assert outs[-1][2]['ok']
    b_w = np.array([H_EARTH * math.cos(AZ), H_EARTH * math.sin(AZ), -V_EARTH])
    b = Rotation.from_euler('Z', 0.7).inv().apply(b_w) @ SOFT.T + HARD
    got = []
    for i in range(600):                                           # 30 s of magnetometer, no IMU
        t = 9330.0 + 0.05 * i
        got.append(e.on_mag(t, b[0] + 1e-3 * i, b[1], b[2]))
    assert all(o is None for o in got)
    assert len(e.pending) <= 64 and len(e.pending) < 10            # only while the IMU is fresh enough
    assert e.status(9345.0)['state'] == 'stale'
    outs = simulate(e, 9360.0, 20.0, lambda t: 0.7, rtk=False, seed=41, game_offset=1.9)   # IMU back, new origin
    assert e.resets == 1 and 'gap' in e.reset_reason and len(e.pending) == 0
    assert 'warmup' in outs[0][2]['reasons'] and outs[0][0] >= 9360.0
    late = [x for x in outs if x[0] > 9367.0]
    assert all(o['ok'] for _, _, o in late) and errors(late).max() < 1.5


def test_waiting_samples_older_than_the_imu_limit_are_dropped(calibrated):
    e, _ = fused(calibrated[0], 9400.0, 10.0)
    b = (30.0, 20.0, -40.0)
    assert e.on_mag(9410.05, *b) is None and len(e.pending) == 1
    e.last_out_t = -1e18
    q = Rotation.from_euler('Z', 1.1).as_quat()
    assert e.on_imu(9410.6, *q) is None                           # 0.55 s later: too old to pair
    assert len(e.pending) == 0 and e.t_mag < 9410.0


def test_game_yaw_step_while_frozen(calibrated):
    """The IMU node restarts while the robot stands: the frozen correction is
    void, the heading comes from the standing magnetometer again."""
    e, outs = fused(calibrated[0], 9500.0, 40.0, speed=lambda t: 0.3 if t < 9520.0 else 0.0)
    assert outs[-1][2]['frozen'] and outs[-1][2]['ok']
    outs = simulate(e, 9543.0, 20.0, lambda t: 0.7, rtk=False, seed=42, game_offset=-1.3, speed=0.0)
    assert e.resets == 1 and 'gap' in e.reset_reason
    assert not outs[0][2]['frozen'] and 'warmup' in outs[0][2]['reasons']
    late = [x for x in outs if x[0] > 9551.0]
    assert all(o['ok'] and o['sigma_deg'] == 8.0 for _, _, o in late) and errors(late).max() < 2.0
    assert np.mean([o['frozen'] for _, _, o in late]) > 0.9


# -- the version 1 file is kept when the format changes ---------------------------

def test_version_1_file_is_kept_aside_once(tmp_path, calibrated):
    import yaml
    est, _ = calibrated
    d = est.cal.to_dict()
    d['version'] = 1
    del d['resid_cov']
    path = str(tmp_path / 'mag_calibration.yaml')
    with open(path, 'w') as fh:
        fh.write('# old node\n')
        yaml.safe_dump(d, fh)
    original = open(path, 'rb').read()
    copy = str(tmp_path / 'mag_calibration.v1.yaml')
    mh.save_calibration(path, est.cal)                             # first version 2 write
    assert sorted(os.listdir(str(tmp_path))) == ['mag_calibration.v1.yaml', 'mag_calibration.yaml']
    assert open(copy, 'rb').read() == original
    assert yaml.safe_load(open(path))['version'] == 2
    # What a node from before version 2 does with the two files.
    with pytest.raises(ValueError):
        mh.Calibration.from_dict(dict(yaml.safe_load(open(path)), version=99))
    assert yaml.safe_load(open(copy))['version'] == 1 and mh.load_calibration(copy, 'uncal')[0] is not None
    mh.save_calibration(path, est.cal)                             # version 2 over version 2: nothing new
    assert open(copy, 'rb').read() == original
    # An existing copy is never replaced, even by another version 1 file.
    with open(path, 'w') as fh:
        yaml.safe_dump(dict(d, offset_deg=12.0), fh)
    mh.save_calibration(path, est.cal)
    assert open(copy, 'rb').read() == original
    assert sorted(os.listdir(str(tmp_path))) == ['mag_calibration.v1.yaml', 'mag_calibration.yaml']


def test_no_copy_without_an_old_file(tmp_path, calibrated):
    path = str(tmp_path / 'new' / 'mag_calibration.yaml')
    mh.save_calibration(path, calibrated[0].cal)
    mh.save_calibration(path, calibrated[0].cal)
    assert os.listdir(str(tmp_path / 'new')) == ['mag_calibration.yaml']
    with open(path, 'w') as fh:
        fh.write('::: broken [')
    mh.save_calibration(path, calibrated[0].cal)                   # unreadable: overwritten, no copy
    assert os.listdir(str(tmp_path / 'new')) == ['mag_calibration.yaml']


# -- the node: callbacks with a stand-in rospy -------------------------------------

def test_node_callbacks_with_stub_rospy(tmp_path, calibrated):
    """nodes/mag_heading imported with a stand-in rospy (no master) and real
    message classes: one pass of every callback, the published heading and status."""
    genpy = pytest.importorskip('genpy')
    pytest.importorskip('sensor_msgs.msg')
    pytest.importorskip('nav_msgs.msg')
    import importlib.machinery
    import importlib.util
    import types
    import yaml
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, MagneticField

    d = calibrated[0].cal.to_dict()
    d['version'] = 1
    del d['resid_cov']
    cal_file = str(tmp_path / 'mag_calibration.yaml')
    with open(cal_file, 'w') as fh:
        yaml.safe_dump(d, fh)
    params = {'~calibration_file': cal_file, '~fit_interval_s': 45.0}
    clock = {'t': 0.0}
    published = {}
    subscribed = []

    class Pub(object):
        def __init__(self, topic, cls, **kw):
            self.topic = topic
            published[topic] = []

        def publish(self, msg):
            published[self.topic].append(msg)

    class Time(genpy.Time):
        @classmethod
        def now(cls):
            return cls.from_sec(clock['t'])

    rp = types.ModuleType('rospy')
    rp.get_param = lambda name, default=None: params.get(name, default)
    rp.Publisher = Pub
    rp.Subscriber = lambda topic, cls, cb, **kw: subscribed.append((topic, cls.__name__))
    rp.Timer = lambda period, cb: None
    rp.Duration = lambda s: s
    rp.Time = Time
    rp.get_time = lambda: clock['t']
    for name in ('loginfo', 'logwarn', 'logerr', 'loginfo_throttle', 'logwarn_throttle'):
        setattr(rp, name, lambda *a, **k: None)
    saved = sys.modules.get('rospy')
    sys.modules['rospy'] = rp
    try:
        path = os.path.join(os.path.dirname(HERE), 'nodes', 'mag_heading')
        loader = importlib.machinery.SourceFileLoader('mag_heading_node_under_test', path)
        mod = importlib.util.module_from_spec(importlib.util.spec_from_loader(loader.name, loader))
        loader.exec_module(mod)
        node = mod.MagHeadingNode()
    finally:
        if saved is None:
            del sys.modules['rospy']
        else:
            sys.modules['rospy'] = saved
    assert node.est.cal is not None and node.est.cal.version == 1
    topics = dict(subscribed)
    assert topics['/bno085/imu'] == 'Imu' and topics['/bno085/mag_uncal'] == 'MagneticField'
    assert topics['/gnss_heading/navheading'] == 'Imu' and topics['/mobile_base_controller/odom'] == 'Odometry'
    has_relpos = mod.NavRELPOSNED9 is not None
    assert ('/gnss_heading/navrelposned' in topics) == has_relpos

    class Feed(object):                       # the estimator interface of simulate(), as ROS messages
        def stamp(self, m, t):
            clock['t'] = t + 0.002
            m.header.stamp = genpy.Time.from_sec(t)
            return m

        def on_imu(self, t, x, y, z, w):
            m = self.stamp(Imu(), t)
            m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = x, y, z, w
            node.on_imu(m)

        def on_mag(self, t, bx, by, bz):
            m = self.stamp(MagneticField(), t)
            m.magnetic_field.x, m.magnetic_field.y, m.magnetic_field.z = bx * 1e-6, by * 1e-6, bz * 1e-6
            node.on_mag(m)

        def on_rtk(self, t, x, y, z, w, cov):
            m = self.stamp(Imu(), t)
            m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = x, y, z, w
            m.orientation_covariance = [1e3, 0, 0, 0, 1e3, 0, 0, 0, cov]
            node.on_rtk(m)

        def on_relpos(self, t, flags):
            if has_relpos:
                clock['t'] = t + 0.001
                node.on_relpos(mod.NavRELPOSNED9(flags=flags))

        def on_motion(self, t, v):
            m = self.stamp(Odometry(), t)
            m.twist.twist.linear.x = v
            node.on_motion(m)

    simulate(Feed(), 100.0, 12.0, spin(0.3), speed=0.3)
    node.on_status(None)
    out = published['bno085/mag_heading']
    assert len(out) > 80                                           # ~10 Hz once calibrated
    last = out[-1]
    yaw = 2.0 * math.atan2(last.orientation.z, last.orientation.w)
    assert abs(mh.wrap(yaw - 0.3 * last.header.stamp.to_sec())) * mh.DEG < 2.0
    assert last.header.frame_id == 'bno_imu_link'
    assert abs(math.degrees(math.sqrt(last.orientation_covariance[8])) - 8.0) < 1e-6   # ok, window still growing
    assert last.orientation_covariance[0] == 1000.0 and out[0].orientation_covariance[8] == 1000.0   # warmup
    s = json.loads(published['bno085/mag_heading/status'][-1].data)
    assert s['state'] == 'ok' and s['calibration_file'] == cal_file and s['mag_topic'] == '/bno085/mag_uncal'
    assert s['fusion']['motion'] == 'moving' and s['calibration']['version'] == 1
    assert s['rtk_compare']['rtk_valid'] is has_relpos
    if not has_relpos:
        assert 'NAV-RELPOSNED' in s['rtk_compare']['rtk_reason']
    node.on_fit(None)                                              # too few directions: refused, file untouched
    assert node.est.last_fit is not None and not node.est.last_fit['accepted']
    assert yaml.safe_load(open(cal_file))['version'] == 1
    assert os.listdir(str(tmp_path)) == ['mag_calibration.yaml']
