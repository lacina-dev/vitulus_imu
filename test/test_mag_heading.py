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


def simulate(est, t0, duration, yaw_fn, tilt_deg=(3.0, 4.0), rtk=True, gyro_drift=0.0, extra=None,
             noise=0.3, rtk_noise_deg=0.5, seed=1, game_offset=0.4, tell_tilt=True, hard=HARD):
    """Feeds est like the robot: IMU 100 Hz, mag 20 Hz, RTK 3 Hz.  Returns a list of
    (t, true yaw, output dict) for every output."""
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
        if rtk and i % 33 == 0:
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
    assert after[-1][2]['sigma_deg'] == 6.0


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
    _, outs = gated(calibrated[0], extra=lambda t: np.array([8.0 * math.sin(3.0 * t), 0.0, 0.0])
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
