"""T35 I3 (2026-11-16), from the 18 night drives dock -> TEST -> dock:
  a self-calibration sample is taken only when the RTK heading stayed valid
  sample_confirm_s after it (default 0 = off: the tests in test_mag_heading.py)
"""
import json
import os

import numpy as np

import test_mag_heading as T
from test_mag_heading import calibrated  # noqa: F401 - fixture
from vitulus_imu import mag_heading as mh

assert json and os and np            # used by some of the tests below


def sample_times(est):
    return np.sort(est.bins.snapshot()['t'])


def test_samples_wait_for_rtk_to_stay_valid(calibrated):  # noqa: F811
    cal = calibrated[0].cal
    a = mh.MagHeadingEstimator(cal, source='uncal')                                     # at once, as before
    b = mh.MagHeadingEstimator(cal, source='uncal', sample_confirm_s=10.0)
    for e in (a, b):
        T.simulate(e, 7000.0, 40.0, T.spin())
    ta, tb = sample_times(a), sample_times(b)
    assert len(ta) > 130 and ta.min() >= 7005.0 and ta.max() > 7039.5                   # 5 s settle, then 4 Hz
    assert len(tb) > 90 and tb.max() <= 7030.0 + 0.35 and tb.max() > 7029.0             # all but the last 10 s
    assert np.array_equal(tb, ta[:len(tb)]) and len(b.sample_wait) == len(ta) - len(tb)
    assert b.samples_unconfirmed == 0
    # RTK lost: what waited is dropped, nothing from the last 10 s gets in
    for e in (a, b):
        T.simulate(e, 7040.0, 5.0, T.spin(), rtk=False)
    ta2 = sample_times(a)                                  # rtk_hold_s: the reference lives 1 s longer
    assert len(b.sample_wait) == 0 and b.samples_unconfirmed == len(ta2) - len(tb)
    assert np.array_equal(sample_times(b), tb) and len(ta) <= len(ta2) <= len(ta) + 5
    st = b.status(7045.0)['self_cal']
    assert st['sample_confirm_s'] == 10.0 and st['samples_waiting'] == 0
    assert st['samples_unconfirmed'] == b.samples_unconfirmed and st['samples'] == len(tb)


def test_samples_of_a_broken_rtk_stretch_are_dropped(calibrated):  # noqa: F811
    cal = calibrated[0].cal
    b = mh.MagHeadingEstimator(cal, source='uncal', sample_confirm_s=10.0)
    T.simulate(b, 7000.0, 12.0, T.spin())                  # samples 7005..7012 wait
    assert len(b.sample_wait) > 20 and b.bins.total() == 0
    T.simulate(b, 7012.0, 3.0, T.spin(), relpos=T.FLOAT)   # RTK not FIXED for 3 s: the stretch ends
    assert len(b.sample_wait) == 0 and b.bins.total() == 0 and b.samples_unconfirmed > 20
    T.simulate(b, 7015.0, 30.0, T.spin())                  # a new stretch: 5 s settle, then confirmed 10 s later
    tb = sample_times(b)
    assert len(tb) > 50 and tb.min() >= 7020.0 and tb.max() <= 7035.0 + 0.35
    # a restart of the gyro origin drops the waiting ones too
    n = b.samples_unconfirmed
    w = len(b.sample_wait)
    assert w > 0
    b._gyro_restart('test')
    assert len(b.sample_wait) == 0 and b.samples_unconfirmed == n + w


def test_fit_uses_only_confirmed_samples():
    """Self-calibration from scratch with the delay: the same calibration as
    without it on the same samples (80 s of them)."""
    est = mh.MagHeadingEstimator(None, source='uncal', sample_confirm_s=10.0)
    T.simulate(est, 1000.0, 90.0, T.spin())
    res = est.fit(1090.0)
    ref = mh.MagHeadingEstimator(None, source='uncal')
    T.simulate(ref, 1000.0, 80.0, T.spin())
    res0 = ref.fit(1080.0)
    assert res['accepted'] and res0['accepted'] and res['n'] == est.bins.total()
    assert abs(res['n'] - res0['n']) <= 2 and abs(res['p90_deg'] - res0['p90_deg']) < 0.3
    assert abs(mh.wrap(est.cal.offset - ref.cal.offset)) * mh.DEG < 0.3
    assert sample_times(est).max() <= 1080.0 + 0.35




def test_off_by_default_and_a_node_parameter(tmp_path, calibrated):  # noqa: F811
    assert mh.MagHeadingEstimator.DEFAULTS['sample_confirm_s'] == 0.0
    cal_file = str(tmp_path / 'mag_calibration.yaml')
    mh.save_calibration(cal_file, calibrated[0].cal)
    node, _, _ = T.samples_node({'~calibration_file': cal_file})
    assert node.est.p['sample_confirm_s'] == 0.0 and 'sample_confirm_s' not in node.est.status(0.0)['self_cal']
    node, _, _ = T.samples_node({'~calibration_file': cal_file, '~sample_confirm_s': 10.0})
    assert node.est.p['sample_confirm_s'] == 10.0 and node.est.status(0.0)['self_cal']['samples_waiting'] == 0
