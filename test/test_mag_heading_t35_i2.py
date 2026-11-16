"""T35 I2 (2026-11-16), from the 18 night drives dock -> TEST -> dock:
  sigma follows the error measured against the RTK heading (sigma_from_rtk,
  default off: the tests in test_mag_heading.py)
"""
import json
import os

import numpy as np

import test_mag_heading as T
from test_mag_heading import calibrated  # noqa: F401 - fixture
from vitulus_imu import mag_heading as mh

assert json and os and np            # used by some of the tests below


def sigmas(outs):
    return np.array([o['sigma_deg'] for _, _, o in outs if o['ok']])


def pair(cal, t0, duration, **kw):
    """The same feed through an estimator with sigma_from_rtk off and on."""
    res = []
    for on in (False, True):
        e = mh.MagHeadingEstimator(cal, source='uncal', self_calibrate=False, sigma_from_rtk=on)
        res.append((e, T.simulate(e, t0, duration, T.WOBBLE, seed=3, **kw)))
    return res


def test_sigma_follows_the_error_measured_against_rtk(calibrated):  # noqa: F811
    (off, o_off), (on, o_on) = pair(calibrated[0].cal, 5000.0, 240.0)
    late_off = [x for x in o_off if x[0] > 5120.0]
    late_on = [x for x in o_on if x[0] > 5120.0]
    assert all(o['ok'] and o['sigma_deg'] == 4.0 for _, _, o in late_off)               # as before
    assert off.sigma_measured() is None and 'sigma' not in off.status(5240.0)            # off: nothing measured
    assert on.sigma_measured() is not None
    rms = on.sigma_measured()
    assert 0.2 < rms < 1.5                                                               # RTK noise 0.5 deg + own error
    s = sigmas(late_on)
    assert len(s) == len(late_on) and np.all(s >= 1.5) and np.all(s < 2.5)               # floor .. 1.5 x rms
    assert abs(s[-1] - max(1.5, 1.5 * rms)) < 0.05
    # the same headings, only sigma differs; never above the old one
    assert [o['yaw'] for _, _, o in o_on] == [o['yaw'] for _, _, o in o_off]
    assert all(a['sigma_deg'] <= b['sigma_deg'] for (_, _, a), (_, _, b) in zip(o_on, o_off) if a['ok'])
    e = T.errors(late_on)
    assert np.mean(e <= 2.0 * s) >= 0.95 and T.covered(late_on) < 3.0
    st = on.status(5240.0)['sigma']
    assert st['from_rtk'] is True and abs(st['measured_rms_deg'] - round(rms, 2)) < 1e-9 and st['measured_s'] > 200
    json.dumps(on.status(5240.0))


def test_sigma_needs_30_s_of_comparisons_and_a_confirmed_anchor(calibrated):  # noqa: F811
    e = mh.MagHeadingEstimator(calibrated[0].cal, source='uncal', self_calibrate=False, sigma_from_rtk=True)
    outs = T.simulate(e, 5000.0, 90.0, T.WOBBLE, seed=3)
    s = {round(t - 5000.0): o['sigma_deg'] for t, _, o in outs if o['ok']}
    assert s[20] == 8.0                                   # window not settled: sigma_start_deg as before
    first_low = min(k for k, v in s.items() if v < 4.0)
    assert 35 <= first_low <= 45, first_low               # 5 s RTK settle + 30 s of comparisons
    assert all(v in (8.0, 4.0) for k, v in s.items() if k < first_low)
    # without RTK nothing is measured: sigma as before
    e2 = mh.MagHeadingEstimator(calibrated[0].cal, source='uncal', self_calibrate=False, sigma_from_rtk=True)
    outs2 = T.simulate(e2, 5000.0, 90.0, T.WOBBLE, seed=3, rtk=False)
    assert e2.sigma_measured() is None and set(sigmas(outs2)) == {8.0}


def test_measured_sigma_is_kept_without_rtk_and_grows_in_a_disturbance(calibrated):  # noqa: F811
    """After RTK is lost the measured base stays; a steady field within the
    gate (the correction follows it) raises sigma as before: every ok output
    stays within 2 sigma."""
    amp = 3.0
    res = []
    for on in (False, True):
        e = mh.MagHeadingEstimator(calibrated[0].cal, source='uncal', self_calibrate=False, sigma_from_rtk=on)
        T.simulate(e, 2880.0, 120.0, T.WOBBLE, seed=3)
        outs = T.simulate(e, 3000.0, 400.0, T.WOBBLE, rtk=False, seed=5,
                          extra=lambda t: np.array([0.0, amp, 0.0]) if 3060.0 < t < 3240.0 else 0.0)
        res.append((e, outs))
    (off, o_off), (on, o_on) = res
    before = [x for x in o_on if x[0] < 3055.0]
    assert np.all(sigmas(before) < 2.5) and np.all(sigmas(before) >= 1.5)                # base kept without RTK
    inside_on = [o for t, _, o in o_on if 3200.0 < t < 3240.0 and o['ok']]
    inside_off = [o for t, _, o in o_off if 3200.0 < t < 3240.0 and o['ok']]
    assert inside_on and all(o['sigma_deg'] > 4.2 for o in inside_on)                    # grows in the disturbance
    assert [o['sigma_deg'] for o in inside_on] == [o['sigma_deg'] for o in inside_off]   # ... exactly as before
    assert T.covered(o_on) <= 2.0, T.covered(o_on)
    back = [x for x in o_on if x[0] > 3370.0]
    assert np.all(sigmas(back[-50:]) < 2.5)                                               # and comes back down


def test_sigma_parameters(calibrated):  # noqa: F811
    e = mh.MagHeadingEstimator(calibrated[0].cal, source='uncal', self_calibrate=False, sigma_from_rtk=True,
                               sigma_min_deg=3.0)
    outs = T.simulate(e, 5000.0, 120.0, T.WOBBLE, seed=3)
    assert sigmas(outs)[-1] == 3.0                                                       # the floor
    e = mh.MagHeadingEstimator(calibrated[0].cal, source='uncal', self_calibrate=False, sigma_from_rtk=True,
                               sigma_rtk_k=50.0)
    outs = T.simulate(e, 5000.0, 120.0, T.WOBBLE, seed=3)
    assert sigmas(outs)[-1] == 4.0                                                       # never above sigma_ok_deg
    assert mh.MagHeadingEstimator.DEFAULTS['sigma_from_rtk'] is False
