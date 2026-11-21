"""T35 (2026-11-21): the anchor that RTK has not confirmed is seated again
after a start inside a disturbance (anchor_reseat_s, default off: the tests in
test_mag_heading.py).
"""
import numpy as np

import test_mag_heading as T
from test_mag_heading import calibrated  # noqa: F401 - fixture
from vitulus_imu import mag_heading as mh


def pair(cal, t0, duration, t_split=None, **kw):
    """The same feed through an estimator with anchor_reseat_s off and on (120 s)."""
    res = []
    for reseat in (0.0, 120.0):
        e = mh.MagHeadingEstimator(cal, source='uncal', self_calibrate=False, anchor_reseat_s=reseat)
        res.append((e, T.simulate(e, t0, duration, T.WOBBLE, seed=5, **kw)))
    return res


def share(outs, lo, hi, reason='anchor'):
    return float(np.mean([reason in o['reasons'] for t, _, o in outs if lo < t <= hi]))


def test_start_inside_a_disturbance_then_out_reseats_the_anchor(calibrated):  # noqa: F811
    """No RTK, the filter starts inside 5 uT (12 deg) and leaves it after 200 s.
    Off: the heading is right again but the state is disturbed / anchor for
    minutes.  On: the anchor is seated at the settled correction, the state
    is ok; the heading is the same and sigma is never below the old one."""
    t0 = 7000.0
    field = lambda t: np.array([0.0, 5.0, 0.0]) if t < t0 + 200.0 else 0.0     # noqa: E731
    (off, o_off), (on, o_on) = pair(calibrated[0].cal, t0, 800.0, rtk=False, extra=field)
    assert [o['yaw'] for _, _, o in o_on] == [o['yaw'] for _, _, o in o_off]
    inside = [x for x in o_off if t0 + 100.0 < x[0] < t0 + 200.0]
    assert T.errors(inside).min() > 9.0                                    # started on the disturbed value
    assert share(o_off, t0 + 420.0, t0 + 620.0) > 0.6                      # as before: falsely disturbed
    assert off.anc_reseats == 0 and 'anchor_reseats' not in o_off[-1][2]
    assert 'anchor_reseats' not in off.status(t0 + 800.0)['fusion']
    assert T.errors([x for x in o_off if x[0] > t0 + 420.0]).max() < 3.0   # while the heading is right
    # on: one re-seat within 4.5 min of leaving (2.5 min to settle within 3 deg + 120 s)
    t_seat = min(t for t, _, o in o_on if o['anchor_reseats'] == 1)
    assert t0 + 320.0 < t_seat < t0 + 470.0, t_seat - t0
    after = [o for t, _, o in o_on if t > t_seat]
    assert all(o['ok'] and o['anchor'] == 'start' and o['anchor_dev_deg'] < 3.0 for o in after)
    assert on.anc_reseats == 1 and 10.0 < on.anc_moved < 14.0
    assert all(o['sigma_deg'] == 8.0 for o in after)                       # max(8, 0.6 x (dev + 12)) = 8
    for (_, _, a), (_, _, b) in zip(o_on, o_off):
        assert not (a['ok'] and b['ok']) or a['sigma_deg'] >= b['sigma_deg'] - 1e-9
    st = on.status(t0 + 800.0)['fusion']
    assert st['anchor_reseats'] == 1 and st['anchor_reseat_s'] == 120.0 and 10.0 < st['anchor_moved_deg'] < 14.0
    # RTK arrives: the anchor is confirmed, the remembered step is dropped
    outs = T.simulate(on, t0 + 800.0, 20.0, T.WOBBLE, seed=6)
    assert outs[-1][2]['anchor'] == 'rtk' and outs[-1][2]['sigma_deg'] == 4.0 and on.anc_moved == 0.0


def test_large_step_keeps_sigma_over_the_step(calibrated):  # noqa: F811
    """7 uT (17 deg): after the re-seat the state is ok, sigma still covers
    the step (0.6 x 17 > 8), as it did before through the deviation."""
    t0 = 7000.0
    field = lambda t: np.array([0.0, 7.0, 0.0]) if t < t0 + 150.0 else 0.0     # noqa: E731
    (off, o_off), (on, o_on) = pair(calibrated[0].cal, t0, 600.0, rtk=False, extra=field)
    assert on.anc_reseats == 1 and on.anc_moved > 14.0
    end_on = [o for t, _, o in o_on if t > t0 + 500.0]
    end_off = [o for t, _, o in o_off if t > t0 + 500.0]
    assert all(o['ok'] and o['sigma_deg'] >= 0.6 * on.anc_moved for o in end_on)
    assert all('anchor' in o['reasons'] for o in end_off)


def test_clean_start_is_unchanged(calibrated):  # noqa: F811
    """No disturbance: nothing is seated again, without RTK and with it."""
    for rtk in (False, True):
        (off, o_off), (on, o_on) = pair(calibrated[0].cal, 7000.0, 400.0, rtk=rtk, gyro_drift=T.D(0.19) / 60.0)
        assert on.anc_reseats == 0 and on.anc_moved == 0.0
        assert len(o_on) == len(o_off)
        for (_, _, a), (_, _, b) in zip(o_on, o_off):
            assert a.pop('anchor_reseats') == 0 and a.pop('anchor_moved_deg') == 0.0
            assert a == b


def test_confirmed_anchor_is_never_reseated(calibrated):  # noqa: F811
    """RTK confirmed the anchor, then 5 uT for 400 s without RTK: the anchor
    stays where RTK left it, the state is disturbed / anchor as before."""
    res = []
    for reseat in (0.0, 120.0):
        e, t0 = T.anchored(calibrated[0], 2940.0, anchor_reseat_s=reseat)
        res.append((e, T.simulate(e, t0, 500.0, T.WOBBLE, rtk=False, seed=5,
                                  extra=lambda t: np.array([0.0, 5.0, 0.0]) if 3060.0 < t < 3460.0 else 0.0)))
    (off, o_off), (on, o_on) = res
    assert on.anc_reseats == 0 and on.anc[2] == 'rtk'
    assert [(o['state'], o['reasons'], o['sigma_deg']) for _, _, o in o_on] == \
        [(o['state'], o['reasons'], o['sigma_deg']) for _, _, o in o_off]
    assert share(o_on, 3200.0, 3440.0) > 0.9


def test_standing_frozen_time_does_not_count(calibrated):  # noqa: F811
    """Standing (correction frozen) is no evidence: the candidate does not age."""
    e = mh.MagHeadingEstimator(calibrated[0].cal, source='uncal', self_calibrate=False, anchor_reseat_s=60.0)
    t0 = 7000.0
    field = lambda t: np.array([0.0, 5.0, 0.0]) if t < t0 + 60.0 else 0.0      # noqa: E731
    T.simulate(e, t0, 60.0, T.WOBBLE, rtk=False, seed=5, extra=field, speed=0.3)
    assert e.anc is not None and e.anc[2] == 'start'
    before = e.anc_cand[1]                                                 # fused seconds counted while driving
    assert 5.0 < before < 31.0
    T.simulate(e, t0 + 60.0, 300.0, lambda t: T.WOBBLE(t0 + 60.0), rtk=False, seed=6, speed=0.0)
    assert e.frozen and e.anc_reseats == 0
    assert e.anc_cand[1] - before < 2.0                                    # 300 s standing: not counted
