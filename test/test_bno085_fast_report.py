"""~fast_report_interval_us of bno085_node: the report interval of the
accelerometer, the gyroscope and the game rotation vector.

  nice -n 10 python3 -m pytest test/test_bno085_fast_report.py -q

The node against the fake sensor on a pty and a private roscore (port 11877).
Runs for the Python node and, with BNO085_CPP_NODE=/path/to/bno085_node_cpp
(built outside the live workspace), for the C++ node as well.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bno085_session import ALL, DRIVERS, FAST, OLD_KEYS, driver, rate, repeats, ros, sent, session  # noqa: E402,F401


def test_default_is_as_before(ros, driver):
    r = session(ros, driver)
    assert r.alive
    assert r.requests == [(f, 50000) for f in ALL]
    assert list(r.health) == OLD_KEYS, 'no new health keys without the parameter'
    for topic in ('imu', 'navheading', 'mag'):
        assert 80 < rate(r, topic) < 110, '%s %.1f Hz' % (topic, rate(r, topic))
    assert repeats(r.keys['imu']) > 0.6 * len(r.keys['imu']), '20 Hz samples in 100 Hz messages'


def test_same_bytes_with_the_default_written_out(ros, driver):
    plain = session(ros, driver, seconds=0.3)
    explicit = session(ros, driver, ['_fast_report_interval_us:=50000'], seconds=0.3)
    assert sent(plain.raw) == sent(explicit.raw) and len(sent(plain.raw)) == 12
    assert list(explicit.health) == OLD_KEYS


def test_fast_interval(ros, driver):
    r = session(ros, driver, ['_fast_report_interval_us:=10000'])
    assert r.alive
    assert r.requests == [(f, 10000 if f in FAST else 50000) for f in ALL]
    assert list(r.health) == OLD_KEYS + ['fast_report', 'fast_report_interval_us']
    assert (r.health['fast_report'], r.health['fast_report_interval_us']) == ('on', 10000)
    assert repeats(r.keys['imu']) == 0, 'a sample published twice'
    assert 55 < rate(r, 'imu') < 106, '/bno085/imu %.1f Hz' % rate(r, 'imu')   # loaded machine, fake stream
    stamps = r.stamps['imu']
    assert all(b > a for a, b in zip(stamps, stamps[1:]))
    for topic in ('navheading', 'mag'):               # the other topics as before
        assert 80 < rate(r, topic) < 110, '%s %.1f Hz' % (topic, rate(r, topic))
    assert 12 < rate(r, 'mag_uncal') < 24
    assert 'not enabled' not in r.log


def test_sensor_does_not_answer_a_fast_request(ros, driver):
    r = session(ros, driver, ['_fast_report_interval_us:=10000'], lambda s: setattr(s, 'refuse_fast', True))
    assert r.alive and r.health['state'] == 'ok'
    assert r.requests == [(0x01, 10000)] + [(f, 50000) for f in ALL]
    assert (r.health['fast_report'], r.health['fast_report_interval_us']) == ('fallback', 50000)
    assert 80 < rate(r, 'imu') < 110, 'as with the default: 100 Hz messages'
    assert repeats(r.keys['imu']) > 0.6 * len(r.keys['imu'])
    assert 'report 1 at 10000 us not enabled (no answer), back to 50000 us' in r.log


def test_sensor_answers_with_the_default_interval(ros, driver):
    r = session(ros, driver, ['_fast_report_interval_us:=10000'], lambda s: s.grant.update({0x02: 50000}))
    assert r.alive and r.health['state'] == 'ok'
    assert r.requests == [(0x01, 10000), (0x02, 10000), (0x01, 50000), (0x02, 50000)] + [
        (f, 50000) for f in ALL[2:]]
    assert (r.health['fast_report'], r.health['fast_report_interval_us']) == ('fallback', 50000)
    assert 80 < rate(r, 'imu') < 110
    assert 'report 2 at 10000 us not enabled (the sensor answered 50000 us)' in r.log


def test_sensor_answers_with_zero(ros, driver):
    r = session(ros, driver, ['_fast_report_interval_us:=10000'], lambda s: s.grant.update({0x08: 0}), seconds=1.0)
    assert r.alive and r.health['state'] == 'ok'
    assert r.requests[-5:] == [(0x08, 10000), (0x01, 50000), (0x02, 50000), (0x08, 50000), (0x0F, 50000)]
    assert r.health['fast_report'] == 'fallback'


def test_imu_goes_on_when_the_game_rotation_vector_stops(ros, driver):
    """Other reports stream on, 0x08 stops: /bno085/imu must not go silent."""
    def stop_later(s):
        s.stop_0x08_after = 2.0
    for params in ([], ['_fast_report_interval_us:=10000']):
        r = session(ros, driver, params, stop_later, seconds=5.0)
        assert r.alive and r.health['state'] == 'ok'
        tail = [(t, k) for t, k in zip(r.stamps['imu'], r.keys['imu']) if t > r.stamps['imu'][-1] - 1.5]
        assert len({k[1:5] for _t, k in tail}) == 1, 'the game rotation vector did not stop: test setup'
        hz = len(tail) / 1.5
        assert 80 < hz < 110, '%s: %.1f Hz while 0x08 is silent' % (params or 'default', hz)
        gaps = [b[0] - a[0] for a, b in zip(tail, tail[1:])]
        assert max(gaps) < 0.08, 'gap %.3f s' % max(gaps)
        stamps = r.stamps['imu']
        assert max(b - a for a, b in zip(stamps, stamps[1:])) < 0.15, 'silence at the moment 0x08 stopped'
        if params:
            assert repeats(r.keys['imu'][:150]) == 0, 'before the stop: one message per sample'
            assert 'no game rotation vector for' in r.log


@pytest.mark.parametrize('value', ['100', '4999', '50001', '-10000', 'abc', 'true'])
def test_value_out_of_range_is_the_default(ros, driver, value):
    r = session(ros, driver, ['_fast_report_interval_us:=' + value], seconds=0.3)
    assert r.alive
    assert r.requests == [(f, 50000) for f in ALL]
    assert list(r.health) == OLD_KEYS
    assert 'fast_report_interval_us' in r.log and 'using 50000' in r.log


def test_sensor_slower_than_asked_is_reported(ros, driver):
    r = session(ros, driver, ['_fast_report_interval_us:=5000'], lambda s: s.grant.update({0x01: 8000, 0x08: 20000}))
    assert (r.health['fast_report'], r.health['fast_report_interval_us']) == ('on', 20000)
    assert repeats(r.keys['imu']) == 0
    assert 35 < rate(r, 'imu') < 56, 'one message per game rotation vector (50 Hz), %.1f Hz' % rate(r, 'imu')


@pytest.mark.skipif('cpp' not in DRIVERS, reason='BNO085_CPP_NODE is not set to an executable bno085_node_cpp')
@pytest.mark.parametrize('params,setup', [
    (['_fast_report_interval_us:=10000'], None),
    (['_fast_report_interval_us:=10000'], lambda s: setattr(s, 'refuse_fast', True)),
    (['_fast_report_interval_us:=7500'], lambda s: s.grant.update({0x02: 50000})),
], ids=['fast', 'no_answer', 'answers_default'])
def test_both_drivers_write_the_same_bytes(ros, params, setup):
    py = session(ros, 'python', params, setup, seconds=1.2)
    cpp = session(ros, 'cpp', params, setup, seconds=1.2)
    assert sent(py.raw) == sent(cpp.raw), 'python %s\ncpp    %s' % (py.raw.hex(), cpp.raw.hex())
    assert len(sent(py.raw)) >= 12
    strip = ('pid', 'sensor_reset_t', 'packet_age_s')
    assert {k: v for k, v in py.health.items() if k not in strip} == {
        k: v for k, v in cpp.health.items() if k not in strip}
    assert list(py.health) == list(cpp.health)
