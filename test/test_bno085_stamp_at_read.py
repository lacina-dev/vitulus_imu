"""~stamp_at_read of bno085_node: header.stamp is the time the report was read
from the port instead of the time of publishing.

  nice -n 10 python3 -m pytest test/test_bno085_stamp_at_read.py -q

The node against the fake sensor on a pty and a private roscore (helpers in
bno085_session.py).  Python node and, with BNO085_CPP_NODE, the C++ node.
"""
import os
import sys
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bno085_session import ALL, OLD_KEYS, driver, rate, ros, session  # noqa: E402,F401

SAMPLE = {'imu': lambda k: k[1:5], 'navheading': lambda k: k[1:5], 'mag': lambda k: k, 'mag_uncal': lambda k: k}


def stamp_follows_sample(r, topic):
    """Pairs of consecutive messages: (same sample and same stamp, new sample and
    newer stamp, anything else)."""
    part = SAMPLE[topic]
    same = new = other = 0
    for (ka, kb), (sa, sb) in zip(zip(r.keys[topic], r.keys[topic][1:]), zip(r.stamps[topic], r.stamps[topic][1:])):
        if part(ka) == part(kb) and sa == sb:
            same += 1
        elif part(ka) != part(kb) and sb > sa:
            new += 1
        else:
            other += 1
    return same, new, other


def test_default_stamps_every_message_anew(ros, driver):
    r = session(ros, driver)
    for topic in ('imu', 'navheading', 'mag'):
        stamps = r.stamps[topic]
        assert all(b > a for a, b in zip(stamps, stamps[1:])), topic
        assert 80 < rate(r, topic) < 110


@pytest.mark.parametrize('value', ['false', '0', 'no', 'off', '""'])
def test_off_written_out_is_the_default(ros, driver, value):
    r = session(ros, driver, ['_stamp_at_read:=' + value], seconds=1.5)
    assert 'stamp_at_read without' not in r.log
    assert r.requests == [(f, 50000) for f in ALL] and list(r.health) == OLD_KEYS
    stamps = r.stamps['imu']
    assert len(stamps) > 100 and all(b > a for a, b in zip(stamps, stamps[1:]))


@pytest.mark.parametrize('value', ['true', '1', 'yes'])
def test_stamp_is_the_time_of_reading(ros, driver, value):
    t0 = time.time()
    r = session(ros, driver, ['_stamp_at_read:=' + value])
    t1 = time.time()
    assert r.alive
    assert r.requests == [(f, 50000) for f in ALL], 'nothing else changes'
    assert list(r.health) == OLD_KEYS
    for topic in ('imu', 'navheading', 'mag'):
        assert 80 < rate(r, topic) < 110, 'still one message per 100 Hz cycle: %s %.1f Hz' % (topic, rate(r, topic))
        same, new, other = stamp_follows_sample(r, topic)
        assert other == 0, '%s: %d pairs where stamp and sample do not go together' % (topic, other)
        assert 15 * r.seconds < new < 23 * r.seconds, '%s: %d new samples in %.1f s' % (topic, new, r.seconds)
        assert same > 2.5 * new, '%s: repeated messages keep the stamp of their sample' % topic
        assert t0 < r.stamps[topic][0] and r.stamps[topic][-1] < t1
    same, new, other = stamp_follows_sample(r, 'mag_uncal')
    assert same == 0 and other == 0 and new > 30, 'mag_uncal: one message per report, each with its own stamp'
    assert 'stamp_at_read without a fast report interval' in r.log, 'the warning at the start'


def test_with_the_fast_interval(ros, driver):
    r = session(ros, driver, ['_stamp_at_read:=true', '_fast_report_interval_us:=10000'])
    if 'fast_report' not in r.health:
        pytest.skip('this node has no ~fast_report_interval_us')
    stamps = r.stamps['imu']
    assert 'stamp_at_read without' not in r.log
    assert 55 < rate(r, 'imu') < 106
    assert all(b > a for a, b in zip(stamps, stamps[1:])), 'one message per sample: every stamp newer'
    assert stamp_follows_sample(r, 'imu')[2] == 0
    same, new, other = stamp_follows_sample(r, 'mag')
    assert other == 0 and same > 2.5 * new
