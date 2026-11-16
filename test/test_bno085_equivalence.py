"""bno085_node (Python) and bno085_node_cpp side by side: the same bytes from the
sensor have to give the same messages, the same bytes back, the same health.

  BNO085_CPP_NODE=/path/to/bno085_node_cpp nice -n 15 python3 -m pytest -q test/test_bno085_equivalence.py

Without BNO085_CPP_NODE (the binary, built outside the live workspace) all is
skipped.  A scripted fake sensor on a pty and a private roscore (port 11875);
no serial port, no running ROS is touched.

Both nodes publish at 100 Hz whatever the sensor sends, so the number of
messages depends on the clock.  Compared exactly is therefore
  * the sequence of different consecutive messages per topic (all fields but
    stamp and seq, floats bit for bit) - the sensor waits after each step until
    both nodes have published it many times;
  * /bno085/mag_uncal message by message (one per packet with report 0x0F);
  * every byte the node writes to the sensor (reset handshake, feature
    requests, their sequence numbers);
  * the health JSON (but pid, times) and the exit code.
header.seq differs by one: rospy counts a topic from 1, roscpp from 0.
The number of 100 Hz messages only has to be near the nominal one.
"""
import json
import os
import shutil
import signal
import struct
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
NODE = os.path.join(ROOT, 'nodes', 'bno085_node')
CPP = os.environ.get('BNO085_CPP_NODE', '')
PORT = 11875
STEP_S = 0.25
sys.path.insert(0, HERE)
from fake_bno085 import FakeSensor, frame  # noqa: E402

pytestmark = pytest.mark.skipif(not (CPP and os.access(CPP, os.X_OK)),
                                reason='BNO085_CPP_NODE is not set to an executable bno085_node_cpp')
DRIVERS = {'python': [sys.executable, NODE], 'cpp': [CPP]}


class ScriptedSensor(FakeSensor):
    """Answers as FakeSensor, streams nothing by itself, keeps every byte it gets."""

    def __init__(self):
        self.raw = bytearray()
        super().__init__()

    def _streamer(self):
        return

    def _reader(self):
        while self.running:
            try:
                chunk = os.read(self.master, 512)
            except BlockingIOError:
                time.sleep(0.0005)
                continue
            except OSError:
                time.sleep(0.01)
                continue
            with self.lock:
                self.raw += chunk
                self.rx += chunk
                self._parse()

    def step(self, *frames):
        """One write: the node finds all of it at once."""
        os.write(self.master, b''.join(frames))
        time.sleep(STEP_S)


def rep(rid, seq, status, *words):
    return struct.pack('<BBBB%dh' % len(words), rid, seq, status, 0, *words)


def batch(seq, *reports, **kw):
    return frame(kw.get('channel', 3), seq, bytes([0xFB, 1, 2, 3, 4]) + b''.join(reports), **{
        k: v for k, v in kw.items() if k == 'end'})


def script(s):
    """The stream after the start: plain reports, then every corner the parser has."""
    def all_reports(n, k):
        return [rep(0x01, n, 3, 100 + k, -200 - k, 2500 + k), rep(0x02, n, 2, 5 + k, -6 - k, 7 + k),
                rep(0x03, n, 1, 320 + k, -160 - k, 480 + k), rep(0x04, n, 0, 11 + k, 12 + k, -13 - k),
                rep(0x05, n, 3, 100 + k, -200, 300, 16000 + k, 55), rep(0x09, n, 3, 10 + k, 20, -30, 4000 + k, 9),
                rep(0x08, n, 3, -100 - k, 200, -300, 16100 + k), rep(0x0F, n, k % 4, 800 + k, -320, 16, -16, 4000 + k, 7)]
    # 1 one batch with every report
    s.step(batch(1, *all_reports(1, 0)))
    # 2 one report per packet; values with 0x7e and 0x7d in them (escaped on the line)
    s.step(*[batch(10 + i, r) for i, r in enumerate(all_reports(2, 1))] +
           [batch(0x7e, rep(0x01, 0x7d, 3, 0x7e7d, 0x7d7e, 0x7e7e), rep(0x02, 3, 3, 0x7d7d, 126, 125))])
    # 3 the same report twice in one batch (the library keeps the first, 0x0F the last)
    s.step(batch(20, rep(0x01, 1, 3, 1, 2, 3), rep(0x0F, 1, 1, 160, 0, 0, 1, 2, 3), rep(0x01, 2, 3, 4, 5, 6),
                 rep(0x0F, 2, 2, 320, 16, -16, 4, 5, 6), rep(0x08, 1, 3, 1, 2, 3, 16000), rep(0x08, 2, 3, 4, 5, 6, 16001)))
    # 4 a frame without its end flag, then good ones
    s.step(batch(21, rep(0x02, 1, 3, 70, 80, 90), end=b'\x55'), batch(22, rep(0x03, 1, 2, 16, 32, 48)),
           batch(23, rep(0x0F, 3, 3, 16, 32, 48, 0, 0, 0)))
    # 5 a report the library does not know, in a batch with a known one
    s.step(batch(24, rep(0x01, 1, 3, 7, 7, 7), bytes([0x2A]) + bytes(13)), batch(25, rep(0x04, 1, 3, 9, 9, 9)))
    s.step(batch(26, rep(0x01, 5, 3, 8, 8, 8)))
    # 6 a feature response zeroes the reading (game quaternion: nothing on /bno085/imu until the next one)
    s.step(frame(2, 27, bytes([0xFC, 0x08]) + bytes(15)), batch(28, rep(0x02, 1, 3, 1, 1, 1)))
    s.step(batch(29, rep(0x08, 1, 3, 10, 20, 30, 16200)))
    # 7 feature responses for the others: zeros are published
    s.step(frame(2, 30, bytes([0xFC, 0x03]) + bytes(15)), frame(2, 31, bytes([0xFC, 0x05]) + bytes(15)),
           batch(32, rep(0x01, 1, 3, 2, 2, 2)))
    s.step(batch(33, rep(0x03, 1, 3, 1600, 16, 16), rep(0x05, 1, 3, 1, 2, 3, 16300, 0)))
    # 8 a quaternion whose parts add up to zero is not published
    s.step(batch(34, rep(0x05, 1, 3, 500, -500, 0, 0, 0), rep(0x08, 1, 3, 500, 0, -500, 0), rep(0x02, 1, 3, 3, 3, 3)))
    s.step(batch(35, rep(0x05, 1, 3, 1, 2, 3, 16301, 0), rep(0x08, 1, 3, 1, 2, 3, 16302)))
    # 9 a packet over 512 bytes (the library reads and drops it), then a good one
    s.step(batch(36, *[rep(0x01, i, 3, 900 + i, 0, 0) for i in range(60)]), batch(37, rep(0x04, 1, 3, 1, 2, 3)))
    s.step(batch(38, rep(0x02, 1, 3, 4, 4, 4)))
    # 10 command responses: a failed DCD save raises in the library, others pass
    s.step(frame(2, 39, bytes([0xF1, 0, 0x06, 0, 0, 1]) + bytes(10)), batch(40, rep(0x01, 1, 3, 5, 5, 5)))
    s.step(frame(2, 41, bytes([0xF1, 0, 0x07, 0, 0, 0]) + bytes(10)), frame(2, 42, bytes([0xF1, 0, 0x06, 0, 0, 0]) + bytes(10)),
           batch(43, rep(0x01, 1, 3, 6, 6, 6)))
    # 11 a batch cut short, a channel that does not exist, an empty packet, a timestamp rebase
    s.step(frame(3, 44, bytes([0xFB, 0, 0, 0, 0]) + rep(0x01, 1, 3, 1, 1, 1)[:7]), batch(45, rep(0x02, 1, 3, 5, 5, 5)))
    s.step(frame(7, 46, bytes([1, 2, 3])), batch(47, rep(0x02, 1, 3, 6, 6, 6)))
    s.step(b'\x7e\x01\x00\x00\x03\x30\x7e', batch(48, rep(0x02, 1, 3, 7, 7, 7)))
    s.step(frame(3, 49, bytes([0xFA, 9, 9, 9, 9]) + rep(0x03, 1, 0, -16, -32, -48)))
    # 12 classifier and raw reports (not used, but they have to be stepped over as the library does)
    s.step(batch(50, rep(0x13, 1, 3, 9), rep(0x01, 1, 3, 10, 10, 10)), batch(51, rep(0x14, 1, 3, -1, 2, 3, 0, 0, 0),
           rep(0x13, 1, 3, 2), bytes([0x1E, 0, 0, 0, 0, 4]) + bytes(10), bytes([0x11]) + bytes(11), bytes([0x19]) + bytes(5),
           rep(0x02, 1, 3, 8, 8, 8)))
    s.step(batch(52, bytes([0x1E, 0, 0, 0, 0, 9]) + bytes(10), rep(0x02, 1, 3, 9, 9, 9)), batch(53, rep(0x04, 1, 3, 4, 4, 4)))
    # 13 the end: every report once more
    s.step(batch(54, *all_reports(9, 40)))
    s.step(batch(55, rep(0x0F, 10, 3, 1, 2, 3, 4, 5, 6)))


@pytest.fixture(scope='module')
def ros():
    tmp = tempfile.mkdtemp(prefix='bno085_equiv_test_')
    env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d' % PORT, ROS_HOSTNAME='127.0.0.1',
               ROS_HOME=tmp, ROS_LOG_DIR=os.path.join(tmp, 'log'))
    env.pop('ROS_IP', None)
    os.environ.update({k: env[k] for k in ('ROS_MASTER_URI', 'ROS_HOSTNAME', 'ROS_HOME', 'ROS_LOG_DIR')})
    os.environ.pop('ROS_IP', None)
    log = open(os.path.join(tmp, 'roscore.log'), 'wb')
    core = subprocess.Popen(['roscore', '-p', str(PORT)], stdout=log, stderr=log, env=env,
                            start_new_session=True)
    import rosgraph
    import rospy
    end = time.monotonic() + 20
    while not rosgraph.is_master_online():
        assert time.monotonic() < end, 'private roscore did not start'
        time.sleep(0.2)
    rospy.init_node('bno085_equiv_test', disable_signals=True)
    yield SimpleNamespace(tmp=tmp, env=env)
    rospy.signal_shutdown('done')
    os.killpg(core.pid, signal.SIGINT)
    try:
        core.wait(8)
    except subprocess.TimeoutExpired:
        os.killpg(core.pid, signal.SIGKILL)
    log.close()
    shutil.rmtree(tmp, ignore_errors=True)


def imu_key(m):
    return (m.header.frame_id, m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w,
            tuple(m.orientation_covariance), m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
            tuple(m.angular_velocity_covariance), m.linear_acceleration.x, m.linear_acceleration.y,
            m.linear_acceleration.z, tuple(m.linear_acceleration_covariance))


def mag_key(m):
    return (m.header.frame_id, m.magnetic_field.x, m.magnetic_field.y, m.magnetic_field.z,
            tuple(m.magnetic_field_covariance))


class Watch(object):
    TOPICS = ('imu', 'navheading', 'mag', 'mag_uncal')

    def __init__(self):
        import rospy
        from sensor_msgs.msg import Imu, MagneticField
        from std_msgs.msg import String
        from vitulus_msgs.msg import Imu_status
        self.keys = {t: [] for t in self.TOPICS}       # every message, without stamp and seq
        self.seqs = {t: [] for t in self.TOPICS + ('imu_status',)}
        self.stamps = {t: [] for t in self.TOPICS}
        self.status, self.health = [], []
        types = {'imu': (Imu, imu_key), 'navheading': (Imu, imu_key), 'mag': (MagneticField, mag_key),
                 'mag_uncal': (MagneticField, mag_key)}
        self.subs = [rospy.Subscriber('/bno085/' + t, types[t][0], self._data, (t, types[t][1]), queue_size=1000)
                     for t in self.TOPICS]
        self.subs += [rospy.Subscriber('/bno085/imu_status', Imu_status, self._status, queue_size=100),
                      rospy.Subscriber('/bno085/health', String, lambda m: self.health.append(json.loads(m.data)),
                                       queue_size=100)]

    def _data(self, msg, arg):
        topic, key = arg
        self.keys[topic].append(key(msg))
        self.seqs[topic].append(msg.header.seq)
        self.stamps[topic].append(msg.header.stamp.to_sec())

    def _status(self, msg):
        self.status.append((msg.status, msg.header.frame_id))
        self.seqs['imu_status'].append(msg.header.seq)

    def changes(self, topic):
        out = []
        for key in self.keys[topic]:
            if not out or out[-1] != key:
                out.append(key)
        return out

    def close(self):
        for s in self.subs:
            s.unregister()

    def until(self, test, limit):
        end = time.monotonic() + limit
        while time.monotonic() < end:
            if test():
                return True
            time.sleep(0.02)
        return test()


def plain(health):
    return {k: v for k, v in health.items() if k not in ('pid', 'sensor_reset_t', 'packet_age_s')}


def history(healths):
    """The different health states in a row.  Without the short time between the
    reset handshake and `ok` (features being enabled): the 1 Hz status may or may
    not fall into it."""
    out = []
    for h in healths:
        if h['state'] == 'connecting' and h['mag_accuracy'] is not None and not h['reason']:
            continue
        if not out or out[-1] != h:
            out.append(h)
    return out


def run(ros, driver, sensor, body, params=()):
    import rospy
    rospy.set_param('/bno085_node/failures', 0)
    log = open(os.path.join(ros.tmp, 'node_%s.log' % driver), 'ab')
    watch = Watch()
    p = subprocess.Popen(DRIVERS[driver] + ['__name:=bno085_node'] + list(params), stdout=log,
                         stderr=subprocess.STDOUT, env=ros.env, start_new_session=True)
    try:
        body(p, watch)
        time.sleep(0.3)
    finally:
        watch.close()
        if p.poll() is None:
            p.send_signal(signal.SIGKILL)
        p.wait()
        log.close()
        time.sleep(0.5)
    return watch


def streamed(ros, driver):
    sensor = ScriptedSensor()
    out = SimpleNamespace()

    def body(p, w):
        assert w.until(lambda: w.health and w.health[-1]['state'] == 'ok', 20.0), w.health[-1:]
        assert w.until(lambda: all(s.get_num_connections() for s in w.subs), 5.0), 'subscribers not connected'
        out.start_raw = bytes(sensor.raw)
        out.start_health = [plain(h) for h in w.health]
        t0 = time.monotonic()
        script(sensor)
        assert p.poll() is None, 'node exited during the stream'
        for i in range(6):           # a status tick with everything in it, the stream still alive
            sensor.step(batch(60 + i, rep(0x04, i, 3, 1, 1, 1)))
        out.seconds = time.monotonic() - t0
        out.health = plain(w.health[-1])
        out.full_health = w.health[-1]

    try:
        out.watch = run(ros, driver, sensor, body, ['_port:=' + sensor.path])
        out.raw = bytes(sensor.raw)
    finally:
        sensor.close()
    return out


@pytest.fixture(scope='module')
def both(ros):
    return {driver: streamed(ros, driver) for driver in ('python', 'cpp')}


def test_bytes_to_the_sensor_are_the_same(both):
    py, cpp = both['python'], both['cpp']
    assert len(py.raw) > 150, 'handshake and nine feature requests expected, got %d bytes' % len(py.raw)
    assert py.raw == cpp.raw, 'python %s\ncpp    %s' % (py.raw.hex(), cpp.raw.hex())
    assert py.raw == py.start_raw and cpp.raw == cpp.start_raw, 'bytes written after the start'


@pytest.mark.parametrize('topic', Watch.TOPICS)
def test_same_messages(both, topic):
    py, cpp = both['python'].watch, both['cpp'].watch
    a, b = py.changes(topic), cpp.changes(topic)
    least = {'imu': 14, 'navheading': 12, 'mag': 6, 'mag_uncal': 6}[topic]
    assert len(a) >= least, 'the script should give at least %d different %s messages, python gave %d' % (
        least, topic, len(a))
    assert len(a) == len(b), '%s: %d different messages from python, %d from cpp' % (topic, len(a), len(b))
    for i, (x, y) in enumerate(zip(a, b)):
        assert x == y, '%s: change %d differs\npython %r\ncpp    %r' % (topic, i, x, y)


def test_mag_uncal_message_by_message(both):
    py, cpp = both['python'].watch, both['cpp'].watch
    assert py.keys['mag_uncal'] == cpp.keys['mag_uncal']
    assert len(py.keys['mag_uncal']) == 6        # one per step with report 0x0F, not one per report or cycle


def test_number_of_messages(both):
    for driver in ('python', 'cpp'):
        r = both[driver]
        for topic in ('imu', 'navheading', 'mag'):
            n = len(r.watch.keys[topic])
            assert 0.6 * 100 * r.seconds < n < 1.1 * 100 * r.seconds + 100, '%s %s: %d messages in %.1f s' % (
                driver, topic, n, r.seconds)
    # /bno085/imu is silent while the game quaternion is zero (steps 6 and 8), /bno085/mag never is
    for driver in ('python', 'cpp'):
        w = both[driver].watch
        assert len(w.keys['mag']) > len(w.keys['imu'])


def test_seq_and_stamps(both):
    for driver in ('python', 'cpp'):
        w = both[driver].watch
        for topic in Watch.TOPICS:
            # header.seq counts the messages of the topic: rospy from 1, roscpp from 0
            # (roscpp writes it itself when it sends, a node cannot set it)
            seqs, first = w.seqs[topic], 1 if driver == 'python' else 0
            assert seqs == list(range(first, first + len(seqs))), '%s %s seq %r' % (driver, topic, seqs[:5])
            stamps = w.stamps[topic]
            assert all(b >= a for a, b in zip(stamps, stamps[1:])), '%s %s stamps go back' % (driver, topic)
            assert abs(stamps[-1] - time.time()) < 60, '%s %s stamp is not the time of publishing' % (driver, topic)
        s = w.seqs['imu_status']
        assert s == list(range(s[0], s[0] + len(s))), '%s imu_status seq' % driver


def test_health_and_status(both):
    py, cpp = both['python'], both['cpp']
    assert list(py.full_health) == list(cpp.full_health), 'keys and their order'
    assert py.health == cpp.health
    assert py.health['frame_errors'] >= 6 and py.health['mag_uncal'] == 'on'
    assert py.health['mag_bias_uT'] == [0.25, 0.31, 0.38]
    assert history(py.start_health) == history(cpp.start_health), 'health during the start'
    for r in (py, cpp):
        assert r.watch.status[-1] == (True, '')
        assert isinstance(r.full_health['sensor_reset_t'], float) and isinstance(r.full_health['packet_age_s'], float)
        assert abs(r.full_health['sensor_reset_t'] - time.time()) < 120


FAULTS = [
    ('silent', lambda s: setattr(s, 'mute', True), ['_advert_timeout_s:=1.0'], 1),
    ('no_port', None, ['_connect_attempts:=1'], 1),
    ('refuses_0f', lambda s: s.refuse.add(0x0F), ['_stall_timeout_s:=30.0'], None),
    ('refuses_gyro', lambda s: s.refuse.add(0x02), ['_connect_attempts:=1'], 1),
    ('no_mag_uncal', None, ['_mag_uncal:=false', '_stall_timeout_s:=30.0'], None),
]


@pytest.mark.parametrize('name,setup,params,code', FAULTS, ids=[f[0] for f in FAULTS])
def test_same_health_in_faults(ros, name, setup, params, code):
    """State, reason, counters and the exit code of both nodes in a fault."""
    result = {}
    for driver in ('python', 'cpp'):
        sensor = ScriptedSensor()
        port = sensor.path if name != 'no_port' else '/tmp/bno085_equiv_no_such_port'
        if setup:
            setup(sensor)
        got = SimpleNamespace()

        def body(p, w):
            if code is None:
                assert w.until(lambda: w.health and w.health[-1]['state'] == 'ok', 25.0), w.health[-1:]
                time.sleep(1.2)
                got.code = p.poll()
            else:
                try:
                    got.code = p.wait(25.0)
                except subprocess.TimeoutExpired:
                    got.code = 'still running'
                time.sleep(0.3)
            got.health = [plain(h) for h in w.health]

        try:
            run(ros, driver, sensor, body, ['_port:=' + port] + params)
            got.raw = bytes(sensor.raw)
        finally:
            sensor.close()
        result[driver] = got
    py, cpp = result['python'], result['cpp']
    assert py.code == code and cpp.code == code, 'exit python %r, cpp %r' % (py.code, cpp.code)
    assert py.health[-1] == cpp.health[-1], 'last health'
    assert history(py.health) == history(cpp.health), 'health history'
    assert py.raw == cpp.raw, 'bytes to the sensor\npython %s\ncpp    %s' % (py.raw.hex(), cpp.raw.hex())
