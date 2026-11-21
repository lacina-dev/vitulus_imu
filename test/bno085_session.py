"""The node on the fake sensor and a private roscore (port 11877), for the tests
of single parameters of bno085_node: the `ros` and `driver` fixtures, session().
Drivers: the Python node and, with BNO085_CPP_NODE=/path/to/bno085_node_cpp
(built outside the live workspace), the C++ node.
"""
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
NODE = os.path.join(ROOT, 'nodes', 'bno085_node')
CPP = os.environ.get('BNO085_CPP_NODE', '')
PORT = 11877
sys.path.insert(0, HERE)
from fake_bno085 import FakeSensor  # noqa: E402
from test_bno085_equivalence import Watch  # noqa: E402

DRIVERS = {'python': [sys.executable, NODE]}
if CPP and os.access(CPP, os.X_OK):
    DRIVERS['cpp'] = [CPP]
ALL = [0x01, 0x02, 0x03, 0x05, 0x04, 0x09, 0x08, 0x0F]     # the order the node enables them in
FAST = (0x01, 0x02, 0x08)
OLD_KEYS = ['state', 'reason', 'ok', 'failures', 'attempt', 'packet_age_s', 'frame_errors', 'sensor_reset_t',
            'pid', 'mag_accuracy', 'mag_uncal', 'mag_uncal_accuracy', 'mag_bias_uT']


class Recording(FakeSensor):
    """FakeSensor that keeps every byte the node writes."""

    def __init__(self):
        self.raw = bytearray()
        super().__init__()

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


@pytest.fixture(scope='module')
def ros():
    tmp = tempfile.mkdtemp(prefix='bno085_fast_test_')
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
    rospy.init_node('bno085_fast_test', disable_signals=True)
    yield SimpleNamespace(tmp=tmp, env=env)
    rospy.signal_shutdown('done')
    os.killpg(core.pid, signal.SIGINT)
    try:
        core.wait(8)
    except subprocess.TimeoutExpired:
        os.killpg(core.pid, signal.SIGKILL)
    log.close()
    shutil.rmtree(tmp, ignore_errors=True)


def session(ros, driver, params=(), setup=None, seconds=3.0):
    """The node on a fresh fake sensor: start, `seconds` of stream, stop."""
    import rospy
    rospy.set_param('/bno085_node/failures', 0)
    for name in ('fast_report_interval_us', 'stamp_at_read'):
        if rospy.has_param('/bno085_node/' + name):
            rospy.delete_param('/bno085_node/' + name)
    sensor = Recording()
    if setup:
        setup(sensor)
    log_path = os.path.join(ros.tmp, 'node_%s_%d.log' % (driver, int(time.time() * 1000)))
    log = open(log_path, 'ab')
    watch = Watch()
    p = subprocess.Popen(DRIVERS[driver] + ['__name:=bno085_node', '_port:=' + sensor.path] + list(params),
                         stdout=log, stderr=subprocess.STDOUT, env=ros.env, start_new_session=True)
    out = SimpleNamespace(watch=watch, sensor=sensor)
    try:
        assert watch.until(lambda: watch.health and watch.health[-1]['state'] == 'ok', 25.0), watch.health[-1:]
        assert watch.until(lambda: all(s.get_num_connections() for s in watch.subs), 5.0)
        time.sleep(0.5)
        stop_after = getattr(sensor, 'stop_0x08_after', None)      # the game rotation vector stops mid-run
        if stop_after is not None:
            stopper = threading.Timer(stop_after, sensor.silent_reports.add, [0x08])
            stopper.daemon = True
            stopper.start()
        first = {t: len(watch.keys[t]) for t in Watch.TOPICS}
        t0 = time.monotonic()
        time.sleep(seconds)
        out.seconds = time.monotonic() - t0
        out.keys = {t: watch.keys[t][first[t]:] for t in Watch.TOPICS}
        out.stamps = {t: watch.stamps[t][first[t]:] for t in Watch.TOPICS}
        out.health = watch.health[-1]
        out.alive = p.poll() is None
    finally:
        watch.close()
        if p.poll() is None:
            p.send_signal(signal.SIGKILL)
        p.wait()
        log.close()
        out.requests = list(sensor.requests)
        out.raw = bytes(sensor.raw)
        sensor.close()
        time.sleep(0.5)
    with open(log_path, 'rb') as f:
        out.log = f.read().decode('utf-8', 'replace')
    for d in (os.path.join(ros.tmp, 'log'),):          # rospy / roscpp write warnings to the log directory
        for base, _dirs, files in os.walk(d):
            for name in files:
                if name.startswith('bno085_node'):
                    with open(os.path.join(base, name), 'rb') as f:
                        out.log += f.read().decode('utf-8', 'replace')
                    os.remove(os.path.join(base, name))
    return out


def sent(raw):
    """What the node wrote, frame by frame: (channel, payload).  Without the
    sequence byte: the node takes it over from the packets it reads on that
    channel, so with a streaming sensor it depends on the timing."""
    out, i = [], 0
    while i + 6 <= len(raw):
        assert raw[i:i + 2] == b'\x7e\x01', 'frame start at %d: %s' % (i, raw[i:i + 8].hex())
        size = raw[i + 2] | (raw[i + 3] << 8)
        assert raw[i + 2 + size:i + 3 + size] == b'\x7e', 'frame end at %d' % i
        out.append((raw[i + 4], bytes(raw[i + 6:i + 2 + size])))
        i += 3 + size
    assert i == len(raw), 'bytes after the last frame'
    return out


def rate(r, topic):
    return len(r.keys[topic]) / r.seconds


def repeats(keys):
    return sum(1 for a, b in zip(keys, keys[1:]) if a == b)


@pytest.fixture(scope='module', params=sorted(DRIVERS))
def driver(request):
    return request.param
