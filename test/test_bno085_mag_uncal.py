"""Report 0x0F (magnetic field uncalibrated) in bno085_node.

  nice -n 10 python3 -m pytest test/test_bno085_mag_uncal.py -q

Parsing on bytes, then the node against the fake sensor on a pty and a private
roscore (port 11873): with 0x0F streaming, and with a sensor that refuses it.
"""
import importlib.machinery
import importlib.util
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
PORT = 11873
sys.path.insert(0, HERE)


def load_node():
    loader = importlib.machinery.SourceFileLoader('bno085_node_under_test', NODE)
    spec = importlib.util.spec_from_loader(loader.name, loader)
    mod = importlib.util.module_from_spec(spec)
    loader.exec_module(mod)
    return mod


@pytest.fixture(scope='module')
def node_mod():
    return load_node()


def report_0f(seq, status, xyz, bias):
    return struct.pack('<BBBB6h', 0x0F, seq, status, 0, *(list(xyz) + list(bias)))


def test_parse_mag_uncal(node_mod):
    raw = report_0f(7, 0xFE, (800, -320, 16), (-16, 4000, -32768))
    assert len(raw) == 16
    xyz, bias, acc = node_mod.parse_mag_uncal(raw)
    assert xyz == (50.0, -20.0, 1.0)
    assert bias == (-1.0, 250.0, -2048.0)
    assert acc == 2                                   # bits 1:0 of the status byte
    assert node_mod.parse_mag_uncal(report_0f(0, 0x03, (0, 0, 0), (0, 0, 0)))[2] == 3
    with pytest.raises(ValueError):
        node_mod.parse_mag_uncal(raw[:10])
    with pytest.raises(ValueError):
        node_mod.parse_mag_uncal(b'\x03' + raw[1:])


def test_library_knows_the_length(node_mod):
    import adafruit_bno08x
    assert node_mod.MAG_UNCAL_SUPPORTED
    assert adafruit_bno08x._report_length(0x0F) == 16
    # The calibrated magnetometer stays as it was.
    assert adafruit_bno08x._AVAIL_SENSOR_REPORTS[0x03] == (2 ** -4, 3, 10)


def fake_packet(data):
    return SimpleNamespace(data=bytearray(data), header=SimpleNamespace(data_length=len(data)))


def bare_bno(node_mod):
    bno = object.__new__(node_mod.BoundedBNO)
    bno._packet_slices = []
    bno._readings = {}
    bno._debug = False
    bno._magnetometer_accuracy = 0
    bno.mag_uncal = bno.mag_bias = bno.mag_uncal_accuracy = None
    bno.mag_uncal_count = 0
    bno._uncal_in_packet = False
    return bno


def test_batch_keeps_newest_0f_and_other_reports(node_mod):
    bno = bare_bno(node_mod)
    old = report_0f(1, 1, (160, 0, 0), (0, 0, 0))     # 10 uT, older
    new = report_0f(2, 3, (320, 0, 0), (16, 0, 0))    # 20 uT, newer (later in the batch)
    acc = struct.pack('<BBBB3h', 0x01, 3, 3, 0, 256, 512, 2560)
    mag = struct.pack('<BBBB3h', 0x03, 4, 2, 0, 16, 32, 48)
    bno._handle_packet(fake_packet(bytes([0xFB, 0, 0, 0, 0]) + old + acc + new + mag))
    assert bno.mag_uncal == (20.0, 0.0, 0.0)
    assert bno.mag_bias == (1.0, 0.0, 0.0)
    assert bno.mag_uncal_accuracy == 3
    assert bno.mag_uncal_count == 1
    assert bno._readings[0x01] == (1.0, 2.0, 10.0)
    assert bno._readings[0x03] == (1.0, 2.0, 3.0)
    assert bno._magnetometer_accuracy == 2
    bno._handle_packet(fake_packet(old))
    assert bno.mag_uncal == (10.0, 0.0, 0.0) and bno.mag_uncal_count == 2


# -- the node against the fake sensor -------------------------------------------

@pytest.fixture(scope='module')
def ros():
    tmp = tempfile.mkdtemp(prefix='bno085_uncal_test_')
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
    rospy.init_node('bno085_uncal_test', disable_signals=True)
    yield SimpleNamespace(tmp=tmp, env=env)
    rospy.signal_shutdown('done')
    os.killpg(core.pid, signal.SIGINT)
    try:
        core.wait(8)
    except subprocess.TimeoutExpired:
        os.killpg(core.pid, signal.SIGKILL)
    log.close()
    shutil.rmtree(tmp, ignore_errors=True)


class Watch(object):
    def __init__(self):
        import rospy
        from rospy.msg import AnyMsg
        from sensor_msgs.msg import MagneticField
        from std_msgs.msg import String
        from rosgraph_msgs.msg import Log
        self.imu_t, self.uncal, self.health, self.logs = [], [], None, []
        self.subs = [rospy.Subscriber('/bno085/imu', AnyMsg, lambda m: self.imu_t.append(time.monotonic())),
                     rospy.Subscriber('/bno085/mag_uncal', MagneticField, self.uncal.append),
                     rospy.Subscriber('/bno085/health', String, self._health),
                     rospy.Subscriber('/rosout_agg', Log, self._log)]

    def _log(self, msg):
        if msg.name == '/bno085_node':
            self.logs.append(msg.msg)

    def _health(self, msg):
        self.health = json.loads(msg.data)

    def close(self):
        for s in self.subs:
            s.unregister()

    def until(self, test, limit):
        end = time.monotonic() + limit
        while time.monotonic() < end:
            if test():
                return True
            time.sleep(0.05)
        return test()


def run_node(ros, sensor, seconds_check):
    from fake_bno085 import FakeSensor  # noqa: F401 - imported for the type only
    import rospy
    rospy.set_param('/bno085_node/failures', 0)
    logpath = os.path.join(ros.tmp, 'node_%d.log' % time.monotonic_ns())
    log = open(logpath, 'wb')
    p = subprocess.Popen([sys.executable, NODE, '__name:=bno085_node', '_port:=' + sensor.path],
                         stdout=log, stderr=subprocess.STDOUT, env=ros.env, start_new_session=True)
    watch = Watch()
    try:
        seconds_check(p, watch)
        time.sleep(0.5)
    finally:
        watch.close()
        if p.poll() is None:
            p.send_signal(signal.SIGKILL)
        p.wait()
        log.close()
    with open(logpath, 'rb') as fh:
        return fh.read().decode(errors='replace') + '\n'.join(watch.logs)


def test_node_publishes_mag_uncal(ros):
    from fake_bno085 import FakeSensor
    sensor = FakeSensor()

    def check(p, w):
        assert w.until(lambda: len(w.uncal) > 10, 15.0), 'no /bno085/mag_uncal'
        n0, t0 = len(w.imu_t), time.monotonic()
        u0 = len(w.uncal)
        time.sleep(2.0)
        assert (len(w.imu_t) - n0) / (time.monotonic() - t0) > 70, 'imu rate'
        rate = (len(w.uncal) - u0) / 2.0
        assert 10 < rate < 30, 'mag_uncal %.1f Hz (one per report, ~20 Hz)' % rate
        m = w.uncal[-1]
        assert m.header.frame_id == 'bno_imu_link'
        assert abs(m.magnetic_field.y - 200 / 16.0 * 1e-6) < 1e-12
        assert abs(m.magnetic_field.z - 1000 / 16.0 * 1e-6) < 1e-12
        assert 100 / 16.0 * 1e-6 <= m.magnetic_field.x <= 150 / 16.0 * 1e-6
        assert w.until(lambda: w.health and w.health.get('mag_uncal') == 'on', 3.0), w.health
        h = w.health
        assert h['state'] == 'ok' and h['mag_uncal_accuracy'] == 3 and h['mag_accuracy'] == 3
        assert h['mag_bias_uT'] == [312.5, 0.19, 0.25]
        assert p.poll() is None

    try:
        out = run_node(ros, sensor, check)
    finally:
        sensor.close()
    assert '0x0F' not in out, out


def test_node_runs_on_without_mag_uncal(ros):
    from fake_bno085 import FakeSensor
    sensor = FakeSensor()
    sensor.refuse = {0x0F}

    def check(p, w):
        assert w.until(lambda: len(w.imu_t) > 5, 20.0), 'no /bno085/imu'
        assert w.until(lambda: w.health and w.health.get('mag_uncal') == 'unavailable', 5.0), w.health
        n0, t0 = len(w.imu_t), time.monotonic()
        time.sleep(5.0)
        assert (len(w.imu_t) - n0) / (time.monotonic() - t0) > 70, 'imu rate'
        assert p.poll() is None, 'node exited'
        assert w.health['state'] == 'ok' and w.health['failures'] == 0
        assert not w.uncal

    try:
        out = run_node(ros, sensor, check)
    finally:
        sensor.close()
    # One connection: soft_reset sends the reset command twice, a reconnect would add more.
    assert sensor.resets == 2, 'sensor reset %d times' % sensor.resets
    assert out.count('0x0F not enabled') == 1, out


def test_health_no_data_when_0f_stops(ros):
    from fake_bno085 import FakeSensor
    sensor = FakeSensor()

    def check(p, w):
        assert w.until(lambda: w.health and w.health.get('mag_uncal') == 'on', 15.0), w.health
        sensor.silent_reports.add(0x0F)
        assert w.until(lambda: w.health.get('mag_uncal') == 'no data', 5.0), w.health
        n0, t0 = len(w.imu_t), time.monotonic()
        time.sleep(1.0)
        assert (len(w.imu_t) - n0) / (time.monotonic() - t0) > 70, 'imu rate'
        assert w.health['state'] == 'ok' and p.poll() is None
        sensor.silent_reports.clear()
        assert w.until(lambda: w.health.get('mag_uncal') == 'on', 5.0), w.health

    try:
        run_node(ros, sensor, check)
    finally:
        sensor.close()
