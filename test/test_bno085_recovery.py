#!/usr/bin/env python3
"""bno085_node against a fake sensor on a pty and a private roscore.

  python3 test/test_bno085_recovery.py [--port 11871] [name ...]

Every scenario runs the new node and, where it names a limit for it, the node of
git HEAD (`--old-ref`), which has to show the old fault within that limit.
Touches no serial port and no running ROS: all goes to its own master.
"""
import argparse
import importlib.machinery
import importlib.util
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import traceback
import xmlrpc.client

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from fake_bno085 import FakeSensor  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--port', type=int, default=11871)
ap.add_argument('--old-ref', default='HEAD')
ap.add_argument('names', nargs='*')
ARGS = ap.parse_args()
TMP = tempfile.mkdtemp(prefix='bno085_test_')
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % ARGS.port, ROS_HOSTNAME='127.0.0.1',
                  ROS_HOME=TMP, ROS_LOG_DIR=os.path.join(TMP, 'log'))
os.environ.pop('ROS_IP', None)
import rosgraph  # noqa: E402
import rospy  # noqa: E402
from rospy.msg import AnyMsg  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from vitulus_msgs.msg import Imu_status  # noqa: E402

NEW = os.path.join(ROOT, 'nodes', 'bno085_node')
OLD_LIMIT = 10.0             # how long the old node is given to get out of a fault


class Node(object):
    def __init__(self, script, port, params=()):
        self.log = open(os.path.join(TMP, 'node.log'), 'ab')
        args = ['__name:=bno085_node', '_port:=' + port] + list(params)
        self.p = subprocess.Popen([sys.executable, script] + args, stdout=self.log,
                                  stderr=subprocess.STDOUT, start_new_session=True)

    def wait(self, limit):
        try:
            return self.p.wait(limit)
        except subprocess.TimeoutExpired:
            return None

    def alive(self):
        return self.p.poll() is None

    def kill(self):
        if self.alive():
            self.p.send_signal(signal.SIGKILL)
        self.p.wait()
        self.log.close()


class Watch(object):
    def __init__(self):
        self.imu_t = []
        self.status = []
        self.health = None
        rospy.Subscriber('/bno085/imu', AnyMsg, lambda m: self.imu_t.append(time.monotonic()))
        rospy.Subscriber('/bno085/imu_status', Imu_status, lambda m: self.status.append(m.status))
        rospy.Subscriber('/bno085/health', String, self._health)

    def _health(self, msg):
        self.health = json.loads(msg.data)

    def clear(self):
        del self.imu_t[:]
        del self.status[:]
        self.health = None

    def rate(self, seconds):
        n = len(self.imu_t)
        time.sleep(seconds)
        return (len(self.imu_t) - n) / seconds

    def until(self, test, limit):
        end = time.monotonic() + limit
        while time.monotonic() < end:
            if test():
                return True
            time.sleep(0.02)
        return test()


def check(ok, text):
    print('      %s %s' % ('ok ' if ok else 'BAD', text))
    if not ok:
        raise AssertionError(text)


def connected(sensor, watch, limit=8.0):
    return watch.until(lambda: len(watch.imu_t) > 5, limit)


# scenarios: fn(script, sensor, watch, is_old)

def run_healthy(script, sensor, watch, old):
    n = Node(script, sensor.path)
    try:
        check(connected(sensor, watch), 'publishes')
        rate = watch.rate(2.0)
        check(rate > 70, 'rate %.0f Hz' % rate)
        if not old:
            check(watch.until(lambda: watch.status and watch.status[-1], 3.0), 'imu_status true')
            check(watch.health and watch.health['state'] == 'ok', 'health ok')
    finally:
        n.kill()


def run_silent(script, sensor, watch, old):
    """Sensor never answers: the 31. 10. stack, old stays in the read for ever."""
    sensor.mute = True
    n = Node(script, sensor.path)
    try:
        if old:
            code = n.wait(OLD_LIMIT)
            check(code is None, 'old still stuck after %.0f s, nothing published' % OLD_LIMIT)
            check(not watch.imu_t, 'no data')
            return
        code = n.wait(15.0)
        check(code == 1, 'exit %r within 15 s' % code)
        check(watch.status and not any(watch.status), 'imu_status false during startup')
        check(watch.health['failures'] == 1, 'failure counted')
    finally:
        n.kill()


def run_silent_then_back(script, sensor, watch, old):
    """Like a respawn: first run fails, the sensor is back for the second one."""
    sensor.mute = True
    n = Node(script, sensor.path)
    code = n.wait(15.0)
    n.kill()
    check(code == 1, 'first run exit %r' % code)
    sensor.mute = False
    n = Node(script, sensor.path)
    try:
        check(connected(sensor, watch), 'second run publishes')
        check(sensor.resets >= 1, 'sensor reset')
        check(watch.until(lambda: rospy.get_param('/bno085_node/failures', 9) == 0, 13.0), 'failures cleared after stable run')
    finally:
        n.kill()


def run_partial(script, sensor, watch, old):
    """Half a frame at the start, then a sensor that answers only a new request."""
    sensor.partial_once = True
    n = Node(script, sensor.path)
    try:
        if old:
            check(n.wait(OLD_LIMIT) is None and not watch.imu_t, 'old stuck in the broken frame')
            return
        check(connected(sensor, watch), 'publishes')
        check(sensor.adverts >= 2, 'request repeated (%d)' % sensor.adverts)
    finally:
        n.kill()


def run_stream_no_answer(script, sensor, watch, old):
    """Streams on but misses the first requests: a read timeout alone would not help."""
    sensor.pre_stream = True
    sensor.ignore_adverts = 2 if not old else 10**6
    n = Node(script, sensor.path)
    try:
        if old:
            check(n.wait(OLD_LIMIT) is None and not watch.imu_t, 'old spins in soft_reset')
            return
        check(connected(sensor, watch), 'publishes after repeated requests')
    finally:
        n.kill()


def run_stream_never_answers(script, sensor, watch, old):
    sensor.pre_stream = True
    sensor.ignore_adverts = 10**6
    n = Node(script, sensor.path)
    try:
        code = n.wait(15.0)
        check(code == 1, 'exit %r within 15 s' % code)
    finally:
        n.kill()


def run_stop_at_runtime(script, sensor, watch, old):
    n = Node(script, sensor.path)
    try:
        check(connected(sensor, watch), 'publishes')
        sensor.stream_stopped = True
        t0 = time.monotonic()
        if old:
            code = n.wait(6.0)
            rate = watch.rate(1.0)
            check(code is None and rate > 50, 'old goes on with frozen values at %.0f Hz' % rate)
            return
        code = n.wait(5.0)
        took = time.monotonic() - t0
        check(code == 2, 'exit %r after %.1f s' % (code, took))
        last = watch.imu_t[-1] - t0
        check(last < 1.0, 'last message %.2f s after the stop' % last)
        check(watch.health['state'] == 'stalled', 'health stalled: ' + watch.health['reason'])
        check(not watch.status[-1], 'imu_status false')
    finally:
        n.kill()


def run_sigterm_in_init(script, sensor, watch, old):
    sensor.mute = True
    n = Node(script, sensor.path)
    try:
        time.sleep(2.5)
        check(n.alive(), 'in init')
        t0 = time.monotonic()
        n.p.send_signal(signal.SIGTERM)
        code = n.wait(5.0)
        if old:
            check(code is None, 'old ignores SIGTERM for 5 s')
            return
        check(code == 0, 'exit %r in %.2f s' % (code, time.monotonic() - t0))
    finally:
        n.kill()


def run_rosnode_kill_in_init(script, sensor, watch, old):
    sensor.mute = True
    n = Node(script, sensor.path)
    try:
        time.sleep(2.5)
        t0 = time.monotonic()
        master = rosgraph.Master('/test')
        xmlrpc.client.ServerProxy(master.lookupNode('/bno085_node')).shutdown('/test', 'test')
        code = n.wait(5.0)
        if old:
            check(code is None, 'old ignores rosnode kill for 5 s')
            return
        check(code == 0, 'exit %r in %.2f s' % (code, time.monotonic() - t0))
    finally:
        n.kill()


def run_damaged_frame(script, sensor, watch, old):
    """The 13:37 crash: one frame without its end flag."""
    n = Node(script, sensor.path)
    try:
        check(connected(sensor, watch), 'publishes')
        resets = sensor.resets
        sensor.corrupt_one = True
        if old:
            code = n.wait(3.0)
            check(code not in (None, 0), 'old dies with exit %r' % code)
            return
        code = n.wait(3.0)
        check(code is None, 'still running')
        check(sensor.resets == resets, 'sensor not reset, yaw origin kept')
        rate = watch.rate(1.0)
        check(rate > 70, 'rate %.0f Hz' % rate)
        check(watch.until(lambda: watch.health['frame_errors'] >= 1, 3.0), 'frame error counted')
    finally:
        n.kill()


def run_no_port(script, sensor, watch, old):
    n = Node(script, os.path.join(TMP, 'no_such_port'))
    try:
        code = n.wait(5.0)
        check(code == 1, 'exit %r' % code)
    finally:
        n.kill()


def run_backoff(script, sensor, watch, old):
    sensor.mute = True
    rospy.set_param('/bno085_node/failures', 4)
    n = Node(script, sensor.path)
    try:
        check(watch.until(lambda: watch.health and watch.health['state'] == 'backoff', 3.0), 'backoff state')
        check(watch.health['failures'] == 4, 'four failures: ' + watch.health['reason'])
        adverts = sensor.adverts
        check(n.wait(2.0) is None and adverts == 0, 'sensor left alone during the pause')
        code = n.wait(15.0)
        check(code == 1, 'exit %r' % code)
        check(rospy.get_param('/bno085_node/failures') == 5, 'failures 5')
    finally:
        n.kill()


def run_failed_state(script, sensor, watch, old):
    sensor.mute = True
    rospy.set_param('/bno085_node/failures', 7)
    n = Node(script, sensor.path, ['_backoff_max_s:=1.0'])
    try:
        code = n.wait(15.0)
        check(code == 1, 'exit %r' % code)
        check(watch.health['state'] == 'failed', 'health failed after ~max_failures')
    finally:
        n.kill()


SUPERVISOR = r'''
import importlib.machinery, importlib.util, os, sys, time, threading
loader = importlib.machinery.SourceFileLoader('bno', sys.argv[1])
spec = importlib.util.spec_from_loader('bno', loader); mod = importlib.util.module_from_spec(spec); loader.exec_module(mod)
flag = threading.Event()
s = mod.Supervisor(flag.is_set, lambda: None, lambda t: print(t, flush=True), shutdown_grace_s=0.5)
s.beat(1.0); s.start()
if sys.argv[2] == 'shutdown':
    s.beat(100.0); flag.set()
time.sleep(30)
'''


def run_supervisor(script, sensor, watch, old):
    for mode, want, limit in (('stuck', 4, 3.0), ('shutdown', 0, 2.0)):
        t0 = time.monotonic()
        p = subprocess.run([sys.executable, '-c', SUPERVISOR, script, mode], timeout=10, stdout=subprocess.DEVNULL)
        took = time.monotonic() - t0
        check(p.returncode == want and took < limit + 1.5, '%s: exit %d in %.1f s' % (mode, p.returncode, took))


# name, fn, does the old node run it
SCENARIOS = [
    ('healthy', run_healthy, True),
    ('silent', run_silent, True),
    ('silent_then_back', run_silent_then_back, False),
    ('partial_bytes', run_partial, True),
    ('stream_no_answer', run_stream_no_answer, True),
    ('stream_never_answers', run_stream_never_answers, False),
    ('stop_at_runtime', run_stop_at_runtime, True),
    ('sigterm_in_init', run_sigterm_in_init, True),
    ('rosnode_kill_in_init', run_rosnode_kill_in_init, True),
    ('damaged_frame', run_damaged_frame, True),
    ('no_port', run_no_port, False),
    ('backoff', run_backoff, False),
    ('failed_state', run_failed_state, False),
    ('supervisor', run_supervisor, False),
]


def old_script(path):
    src = subprocess.check_output(['git', '-C', ROOT, 'show', ARGS.old_ref + ':nodes/bno085_node']).decode()
    src = src.replace('"/dev/imu", baudrate', 'rospy.get_param("~port"), baudrate')
    with open(path, 'w') as fh:
        fh.write(src)
    return path


def main():
    log = open(os.path.join(TMP, 'roscore.log'), 'wb')
    core = subprocess.Popen(['roscore', '-p', str(ARGS.port)], stdout=log, stderr=log, start_new_session=True)
    results = []
    try:
        end = time.monotonic() + 15
        while not rosgraph.is_master_online():
            if time.monotonic() > end:
                raise RuntimeError('private roscore did not start')
            time.sleep(0.2)
        rospy.init_node('bno085_test', disable_signals=True)
        watch = Watch()
        old = old_script(os.path.join(TMP, 'bno085_node_old'))
        for name, fn, with_old in SCENARIOS:
            if ARGS.names and name not in ARGS.names:
                continue
            for label, script, is_old in (('new', NEW, False), ('old', old, True)):
                if is_old and not with_old:
                    continue
                rospy.set_param('/bno085_node/failures', 0)
                watch.clear()
                sensor = FakeSensor()
                print('[%s] %s' % (label, name), flush=True)
                t0 = time.monotonic()
                try:
                    fn(script, sensor, watch, is_old)
                    results.append((label, name, True, time.monotonic() - t0))
                except Exception as exc:  # noqa: BLE001
                    print('      FAIL', exc)
                    if not isinstance(exc, AssertionError):
                        traceback.print_exc()
                    results.append((label, name, False, time.monotonic() - t0))
                finally:
                    sensor.close()
                time.sleep(0.3)
    finally:
        rospy.signal_shutdown('done')
        os.killpg(core.pid, signal.SIGINT)
        try:
            core.wait(8)
        except subprocess.TimeoutExpired:
            os.killpg(core.pid, signal.SIGKILL)
    print()
    for label, name, ok, took in results:
        print('%-4s %-4s %-24s %5.1f s' % ('PASS' if ok else 'FAIL', label, name, took))
    failed = [r for r in results if not r[2]]
    print('%d of %d passed; logs %s' % (len(results) - len(failed), len(results), TMP))
    return 1 if failed else 0


if __name__ == '__main__':
    code = main()
    sys.stdout.flush()
    os._exit(code)
