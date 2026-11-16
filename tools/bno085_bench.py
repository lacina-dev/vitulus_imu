#!/usr/bin/env python3
"""CPU and period of the two BNO085 drivers on the fake sensor.

  nice -n 15 python3 tools/bno085_bench.py --cpp /path/to/bno085_node_cpp [--seconds 60] [--rates 20,100]

Starts a private roscore (port 11877) and, one after the other, the Python node
and the C++ node on a pty with test/fake_bno085.py streaming every report
`rate` times a second (the node asks for 20 Hz; 100 Hz shows the load of a
faster stream).  CPU is pidstat over the run, the period comes from the stamps
of /bno085/imu (the time of publishing in the node).  No serial port and no
running ROS is touched.  The fake sensor is not the real one: an indication.
"""
import argparse
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'test'))
from fake_bno085 import CH_REPORTS, REPORT_LEN, FakeSensor, frame  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--cpp', default=os.environ.get('BNO085_CPP_NODE', ''))
ap.add_argument('--seconds', type=int, default=60)
ap.add_argument('--rates', default='20,100')
ap.add_argument('--port', type=int, default=11877)
ARGS = ap.parse_args()
TMP = tempfile.mkdtemp(prefix='bno085_bench_')
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % ARGS.port, ROS_HOSTNAME='127.0.0.1',
                  ROS_HOME=TMP, ROS_LOG_DIR=os.path.join(TMP, 'log'))
os.environ.pop('ROS_IP', None)
import rosgraph  # noqa: E402
import rospy  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402


class RateSensor(FakeSensor):
    """FakeSensor with the stream period as a parameter."""

    def __init__(self, rate):
        self.period = 1.0 / rate
        super().__init__()

    def _streamer(self):
        n = 0
        next_t = time.monotonic()
        while self.running:
            next_t += self.period
            time.sleep(max(0.0, next_t - time.monotonic()))
            n += 1
            ids = [i for i in sorted(self.features) if i in REPORT_LEN]
            for rid in ids:
                self.seq += 1
                self._send(frame(CH_REPORTS, self.seq, bytes([0xfb, 0, 0, 0, 0]) + self._report(rid, n)))


def measure(label, cmd, rate):
    sensor = RateSensor(rate)
    stamps = []
    sub = rospy.Subscriber('/bno085/imu', Imu, lambda m: stamps.append(m.header.stamp.to_sec()), queue_size=1000)
    rospy.set_param('/bno085_node/failures', 0)
    log = open(os.path.join(TMP, 'node.log'), 'ab')
    p = subprocess.Popen(cmd + ['__name:=bno085_node', '_port:=' + sensor.path], stdout=log,
                         stderr=subprocess.STDOUT, start_new_session=True)
    try:
        end = time.monotonic() + 30
        while len(stamps) < 300:
            if time.monotonic() > end or p.poll() is not None:
                raise RuntimeError('%s does not publish' % label)
            time.sleep(0.1)
        del stamps[:]
        out = subprocess.check_output(['pidstat', '-u', '-p', str(p.pid), str(ARGS.seconds), '1'],
                                      env=dict(os.environ, LC_ALL='C')).decode()
        got = list(stamps)
        if p.poll() is not None:
            raise RuntimeError('%s exited during the run' % label)
    finally:
        sub.unregister()
        if p.poll() is None:
            p.send_signal(signal.SIGKILL)
        p.wait()
        log.close()
        sensor.close()
        time.sleep(1.0)
    avg = [l.split() for l in out.splitlines() if l.startswith('Average') and re.search(r'\d', l)][-1]
    usr, system, cpu = float(avg[3]), float(avg[4]), float(avg[7])
    d = sorted((b - a) * 1000.0 for a, b in zip(got, got[1:]))
    pick = lambda q: d[min(len(d) - 1, int(q * len(d)))]  # noqa: E731
    over = sum(1 for x in d if abs(x - 10.0) > 1.0)
    print('%-6s sensor %3d Hz: CPU %5.2f %% (usr %.2f sys %.2f) | %d msgs, %.1f Hz | period ms min %.2f median %.2f '
          'p99 %.2f max %.2f | off 10 ms by > 1 ms: %.2f %%' % (
              label, rate, cpu, usr, system, len(got), len(got) / float(ARGS.seconds), d[0], pick(0.5), pick(0.99),
              d[-1], 100.0 * over / len(d)), flush=True)


def main():
    log = open(os.path.join(TMP, 'roscore.log'), 'wb')
    core = subprocess.Popen(['roscore', '-p', str(ARGS.port)], stdout=log, stderr=log, start_new_session=True)
    try:
        end = time.monotonic() + 15
        while not rosgraph.is_master_online():
            if time.monotonic() > end:
                raise RuntimeError('private roscore did not start')
            time.sleep(0.2)
        rospy.init_node('bno085_bench', disable_signals=True)
        drivers = [('python', [sys.executable, os.path.join(ROOT, 'nodes', 'bno085_node')])]
        if ARGS.cpp and os.access(ARGS.cpp, os.X_OK):
            drivers.append(('cpp', [ARGS.cpp]))
        else:
            print('SKIP cpp: --cpp / BNO085_CPP_NODE is not an executable')
        print('load average at the start: %s' % open('/proc/loadavg').read().strip(), flush=True)
        for rate in [int(r) for r in ARGS.rates.split(',')]:
            for label, cmd in drivers:
                measure(label, cmd, rate)
    finally:
        rospy.signal_shutdown('done')
        os.killpg(core.pid, signal.SIGINT)
        try:
            core.wait(8)
        except subprocess.TimeoutExpired:
            os.killpg(core.pid, signal.SIGKILL)
        log.close()
        shutil.rmtree(TMP, ignore_errors=True)


if __name__ == '__main__':
    main()
    sys.stdout.flush()
    os._exit(0)
