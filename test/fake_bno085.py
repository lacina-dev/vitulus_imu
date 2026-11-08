"""A BNO085 on a pty: enough SHTP over UART for adafruit_bno08x and bno085_node."""
import fcntl
import os
import pty
import struct
import threading
import time
import tty

CH_COMMAND, CH_EXE, CH_CONTROL, CH_REPORTS = 0, 1, 2, 3
REPORT_LEN = {0x01: 10, 0x02: 10, 0x03: 10, 0x04: 10, 0x05: 14, 0x06: 10, 0x08: 12, 0x09: 14,
              0x0F: 16}
REPORT_LEN.pop(0x06)


def frame(channel, seq, payload, end=b'\x7e'):
    body = struct.pack('<HBB', len(payload) + 4, channel, seq & 0xff) + bytes(payload)
    escaped = bytearray()
    for b in body:
        if b in (0x7e, 0x7d):
            escaped += bytes([0x7d, b ^ 0x20])
        else:
            escaped.append(b)
    return b'\x7e\x01' + bytes(escaped) + end


class FakeSensor(object):
    def __init__(self):
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        tty.setraw(self.master)
        flags = fcntl.fcntl(self.master, fcntl.F_GETFL)
        fcntl.fcntl(self.master, fcntl.F_SETFL, flags | os.O_NONBLOCK)
        self.path = os.ttyname(self.slave)
        self.mute = False            # answers nothing, sends nothing
        self.pre_stream = False      # streams before anybody asks (left by a dead node)
        self.ignore_adverts = 0      # so many advertisement requests go unanswered
        self.partial_once = False    # a cut frame, then deaf until the next request
        self.stream_stopped = False
        self.corrupt_one = False
        self.refuse = set()          # feature ids it does not enable (no answer, no stream)
        self.silent_reports = set()  # enabled, but these reports stop coming
        self.adverts = self.resets = 0
        self.features = set()
        self.seq = 0
        self.rx = bytearray()
        self.running = True
        self.lock = threading.Lock()
        self.threads = [threading.Thread(target=self._reader, daemon=True),
                        threading.Thread(target=self._streamer, daemon=True)]
        for t in self.threads:
            t.start()

    def close(self):
        self.running = False
        for t in self.threads:
            t.join(1.0)
        os.close(self.master)
        os.close(self.slave)

    def reset_state(self):
        with self.lock:
            self.features = set()
            self.rx = bytearray()

    def _send(self, data):
        try:
            os.write(self.master, data)
        except OSError:
            pass                     # full buffer: a real UART loses bytes as well

    def _reply(self, channel, payload):
        self.seq += 1
        self._send(frame(channel, self.seq, payload))

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
                self.rx += chunk
                self._parse()

    def _parse(self):
        while True:
            start = self.rx.find(b'\x7e\x01')
            if start < 0 or len(self.rx) < start + 4:
                return
            del self.rx[:start]
            size = struct.unpack_from('<H', self.rx, 2)[0]
            if len(self.rx) < 2 + size + 1:
                return
            packet = bytes(self.rx[2:2 + size])
            del self.rx[:2 + size + 1]
            self._on_packet(packet[2], packet[4:])

    def _on_packet(self, channel, data):
        if self.mute:
            return
        if channel == CH_COMMAND:
            self.adverts += 1
            if self.partial_once:
                self.partial_once = False
                self._send(b'\x7e\x01\x40\x00\x03\x07\x01\x02')
            elif self.ignore_adverts > 0:
                self.ignore_adverts -= 1
            else:
                self._reply(CH_COMMAND, bytes([0x00]) + bytes(20))
        elif channel == CH_EXE and data[:1] == b'\x01':
            self.resets += 1
            self.features = set()
            self.stream_stopped = False
            self._reply(CH_EXE, b'\x01')
        elif channel == CH_CONTROL and data[:1] == b'\xf9':
            self._reply(CH_CONTROL, bytes([0xf8, 0, 3, 2]) + struct.pack('<IIH', 10003608, 7, 1) + bytes(2))
        elif channel == CH_CONTROL and data[:1] == b'\xfd':
            feature = data[1]
            if feature in self.refuse:
                return
            self._reply(CH_CONTROL, bytes([0xfc, feature]) + bytes(15))
            self.features.add(feature)

    def _report(self, rid, n):
        size = REPORT_LEN[rid]
        words = [100 + n % 50, 200, 16000 if rid in (0x05, 0x08, 0x09) else 1000, 5000, 3, 4][:(size - 4) // 2]
        return bytes([rid, n & 0xff, 3, 0]) + struct.pack('<%dh' % len(words), *words)

    def _streamer(self):
        n = 0
        while self.running:
            time.sleep(0.05)         # the library default, 20 Hz per report
            n += 1
            if self.mute or self.stream_stopped:
                continue
            ids = sorted(self.features) if self.features else ([0x01, 0x02, 0x08] if self.pre_stream else [])
            ids = [i for i in ids if i in REPORT_LEN and i not in self.silent_reports]
            for rid in ids:
                self.seq += 1
                payload = bytes([0xfb, 0, 0, 0, 0]) + self._report(rid, n)
                if self.corrupt_one:
                    self.corrupt_one = False
                    self._send(frame(CH_REPORTS, self.seq, payload, end=b'\x55'))
                else:
                    self._send(frame(CH_REPORTS, self.seq, payload))
