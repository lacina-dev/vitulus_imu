// bno085_node_cpp: the BNO085 UART driver of nodes/bno085_node (Python) in C++.
//
// Same topics, types, frame, covariances, parameters, health JSON, exit codes,
// SHTP bytes and error handling as the Python node; see its docstring.  The
// protocol part follows adafruit_bno08x 1.1.10 (BNO08X / BNO08X_UART) as the
// Python node uses it through BoundedBNO, including its quirks (shared
// sequence numbers, reports of a batch processed last to first, readings zeroed
// by a feature response, a packet over 512 bytes read and dropped).  Every
// place that mirrors Python names the Python function.
//
// What differs, on purpose: the port is read in blocks (one poll + one read per
// burst instead of select + read per byte) and the library's busy waits during
// the start sleep 0.5 ms when nothing is there.
//
//   exit 0 shutdown, 1 connect failed, 2 packets stopped, 3 port lost, 4 hard exit
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/String.h>
#include <vitulus_msgs/Imu_status.h>

#include "bno085_supervisor.h"

namespace bno085 {

const int CHANNEL_SHTP_COMMAND = 0;
const int CHANNEL_EXE = 1;
const int CHANNEL_CONTROL = 2;
const int CHANNEL_SENSOR_REPORTS = 3;

const uint8_t REPORT_ACCELEROMETER = 0x01;
const uint8_t REPORT_GYROSCOPE = 0x02;
const uint8_t REPORT_MAGNETOMETER = 0x03;
const uint8_t REPORT_LINEAR_ACCELERATION = 0x04;
const uint8_t REPORT_ROTATION_VECTOR = 0x05;
const uint8_t REPORT_GAME_ROTATION_VECTOR = 0x08;
const uint8_t REPORT_GEOMAGNETIC_ROTATION_VECTOR = 0x09;
const uint8_t REPORT_MAG_UNCAL = 0x0F;
const uint8_t REPORT_STEP_COUNTER = 0x11;
const uint8_t REPORT_STABILITY_CLASSIFIER = 0x13;
const uint8_t REPORT_RAW_ACCELEROMETER = 0x14;
const uint8_t REPORT_RAW_GYROSCOPE = 0x15;
const uint8_t REPORT_RAW_MAGNETOMETER = 0x16;
const uint8_t REPORT_SHAKE_DETECTOR = 0x19;
const uint8_t REPORT_ACTIVITY_CLASSIFIER = 0x1E;

const uint8_t COMMAND_RESPONSE = 0xF1;
const uint8_t PRODUCT_ID_RESPONSE = 0xF8;
const uint8_t PRODUCT_ID_REQUEST = 0xF9;
const uint8_t TIMESTAMP_REBASE = 0xFA;
const uint8_t BASE_TIMESTAMP = 0xFB;
const uint8_t GET_FEATURE_RESPONSE = 0xFC;
const uint8_t SET_FEATURE_COMMAND = 0xFD;

const uint32_t DEFAULT_REPORT_INTERVAL = 50000;  // microseconds, the library default
const long FAST_REPORT_MIN_US = 5000;            // ~fast_report_interval_us: 5000-49999, else the default
const double FAST_REPEAT_AFTER_S = 0.05;         // no new game rotation vector for so long: repeat the last one
const double PACKET_READ_TIMEOUT = 2.0;
const double FEATURE_ENABLE_TIMEOUT = 2.0;
const double SHTP_ADVERTISE_RESEND_S = 1.0;
const size_t DATA_BUFFER_SIZE = 512;
const int SAVE_DCD = 0x06;
const double Q4 = 0.0625;

// ---------------------------------------------------------------------------
// Errors, by the Python exception they stand for.

enum class Err {
  UartSilent,         // nothing arrived within the read timeout
  ShutdownRequested,  // the node is shutting down while we wait for the sensor
  ImuInit,            // ImuInitError: the sensor did not complete the reset handshake
  Runtime,            // RuntimeError
  Packet,             // adafruit_bno08x.PacketError
  Index,              // IndexError
  Key,                // KeyError
  Serial,             // serial.SerialException (and SerialTimeoutException)
  OS                  // OSError
};

struct PyError {
  Err kind;
  std::string name;  // type(exc).__name__
  std::string msg;   // str(exc)
};

static PyError err(Err kind, const std::string& msg = "") {
  const char* name = "Exception";
  switch (kind) {
    case Err::UartSilent: name = "UartSilent"; break;
    case Err::ShutdownRequested: name = "ShutdownRequested"; break;
    case Err::ImuInit: name = "ImuInitError"; break;
    case Err::Runtime: name = "RuntimeError"; break;
    case Err::Packet: name = "PacketError"; break;
    case Err::Index: name = "IndexError"; break;
    case Err::Key: name = "KeyError"; break;
    case Err::Serial: name = "SerialException"; break;
    case Err::OS: name = "OSError"; break;
  }
  return PyError{kind, name, msg};
}

static std::string fmt(const char* format, ...) __attribute__((format(printf, 1, 2)));
static std::string fmt(const char* format, ...) {
  char text[512];
  va_list ap;
  va_start(ap, format);
  vsnprintf(text, sizeof(text), format, ap);
  va_end(ap);
  return text;
}

static std::string errnoText(int e) { return fmt("[Errno %d] %s", e, strerror(e)); }

static double wallTime() {
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

// ---------------------------------------------------------------------------
// The port: serial.Serial(port, baudrate, timeout, write_timeout, exclusive=True)
// behind GuardedUart, with a block buffer in front of read(1).

struct KernelTermios2 {  // struct termios2 of <asm/termbits.h>, which clashes with <termios.h>
  tcflag_t c_iflag, c_oflag, c_cflag, c_lflag;
  cc_t c_line;
  cc_t c_cc[19];
  speed_t c_ispeed, c_ospeed;
};
#define BNO_TCGETS2 _IOR('T', 0x2A, KernelTermios2)
#define BNO_TCSETS2 _IOW('T', 0x2B, KernelTermios2)
const tcflag_t BNO_BOTHER = 0010000;
const tcflag_t BNO_CBAUD = 0010017;

static bool standardBaud(long baud, speed_t* out) {
  static const struct { long baud; speed_t speed; } table[] = {
      {0, B0}, {50, B50}, {75, B75}, {110, B110}, {134, B134}, {150, B150}, {200, B200},
      {300, B300}, {600, B600}, {1200, B1200}, {1800, B1800}, {2400, B2400}, {4800, B4800},
      {9600, B9600}, {19200, B19200}, {38400, B38400}, {57600, B57600}, {115200, B115200},
      {230400, B230400}, {460800, B460800}, {500000, B500000}, {576000, B576000},
      {921600, B921600}, {1000000, B1000000}, {1152000, B1152000}, {1500000, B1500000},
      {2000000, B2000000}, {2500000, B2500000}, {3000000, B3000000}, {3500000, B3500000},
      {4000000, B4000000}};
  for (const auto& row : table) {
    if (row.baud == baud) {
      *out = row.speed;
      return true;
    }
  }
  return false;
}

class GuardedUart {
 public:
  GuardedUart(const std::string& port, long baudrate, double read_timeout,
              std::function<bool()> should_stop)
      : should_stop_(std::move(should_stop)), timeout_(read_timeout) {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      const int e = errno;
      throw err(Err::Serial, fmt("[Errno %d] could not open port %s: [Errno %d] %s: '%s'", e,
                                 port.c_str(), e, strerror(e), port.c_str()));
    }
    try {
      configure(port, baudrate);
      // pyserial raises DTR and RTS at open; a pty has neither.
      for (int line : {TIOCM_DTR, TIOCM_RTS}) {
        if (ioctl(fd_, TIOCMBIS, &line) < 0 && errno != EINVAL && errno != ENOTTY)
          throw err(Err::OS, errnoText(errno));
      }
      tcflush(fd_, TCIFLUSH);  // Serial.open
      tcflush(fd_, TCIFLUSH);  // GuardedUart: reset_input_buffer
      tcflush(fd_, TCOFLUSH);  //              reset_output_buffer
    } catch (...) {
      ::close(fd_);
      fd_ = -1;
      throw;
    }
  }

  ~GuardedUart() { close(); }
  GuardedUart(const GuardedUart&) = delete;
  GuardedUart& operator=(const GuardedUart&) = delete;

  void close() {
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
  }

  // GuardedUart.read(1): a byte, or UartSilent / ShutdownRequested after the
  // read timeout, or SerialException when the port is gone.
  inline uint8_t readByte() {
    if (pos_ == len_) fill();
    return buf_[pos_++];
  }

  void writeByte(uint8_t b) {
    const double end = monotonic() + timeout_;
    while (true) {
      const ssize_t n = ::write(fd_, &b, 1);
      if (n < 0 && errno != EAGAIN && errno != EALREADY && errno != EWOULDBLOCK &&
          errno != EINPROGRESS && errno != EINTR)
        throw err(Err::Serial, "write failed: " + errnoText(errno));
      // As pyserial: wait until the port takes more, at most the write timeout.
      const double left = end - monotonic();
      if (left <= 0.0) throw serialTimeout();
      struct pollfd p = {fd_, POLLOUT, 0};
      const int r = ::poll(&p, 1, static_cast<int>(std::ceil(left * 1000.0)));
      if (r < 0 && errno != EINTR) throw err(Err::Serial, "write failed: " + errnoText(errno));
      if (r == 0) throw serialTimeout();
      if (n == 1) return;
    }
  }

  // Serial.in_waiting, with the bytes we already took from the kernel.
  size_t inWaiting() {
    int n = 0;
    if (ioctl(fd_, TIOCINQ, &n) < 0) throw err(Err::OS, errnoText(errno));
    return (len_ - pos_) + static_cast<size_t>(n);
  }

 private:
  static PyError serialTimeout() {
    PyError e = err(Err::Serial, "Write timeout");
    e.name = "SerialTimeoutException";
    return e;
  }

  void configure(const std::string& port, long baudrate) {
    // exclusive=True: the lock comes before anything on the line is changed.
    if (flock(fd_, LOCK_EX | LOCK_NB) < 0) {
      const int e = errno;
      throw err(Err::Serial, fmt("[Errno %d] Could not exclusively lock port %s: %s", e,
                                 port.c_str(), errnoText(e).c_str()));
    }
    struct termios tio;
    if (tcgetattr(fd_, &tio) < 0)
      throw err(Err::Serial, "Could not configure port: " + fmt("(%d, '%s')", errno, strerror(errno)));
    speed_t speed = B38400;
    const bool custom = !standardBaud(baudrate, &speed);
    if (custom && baudrate <= 0)
      throw err(Err::Serial, fmt("Invalid baud rate: %ld", baudrate));  // pyserial: ValueError
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN | ECHOCTL | ECHOKE);
    tio.c_oflag &= ~(OPOST | ONLCR | OCRNL);
    tio.c_iflag &= ~(INLCR | IGNCR | ICRNL | IGNBRK | IUCLC | PARMRK);
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~CSTOPB;
    tio.c_iflag &= ~(INPCK | ISTRIP);
    tio.c_cflag &= ~(PARENB | PARODD | CMSPAR);
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &tio) < 0)
      throw err(Err::Serial, "Could not configure port: " + fmt("(%d, '%s')", errno, strerror(errno)));
    if (custom) {
      KernelTermios2 t2;
      if (ioctl(fd_, BNO_TCGETS2, &t2) < 0)
        throw err(Err::Serial, fmt("Failed to set custom baud rate (%ld): %s", baudrate,
                                   errnoText(errno).c_str()));
      t2.c_cflag &= ~BNO_CBAUD;
      t2.c_cflag |= BNO_BOTHER;
      t2.c_ispeed = t2.c_ospeed = static_cast<speed_t>(baudrate);
      if (ioctl(fd_, BNO_TCSETS2, &t2) < 0)
        throw err(Err::Serial, fmt("Failed to set custom baud rate (%ld): %s", baudrate,
                                   errnoText(errno).c_str()));
    }
  }

  void fill() {
    const double end = monotonic() + timeout_;
    while (true) {
      double left = end - monotonic();
      if (left < 0.0) left = 0.0;
      struct pollfd p = {fd_, POLLIN, 0};
      const int r = ::poll(&p, 1, static_cast<int>(std::ceil(left * 1000.0)));
      if (r < 0) {
        if (errno == EAGAIN || errno == EALREADY || errno == EWOULDBLOCK || errno == EINPROGRESS ||
            errno == EINTR) {
          if (monotonic() < end && !should_stop_()) continue;
          silent();
        }
        throw err(Err::Serial, "read failed: " + errnoText(errno));
      }
      if (r == 0) silent();
      const ssize_t n = ::read(fd_, buf_, sizeof(buf_));
      if (n > 0) {
        pos_ = 0;
        len_ = static_cast<size_t>(n);
        return;
      }
      if (n == 0)
        throw err(Err::Serial, "device reports readiness to read but returned no data "
                               "(device disconnected or multiple access on port?)");
      if (errno != EAGAIN && errno != EALREADY && errno != EWOULDBLOCK && errno != EINPROGRESS &&
          errno != EINTR)
        throw err(Err::Serial, "read failed: " + errnoText(errno));
      if (monotonic() >= end) silent();
    }
  }

  [[noreturn]] void silent() {
    if (should_stop_()) throw err(Err::ShutdownRequested);
    throw err(Err::UartSilent);
  }

  std::function<bool()> should_stop_;
  double timeout_;
  int fd_ = -1;
  size_t pos_ = 0, len_ = 0;
  uint8_t buf_[4096];
};

// ---------------------------------------------------------------------------
// What the status thread may read of a sensor object (self.bno in Python).

struct Telemetry {
  std::mutex m;
  bool has_report_t = false;
  double last_report_t = 0.0;
  int frame_errors = 0;
  int mag_accuracy = 0;         // _magnetometer_accuracy
  bool has_uncal = false;       // mag_uncal / mag_bias / mag_uncal_accuracy are not None
  double mag_bias[3] = {0.0, 0.0, 0.0};
  int mag_uncal_accuracy = 0;
};

struct Packet {
  int channel = 0;
  int sequence = 0;
  size_t data_length = 0;
  const uint8_t* data = nullptr;
};

struct Reading {
  bool has = false;
  double v[4] = {0.0, 0.0, 0.0, 0.0};
};

// BoundedBNO(BNO08X_UART): the sensor, with a time limit on everything in initialize().
class BoundedBNO {
 public:
  BoundedBNO(GuardedUart& uart, double advert_timeout, std::function<bool()> should_stop)
      : tele(std::make_shared<Telemetry>()), uart_(uart), advert_timeout_(advert_timeout),
        should_stop_(std::move(should_stop)), data_buffer_(DATA_BUFFER_SIZE, 0) {
    slices_.reserve(64);
  }

  std::shared_ptr<Telemetry> tele;
  double mag_uncal[3] = {0.0, 0.0, 0.0};  // uT
  long mag_uncal_count = 0;
  long game_count = 0;  // game rotation vector reports taken

  // One try only: the caller reopens the port between tries.
  void initialize() {
    softReset();
    checkId();
  }

  void enableFeature(uint8_t feature_id) {
    uint8_t report[17] = {0};
    report[0] = SET_FEATURE_COMMAND;
    report[1] = feature_id;
    report[5] = DEFAULT_REPORT_INTERVAL & 0xFF;
    report[6] = (DEFAULT_REPORT_INTERVAL >> 8) & 0xFF;
    report[7] = (DEFAULT_REPORT_INTERVAL >> 16) & 0xFF;
    report[8] = (DEFAULT_REPORT_INTERVAL >> 24) & 0xFF;
    sendPacket(CHANNEL_CONTROL, report, sizeof(report));
    const double start = monotonic();
    while (monotonic() - start < FEATURE_ENABLE_TIMEOUT) {
      processAvailablePackets(10);
      if (readings_[feature_id].has) return;
      if (!dataReady()) sleepS(0.0005);
    }
    throw err(Err::Runtime, fmt("('Was not able to enable feature', %d)", feature_id));
  }

  // BoundedBNO.enable_feature_at: another report interval; waits for the feature
  // response and returns the interval the sensor took [us].
  uint32_t enableFeatureAt(uint8_t feature_id, uint32_t interval_us) {
    uint8_t report[17] = {0};
    report[0] = SET_FEATURE_COMMAND;
    report[1] = feature_id;
    report[5] = interval_us & 0xFF;
    report[6] = (interval_us >> 8) & 0xFF;
    report[7] = (interval_us >> 16) & 0xFF;
    report[8] = (interval_us >> 24) & 0xFF;
    feature_answered_[feature_id] = false;
    sendPacket(CHANNEL_CONTROL, report, sizeof(report));
    const double start = monotonic();
    while (monotonic() - start < FEATURE_ENABLE_TIMEOUT) {
      processAvailablePackets(10);
      if (feature_answered_[feature_id]) return feature_interval_[feature_id];
      if (!dataReady()) sleepS(0.0005);
    }
    throw err(Err::Runtime, fmt("('Was not able to enable feature', %d)", feature_id));
  }

  uint32_t featureInterval(uint8_t feature_id) const { return feature_interval_[feature_id]; }

  void poll() { processAvailablePackets(200); }

  bool reportAge(double* age) const {
    if (!tele->has_report_t) return false;
    *age = monotonic() - tele->last_report_t;
    return true;
  }

  void addFrameError() {
    std::lock_guard<std::mutex> lock(tele->m);
    tele->frame_errors += 1;
  }

  int frameErrors() const { return tele->frame_errors; }

  // The library properties: each reads what is waiting first.
  const double* magnetic() { return reading(REPORT_MAGNETOMETER, "No magfield report found, is it enabled?"); }
  const double* acceleration() { return reading(REPORT_ACCELEROMETER, "No accel report found, is it enabled?"); }
  const double* gyro() { return reading(REPORT_GYROSCOPE, "No gyro report found, is it enabled?"); }
  const double* quaternion() { return reading(REPORT_ROTATION_VECTOR, "No quaternion report found, is it enabled?"); }
  const double* gameQuaternion() {
    return reading(REPORT_GAME_ROTATION_VECTOR, "No game quaternion report found, is it enabled?");
  }

 private:
  const double* reading(uint8_t id, const char* missing) {
    processAvailablePackets(0);
    if (!readings_[id].has) throw err(Err::Runtime, missing);
    return readings_[id].v;
  }

  bool dataReady() { return uart_.inWaiting() >= 4; }

  // BNO08X_UART._send_packet: no escaping, a pause after every byte.
  void sendPacket(int channel, const uint8_t* data, size_t data_length) {
    const size_t write_length = data_length + 4;
    data_buffer_[0] = write_length & 0xFF;
    data_buffer_[1] = (write_length >> 8) & 0xFF;
    data_buffer_[2] = static_cast<uint8_t>(channel);
    data_buffer_[3] = static_cast<uint8_t>(sequence_number_[channel]);
    memcpy(&data_buffer_[4], data, data_length);
    uart_.writeByte(0x7E);  // start byte
    sleepS(0.001);
    uart_.writeByte(0x01);  // SHTP byte
    sleepS(0.001);
    for (size_t i = 0; i < write_length; ++i) {
      uart_.writeByte(data_buffer_[i]);
      sleepS(0.001);
    }
    sleepS(0.001);
    uart_.writeByte(0x7E);  // end byte
    sequence_number_[channel] = (sequence_number_[channel] + 1) % 256;
  }

  // BNO08X_UART._read_into
  inline void readInto(size_t start, size_t end) {
    for (size_t idx = start; idx < end; ++idx) {
      uint8_t b = uart_.readByte();
      if (b == 0x7D) b = uart_.readByte() ^ 0x20;  // control escape
      data_buffer_[idx] = b;
    }
  }

  // BNO08X_UART._read_header
  void readHeader() {
    while (uart_.readByte() != 0x7E) {
    }
    uint8_t b = uart_.readByte();
    if (b == 0x7E) b = uart_.readByte();  // second 0x7e
    if (b != 0x01) throw err(Err::Runtime, "Unhandled UART control SHTP protocol");
    readInto(0, 4);
  }

  // BNO08X_UART._read_packet under BoundedBNO._read_packet
  Packet readPacket() {
    readHeader();
    const size_t packet_byte_count = (data_buffer_[0] | (data_buffer_[1] << 8)) & 0x7FFF;
    const int channel_number = data_buffer_[2];
    const int sequence_number = data_buffer_[3];
    if (channel_number > 5) throw err(Err::Index, "list assignment index out of range");
    sequence_number_[channel_number] = sequence_number;
    if (packet_byte_count == 0) throw err(Err::Packet, "No packet available");
    // The library takes a new, empty buffer for a packet over 512 bytes and
    // loses the header with it: such a packet is read and comes out as an empty
    // packet of channel 0.
    const bool dropped = packet_byte_count > DATA_BUFFER_SIZE;
    if (dropped) data_buffer_.assign(packet_byte_count, 0);
    readInto(4, packet_byte_count);
    if (uart_.readByte() != 0x7E) throw err(Err::Runtime, "Didn't find packet end");
    Packet packet;
    if (!dropped) {
      packet.channel = channel_number;
      packet.sequence = sequence_number;
      packet.data_length = packet_byte_count > 4 ? packet_byte_count - 4 : 0;
      packet.data = &data_buffer_[4];
    }
    sequence_number_[packet.channel] = packet.sequence;  // _update_sequence_number
    if (packet.channel == CHANNEL_SENSOR_REPORTS) {
      std::lock_guard<std::mutex> lock(tele->m);
      tele->last_report_t = monotonic();
      tele->has_report_t = true;
    }
    return packet;
  }

  // BNO08X._process_available_packets; max_packets 0 = no limit.
  void processAvailablePackets(int max_packets) {
    int processed_count = 0;
    while (dataReady()) {
      if (max_packets && processed_count > max_packets) return;
      Packet packet;
      try {
        packet = readPacket();
      } catch (const PyError& e) {
        if (e.kind == Err::Packet) continue;
        throw;
      }
      handlePacket(packet);
      processed_count += 1;
    }
  }

  // BNO08X._wait_for_packet
  Packet waitForPacket() {
    const double start = monotonic();
    while (monotonic() - start < PACKET_READ_TIMEOUT) {
      if (!dataReady()) {
        sleepS(0.0005);
        continue;
      }
      return readPacket();
    }
    throw err(Err::Runtime, "Timed out waiting for a packet");
  }

  // BNO08X._wait_for_packet_type; the wanted packet is returned, not handled.
  Packet waitForPacketType(int channel_number, uint8_t report_id, double timeout = 5.0) {
    const double start = monotonic();
    while (monotonic() - start < timeout) {
      const Packet packet = waitForPacket();
      if (packet.channel == channel_number) {
        if (packet.data_length == 0) throw err(Err::Index, "bytearray index out of range");
        if (packet.data[0] == report_id) return packet;
      }
      if (packet.channel != CHANNEL_EXE && packet.channel != CHANNEL_SHTP_COMMAND) handlePacket(packet);
    }
    throw err(Err::Runtime, fmt("('Timed out waiting for a packet on channel', %d)", channel_number));
  }

  // BoundedBNO.soft_reset
  void softReset() {
    const double deadline = monotonic() + advert_timeout_;
    double resend_t = 0.0;
    while (true) {
      if (should_stop_()) throw err(Err::ShutdownRequested);
      const double now = monotonic();
      if (now >= deadline) throw err(Err::ImuInit, fmt("no SHTP answer in %.1f s", advert_timeout_));
      if (now >= resend_t) {
        const uint8_t advertise[2] = {0, 1};
        sendPacket(CHANNEL_SHTP_COMMAND, advertise, 2);
        resend_t = monotonic() + SHTP_ADVERTISE_RESEND_S;
        sleepS(0.5);
      }
      Packet packet;
      try {
        packet = readPacket();
      } catch (const PyError& e) {
        if (e.kind == Err::UartSilent) continue;
        if (e.kind == Err::Runtime || e.kind == Err::Packet || e.kind == Err::Index) {
          addFrameError();  // we joined the stream in the middle of a frame
          continue;
        }
        throw;
      }
      if (packet.channel == CHANNEL_SHTP_COMMAND) break;
    }
    const uint8_t reset[1] = {1};
    sendPacket(CHANNEL_EXE, reset, 1);
    sleepS(0.5);
    sendPacket(CHANNEL_EXE, reset, 1);
    sleepS(0.5);
  }

  // BNO08X._check_id
  void checkId() {
    if (id_read_) return;
    const uint8_t request[2] = {PRODUCT_ID_REQUEST, 0};
    sendPacket(CHANNEL_CONTROL, request, 2);
    while (true) {
      waitForPacketType(CHANNEL_CONTROL, PRODUCT_ID_RESPONSE);
      // _parse_sensor_id: the software part number, from the buffer of the last packet
      if (data_buffer_.size() >= 12 && data_buffer_[4] == PRODUCT_ID_RESPONSE) {
        const uint32_t part = data_buffer_[8] | (data_buffer_[9] << 8) | (data_buffer_[10] << 16) |
                              (static_cast<uint32_t>(data_buffer_[11]) << 24);
        if (part) {
          id_read_ = true;
          return;
        }
      }
    }
  }

  // adafruit_bno08x._report_length (with 0x0F as register_mag_uncal adds it)
  static int reportLength(uint8_t id) {
    switch (id) {
      case REPORT_ACCELEROMETER:
      case REPORT_GYROSCOPE:
      case REPORT_MAGNETOMETER:
      case REPORT_LINEAR_ACCELERATION: return 10;
      case REPORT_ROTATION_VECTOR:
      case REPORT_GEOMAGNETIC_ROTATION_VECTOR: return 14;
      case REPORT_GAME_ROTATION_VECTOR: return 12;
      case REPORT_STEP_COUNTER: return 12;
      case REPORT_SHAKE_DETECTOR:
      case REPORT_STABILITY_CLASSIFIER: return 6;
      case REPORT_ACTIVITY_CLASSIFIER:
      case REPORT_RAW_ACCELEROMETER:
      case REPORT_RAW_GYROSCOPE:
      case REPORT_RAW_MAGNETOMETER:
      case REPORT_MAG_UNCAL: return 16;
      case PRODUCT_ID_RESPONSE:
      case COMMAND_RESPONSE: return 16;
      case GET_FEATURE_RESPONSE: return 17;
      case BASE_TIMESTAMP:
      case TIMESTAMP_REBASE: return 5;
      default: throw err(Err::Key, fmt("%d", id));
    }
  }

  // BoundedBNO._handle_packet: _separate_batch, then the reports last to first.
  void handlePacket(const Packet& packet) {
    uncal_in_packet_ = false;
    slices_.clear();
    size_t next = 0;
    while (next < packet.data_length) {
      const int required = reportLength(packet.data[next]);
      const size_t unprocessed = packet.data_length - next;
      if (unprocessed < static_cast<size_t>(required))
        throw err(Err::Runtime, fmt("('Unprocessable Batch bytes', %zu)", unprocessed));
      slices_.push_back(packet.data + next);
      next += static_cast<size_t>(required);
    }
    while (!slices_.empty()) {
      const uint8_t* report = slices_.back();
      slices_.pop_back();
      processReport(report[0], report);
    }
  }

  static inline int16_t i16(const uint8_t* p) { return static_cast<int16_t>(p[0] | (p[1] << 8)); }

  // BoundedBNO._process_report over BNO08X._process_report
  void processReport(uint8_t id, const uint8_t* r) {
    if (id == REPORT_MAG_UNCAL) {
      // Reports of a batch come newest first here: keep the first.
      if (uncal_in_packet_) return;
      uncal_in_packet_ = true;
      // parse_mag_uncal: X Y Z uncalibrated, X Y Z hard-iron bias, int16 Q4 microtesla
      std::lock_guard<std::mutex> lock(tele->m);
      for (int i = 0; i < 3; ++i) {
        mag_uncal[i] = i16(r + 4 + 2 * i) * Q4;
        tele->mag_bias[i] = i16(r + 10 + 2 * i) * Q4;
      }
      tele->mag_uncal_accuracy = r[2] & 0x03;
      tele->has_uncal = true;
      mag_uncal_count += 1;
      return;
    }
    if (id >= 0xF0) {  // _handle_control_report
      if (id == GET_FEATURE_RESPONSE) {
        Reading& reading = readings_[r[1]];  // _INITIAL_REPORTS: zeros
        reading = Reading();
        reading.has = true;
        feature_interval_[r[1]] = static_cast<uint32_t>(r[5]) | (static_cast<uint32_t>(r[6]) << 8) |
                                  (static_cast<uint32_t>(r[7]) << 16) | (static_cast<uint32_t>(r[8]) << 24);
        feature_answered_[r[1]] = true;
      } else if (id == COMMAND_RESPONSE) {
        if (r[2] == SAVE_DCD && r[5] != 0) throw err(Err::Runtime, "Unable to save calibration data");
      }
      return;
    }
    double scalar = 1.0;
    int count = 3;
    switch (id) {
      case REPORT_STEP_COUNTER:
      case REPORT_SHAKE_DETECTOR: return;
      case REPORT_STABILITY_CLASSIFIER:
        if (r[4] > 4) throw err(Err::Index, "list index out of range");
        return;
      case REPORT_ACTIVITY_CLASSIFIER:
        if (r[5] > 8) throw err(Err::Index, "list index out of range");
        return;
      case REPORT_ACCELEROMETER:
      case REPORT_LINEAR_ACCELERATION: scalar = 1.0 / 256.0; break;       // Q8, m/s2
      case REPORT_GYROSCOPE: scalar = 1.0 / 512.0; break;                 // Q9, rad/s
      case REPORT_MAGNETOMETER: scalar = 1.0 / 16.0; break;               // Q4, uT
      case REPORT_ROTATION_VECTOR:
      case REPORT_GAME_ROTATION_VECTOR: scalar = 1.0 / 16384.0; count = 4; break;       // Q14
      case REPORT_GEOMAGNETIC_ROTATION_VECTOR: scalar = 1.0 / 4096.0; count = 4; break;  // Q12
      default: break;  // raw reports: unsigned, scale 1
    }
    const bool raw = id == REPORT_RAW_ACCELEROMETER || id == REPORT_RAW_GYROSCOPE || id == REPORT_RAW_MAGNETOMETER;
    Reading& reading = readings_[id];
    for (int i = 0; i < count; ++i) {
      const int16_t v = i16(r + 4 + 2 * i);
      reading.v[i] = (raw ? static_cast<double>(static_cast<uint16_t>(v)) : static_cast<double>(v)) * scalar;
    }
    reading.has = true;
    if (id == REPORT_GAME_ROTATION_VECTOR) game_count += 1;
    if (id == REPORT_MAGNETOMETER) {
      std::lock_guard<std::mutex> lock(tele->m);
      tele->mag_accuracy = r[2] & 0x03;
    }
  }

  GuardedUart& uart_;
  double advert_timeout_;
  std::function<bool()> should_stop_;
  std::vector<uint8_t> data_buffer_;
  std::vector<const uint8_t*> slices_;
  int sequence_number_[6] = {0, 0, 0, 0, 0, 0};
  Reading readings_[256];
  uint32_t feature_interval_[256] = {0};  // interval of the last feature response [us]
  bool feature_answered_[256] = {false};
  bool uncal_in_packet_ = false;
  bool id_read_ = false;
};

// ---------------------------------------------------------------------------
// json.dumps and float repr of Python, for bno085/health.

static std::string pyFloat(double v) {
  if (std::isnan(v)) return "NaN";
  if (std::isinf(v)) return v > 0 ? "Infinity" : "-Infinity";
  char text[40];
  int precision = 1;
  for (; precision <= 17; ++precision) {  // the shortest digits that read back the same
    snprintf(text, sizeof(text), "%.*e", precision - 1, v);
    if (strtod(text, nullptr) == v) break;
  }
  std::string s(text);
  const size_t epos = s.find('e');
  const int exponent = atoi(s.c_str() + epos + 1);
  std::string mantissa = s.substr(0, epos);
  std::string sign;
  if (mantissa[0] == '-') {
    sign = "-";
    mantissa.erase(0, 1);
  }
  std::string digits;
  for (char c : mantissa)
    if (c != '.') digits += c;
  while (digits.size() > 1 && digits.back() == '0') digits.pop_back();
  if (exponent < -4 || exponent >= 16) {
    std::string out = sign + digits.substr(0, 1);
    if (digits.size() > 1) out += "." + digits.substr(1);
    return out + fmt("e%c%02d", exponent < 0 ? '-' : '+', std::abs(exponent));
  }
  if (exponent < 0) return sign + "0." + std::string(static_cast<size_t>(-exponent - 1), '0') + digits;
  const size_t whole = static_cast<size_t>(exponent) + 1;
  if (digits.size() <= whole) return sign + digits + std::string(whole - digits.size(), '0') + ".0";
  return sign + digits.substr(0, whole) + "." + digits.substr(whole);
}

static std::string pyRound2(double v) {  // repr(round(v, 2))
  char text[64];
  snprintf(text, sizeof(text), "%.2f", v);
  return pyFloat(strtod(text, nullptr));
}

static std::string jsonString(const std::string& s) {  // ensure_ascii, as json.dumps
  std::string out = "\"";
  size_t i = 0;
  while (i < s.size()) {
    const unsigned char c = static_cast<unsigned char>(s[i]);
    uint32_t cp = c;
    size_t extra = 0;
    if (c >= 0xF0) { cp = c & 0x07; extra = 3; }
    else if (c >= 0xE0) { cp = c & 0x0F; extra = 2; }
    else if (c >= 0xC0) { cp = c & 0x1F; extra = 1; }
    if (extra && i + extra >= s.size()) {  // cut sequence: the byte as it is
      extra = 0;
      cp = c;
    }
    for (size_t k = 1; k <= extra; ++k) cp = (cp << 6) | (static_cast<unsigned char>(s[i + k]) & 0x3F);
    i += extra + 1;
    switch (cp) {
      case '"': out += "\\\""; continue;
      case '\\': out += "\\\\"; continue;
      case '\n': out += "\\n"; continue;
      case '\r': out += "\\r"; continue;
      case '\t': out += "\\t"; continue;
      case '\b': out += "\\b"; continue;
      case '\f': out += "\\f"; continue;
      default: break;
    }
    if (cp >= 0x20 && cp < 0x7F) {
      out += static_cast<char>(cp);
    } else if (cp < 0x10000) {
      out += fmt("\\u%04x", cp);
    } else {
      cp -= 0x10000;
      out += fmt("\\u%04x\\u%04x", 0xD800 + (cp >> 10), 0xDC00 + (cp & 0x3FF));
    }
  }
  return out + "\"";
}

// ---------------------------------------------------------------------------
// Parameters as rospy.get_param(name, default) takes them: whatever type is set.

static double paramDouble(const std::string& name, double fallback) {
  XmlRpc::XmlRpcValue v;
  if (!ros::param::get(name, v)) return fallback;
  if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble) return static_cast<double>(v);
  if (v.getType() == XmlRpc::XmlRpcValue::TypeInt) return static_cast<int>(v);
  if (v.getType() == XmlRpc::XmlRpcValue::TypeBoolean) return static_cast<bool>(v) ? 1.0 : 0.0;
  if (v.getType() == XmlRpc::XmlRpcValue::TypeString) return atof(static_cast<std::string>(v).c_str());
  return fallback;
}

static long paramInt(const std::string& name, long fallback) {
  XmlRpc::XmlRpcValue v;
  if (!ros::param::get(name, v)) return fallback;
  if (v.getType() == XmlRpc::XmlRpcValue::TypeInt) return static_cast<int>(v);
  return static_cast<long>(paramDouble(name, static_cast<double>(fallback)));
}

static bool paramTruthy(const std::string& name, bool fallback) {
  XmlRpc::XmlRpcValue v;
  if (!ros::param::get(name, v)) return fallback;
  if (v.getType() == XmlRpc::XmlRpcValue::TypeBoolean) return static_cast<bool>(v);
  if (v.getType() == XmlRpc::XmlRpcValue::TypeInt) return static_cast<int>(v) != 0;
  if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble) return static_cast<double>(v) != 0.0;
  if (v.getType() == XmlRpc::XmlRpcValue::TypeString) return !static_cast<std::string>(v).empty();
  return true;
}

// ---------------------------------------------------------------------------

static std::atomic<bool> g_signal(false);

static void onSignal(int) {
  g_signal = true;
  ros::requestShutdown();
}

static bool isShutdown() { return g_signal.load() || ros::isShuttingDown() || !ros::ok(); }

class BNO085Node {
 public:
  BNO085Node()
      : supervisor_(isShutdown, [this] { publishStatus(); },
                    [](const std::string& text) { ROS_ERROR_STREAM(text); }) {
    ros::param::param<std::string>("~port", port_, "/dev/imu");
    baud_ = paramInt("~baud", 3000000);
    read_timeout_s_ = paramDouble("~read_timeout_s", 0.2);
    advert_timeout_s_ = paramDouble("~advert_timeout_s", 3.0);
    connect_attempts_ = paramInt("~connect_attempts", 2);
    init_limit_s_ = paramDouble("~init_limit_s", 40.0);
    run_limit_s_ = paramDouble("~run_limit_s", 5.0);
    stale_s_ = paramDouble("~stale_s", 0.5);
    stall_timeout_s_ = paramDouble("~stall_timeout_s", 2.0);
    stable_s_ = paramDouble("~stable_s", 10.0);
    backoff_max_s_ = paramDouble("~backoff_max_s", 30.0);
    max_failures_ = paramInt("~max_failures", 8);
    want_mag_uncal_ = paramTruthy("~mag_uncal", true);
    mag_uncal_warn_s_ = paramDouble("~mag_uncal_warn_s", 5.0);
    mag_uncal_stale_s_ = paramDouble("~mag_uncal_stale_s", 2.0);
    failures_ = paramInt("~failures", 0);

    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("bno085/imu", 10);
    navheading_pub_ = nh_.advertise<sensor_msgs::Imu>("bno085/navheading", 10);
    mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("bno085/mag", 10);
    mag_uncal_pub_ = nh_.advertise<sensor_msgs::MagneticField>("bno085/mag_uncal", 10);
    imu_status_pub_ = nh_.advertise<vitulus_msgs::Imu_status>("bno085/imu_status", 10);
    health_pub_ = nh_.advertise<std_msgs::String>("bno085/health", 1, true);
    name_ = ros::this_node::getName();
    fast_interval_us_ = readFastInterval();
    supervisor_.start();
  }

  int run() {
    int code = EXIT_OK;
    if (connect()) {
      code = spin();
    } else if (!isShutdown()) {
      noteFailure("connecting", reason().empty() ? "no sensor" : reason());
      code = EXIT_CONNECT;
    }
    supervisor_.leaveWithin(2.0, code);
    closeUart();
    return code;
  }

 private:
  // ~fast_report_interval_us as an interval the node may ask for; anything else
  // is the default.
  long readFastInterval() const {
    XmlRpc::XmlRpcValue v;
    if (!ros::param::get("~fast_report_interval_us", v)) return DEFAULT_REPORT_INTERVAL;
    long value = -1;
    if (v.getType() == XmlRpc::XmlRpcValue::TypeInt) {
      value = static_cast<int>(v);
    } else if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
      const double d = static_cast<double>(v);
      if (std::isfinite(d) && std::fabs(d) < 1e9) value = static_cast<long>(d);
    } else if (v.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
      value = static_cast<bool>(v) ? 1 : 0;
    } else if (v.getType() == XmlRpc::XmlRpcValue::TypeString) {
      const std::string text = static_cast<std::string>(v);
      char* end = nullptr;
      const double d = strtod(text.c_str(), &end);
      if (end != text.c_str() && *end == '\0' && std::isfinite(d) && std::fabs(d) < 1e9)
        value = static_cast<long>(d);
    }
    if (value == static_cast<long>(DEFAULT_REPORT_INTERVAL)) return value;
    if (value < FAST_REPORT_MIN_US || value >= static_cast<long>(DEFAULT_REPORT_INTERVAL)) {
      ROS_WARN_STREAM("<" << name_ << "> fast_report_interval_us is not within " << FAST_REPORT_MIN_US
                          << "-" << DEFAULT_REPORT_INTERVAL << ", using " << DEFAULT_REPORT_INTERVAL);
      return DEFAULT_REPORT_INTERVAL;
    }
    return value;
  }

  // One of the fast reports at ~fast_report_interval_us.  When the sensor does
  // not take it, this report and the ones before it go back to the default
  // interval and the node runs as without the parameter.
  void enableFast(uint8_t feature) {
    std::string why;
    try {
      const uint32_t granted = bno_->enableFeatureAt(feature, static_cast<uint32_t>(fast_interval_us_));
      if (!(granted > 0 && granted < DEFAULT_REPORT_INTERVAL))
        why = fmt("the sensor answered %lu us", static_cast<unsigned long>(granted));
    } catch (const PyError& e) {
      if (e.kind != Err::Runtime) throw;
      why = "no answer";
    }
    if (why.empty()) {
      fast_done_.push_back(feature);
      uint32_t slowest = 0;
      for (uint8_t f : fast_done_) slowest = std::max(slowest, bno_->featureInterval(f));
      std::lock_guard<std::mutex> lock(state_m_);
      fast_granted_us_ = slowest;
      return;
    }
    {
      std::lock_guard<std::mutex> lock(state_m_);
      fast_state_ = "fallback";
      fast_granted_us_ = DEFAULT_REPORT_INTERVAL;
    }
    fast_on_ = false;
    ROS_WARN_STREAM("<" << name_ << "> report " << static_cast<int>(feature) << " at " << fast_interval_us_
                        << " us not enabled (" << why << "), back to " << DEFAULT_REPORT_INTERVAL
                        << " us for all reports");
    std::vector<uint8_t> again = fast_done_;
    again.push_back(feature);
    for (uint8_t done : again) {
      bno_->enableFeature(done);
      supervisor_.beat(init_limit_s_);
    }
  }

  double backoffS() const {
    if (failures_ <= 1) return 0.0;
    return std::min(std::pow(2.0, static_cast<double>(failures_ - 2)), backoff_max_s_);
  }

  std::string reason() {
    std::lock_guard<std::mutex> lock(state_m_);
    return reason_;
  }

  void setReason(const std::string& reason) {
    std::lock_guard<std::mutex> lock(state_m_);
    reason_ = reason;
  }

  void setState(const std::string& state, const std::string& reason = "") {
    {
      std::lock_guard<std::mutex> lock(state_m_);
      state_ = state;
      reason_ = reason;
    }
    publishStatus();
  }

  void setMagUncalState(const char* state) {
    std::lock_guard<std::mutex> lock(state_m_);
    mag_uncal_state_ = state;
  }

  // self.bno as the status sees it: None, or the sensor object of this try.
  void showBno(const std::shared_ptr<Telemetry>& tele) {
    std::lock_guard<std::mutex> lock(state_m_);
    shown_ = tele;
  }

  // Runs in the main thread (set_state) and in the supervisor thread (1 Hz).
  void publishStatus() {
    std::lock_guard<std::mutex> pub_lock(publish_m_);
    std::string state, reason, mag_uncal_state, fast_state;
    unsigned long fast_granted_us;
    long failures, attempt;
    double sensor_reset_t;
    std::shared_ptr<Telemetry> tele;
    {
      std::lock_guard<std::mutex> lock(state_m_);
      state = state_;
      reason = reason_;
      mag_uncal_state = mag_uncal_state_;
      failures = failures_;
      attempt = attempt_;
      sensor_reset_t = sensor_reset_t_;
      tele = shown_;
      fast_state = fast_state_;
      fast_granted_us = fast_granted_us_;
    }
    bool has_age = false, has_uncal = false;
    double age = 0.0, bias[3] = {0.0, 0.0, 0.0};
    int frame_errors = 0, mag_accuracy = 0, uncal_accuracy = 0;
    if (tele) {
      std::lock_guard<std::mutex> lock(tele->m);
      has_age = tele->has_report_t;
      if (has_age) age = monotonic() - tele->last_report_t;
      frame_errors = tele->frame_errors;
      mag_accuracy = tele->mag_accuracy;
      has_uncal = tele->has_uncal;
      uncal_accuracy = tele->mag_uncal_accuracy;
      for (int i = 0; i < 3; ++i) bias[i] = tele->mag_bias[i];
    }
    const bool fresh = state == "ok" && has_age && age <= stale_s_;
    vitulus_msgs::Imu_status status;
    status.status = fresh;
    status.header.stamp = ros::Time::now();
    imu_status_pub_.publish(status);

    std::string json = "{\"state\": " + jsonString(state) + ", \"reason\": " + jsonString(reason) +
                       ", \"ok\": " + (fresh ? "true" : "false") +
                       fmt(", \"failures\": %ld, \"attempt\": %ld", failures, attempt) +
                       ", \"packet_age_s\": " + (has_age ? pyRound2(age) : std::string("null")) +
                       fmt(", \"frame_errors\": %d", frame_errors) +
                       ", \"sensor_reset_t\": " + pyFloat(sensor_reset_t) +
                       fmt(", \"pid\": %d", static_cast<int>(getpid())) +
                       ", \"mag_accuracy\": " + (tele ? fmt("%d", mag_accuracy) : std::string("null")) +
                       ", \"mag_uncal\": " + jsonString(mag_uncal_state) +
                       ", \"mag_uncal_accuracy\": " +
                       (tele && has_uncal ? fmt("%d", uncal_accuracy) : std::string("null")) +
                       ", \"mag_bias_uT\": ";
    if (tele && has_uncal)
      json += "[" + pyRound2(bias[0]) + ", " + pyRound2(bias[1]) + ", " + pyRound2(bias[2]) + "]";
    else
      json += "null";
    if (fast_interval_us_ != static_cast<long>(DEFAULT_REPORT_INTERVAL))
      json += ", \"fast_report\": " + jsonString(fast_state) +
              fmt(", \"fast_report_interval_us\": %lu", fast_granted_us);
    json += "}";
    std_msgs::String health;
    health.data = json;
    health_pub_.publish(health);
  }

  // A pause that a shutdown ends at once.
  void wait(double seconds) {
    const double end = monotonic() + seconds;
    while (monotonic() < end && !isShutdown()) {
      supervisor_.beat(init_limit_s_);
      sleepS(0.1);
    }
  }

  void closeUart() {
    if (uart_) {
      uart_->close();
      uart_.reset();
    }
  }

  void connectOnce() {
    ROS_INFO_STREAM("<" << name_ << "> Connecting UART " << port_ << ", baudrate " << baud_ << "...");
    bno_.reset();  // it refers to the port that goes away now
    uart_.reset(new GuardedUart(port_, baud_, read_timeout_s_, isShutdown));
    std::unique_ptr<BoundedBNO> bno(new BoundedBNO(*uart_, advert_timeout_s_, isShutdown));
    bno->initialize();
    bno_ = std::move(bno);
    showBno(bno_->tele);
    imu_sent_ = 0;
    imu_new_t_ = monotonic();
    fast_done_.clear();
    fast_on_ = fast_interval_us_ != static_cast<long>(DEFAULT_REPORT_INTERVAL);
    {
      std::lock_guard<std::mutex> lock(state_m_);
      fast_granted_us_ = DEFAULT_REPORT_INTERVAL;
      fast_state_ = fast_on_ ? "on" : "pending";
    }
    for (uint8_t feature : {REPORT_ACCELEROMETER, REPORT_GYROSCOPE, REPORT_MAGNETOMETER,
                            REPORT_ROTATION_VECTOR, REPORT_LINEAR_ACCELERATION,
                            REPORT_GEOMAGNETIC_ROTATION_VECTOR, REPORT_GAME_ROTATION_VECTOR}) {
      const bool fast = feature == REPORT_ACCELEROMETER || feature == REPORT_GYROSCOPE ||
                        feature == REPORT_GAME_ROTATION_VECTOR;
      if (fast_on_ && fast)
        enableFast(feature);
      else
        bno_->enableFeature(feature);
      supervisor_.beat(init_limit_s_);
    }
    {
      std::lock_guard<std::mutex> lock(state_m_);
      sensor_reset_t_ = wallTime();
    }
    enableMagUncal();
    ROS_INFO_STREAM("<" << name_ << "> UART connected.");
  }

  void warnMagUncal(const std::string& text) {
    if (!mag_uncal_warned_) {
      mag_uncal_warned_ = true;
      ROS_WARN_STREAM("<" << name_ << "> " << text);
    }
  }

  // Report 0x0F, optional: a refusal costs ~2 s per try, never the connection.
  void enableMagUncal() {
    if (!want_mag_uncal_) {
      setMagUncalState("off");
      return;
    }
    std::string error;
    mag_uncal_sent_ = 0;
    for (int i = 0; i < 2; ++i) {
      try {
        bno_->enableFeature(REPORT_MAG_UNCAL);
        setMagUncalState("no data");
        mag_uncal_enabled_t_ = monotonic();
        supervisor_.beat(init_limit_s_);
        return;
      } catch (const PyError& e) {
        supervisor_.beat(init_limit_s_);
        if (e.kind != Err::Runtime && e.kind != Err::Packet && e.kind != Err::Index && e.kind != Err::Key)
          throw;
        error = e.msg;
      }
    }
    setMagUncalState("unavailable");
    warnMagUncal("magnetometer report 0x0F not enabled (" + error + "), /bno085/mag_uncal off");
  }

  // True when the sensor streams; false after the tries or at a shutdown.
  bool connect() {
    const double pause = backoffS();
    if (pause > 0) {
      setState(failures_ >= max_failures_ ? "failed" : "backoff",
               fmt("waiting %.0f s, %ld failures", pause, failures_));
      wait(pause);
    }
    for (long attempt = 1; attempt <= connect_attempts_; ++attempt) {
      {
        std::lock_guard<std::mutex> lock(state_m_);
        attempt_ = attempt;
      }
      if (isShutdown()) return false;
      setState("connecting");
      supervisor_.beat(init_limit_s_);
      try {
        connectOnce();
        setState("ok");
        return true;
      } catch (const PyError& e) {
        if (e.kind == Err::ShutdownRequested) return false;
        if (e.kind == Err::Key) throw;  // not caught in Python either: the process ends, exit 1
        const std::string reason = e.name + ": " + e.msg;
        setReason(reason);
        ROS_WARN_STREAM("<" << name_ << "> connect try " << attempt << "/" << connect_attempts_
                            << " failed: " << reason);
      }
      showBno(nullptr);
      bno_.reset();
      closeUart();
      wait(0.3);
    }
    return false;
  }

  void storeFailures() {
    try {
      ros::param::set("~failures", static_cast<int>(failures_));
    } catch (...) {  // only the pause gets shorter
    }
  }

  void noteFailure(std::string state, const std::string& reason) {
    {
      std::lock_guard<std::mutex> lock(state_m_);
      failures_ += 1;
    }
    storeFailures();
    if (failures_ >= max_failures_) state = "failed";
    if (failures_ == max_failures_)
      ROS_ERROR_STREAM("<" << name_ << "> IMU failed " << failures_ << " times in a row: " << reason);
    else
      ROS_WARN_STREAM("<" << name_ << "> IMU " << state << ": " << reason << " (failure " << failures_ << ")");
    setState(state, reason);
  }

  void clearFailures() {
    {
      std::lock_guard<std::mutex> lock(state_m_);
      failures_ = 0;
    }
    storeFailures();
  }

  // Returns the exit code.
  int spin() {
    ros::Rate r(100);  // 100Hz
    ROS_INFO_STREAM("<" << name_ << "> Initialised.");
    const double started = monotonic();
    int last_errors = 0;
    double age = 0.0;
    while (!isShutdown()) {
      supervisor_.beat(run_limit_s_);
      try {
        bno_->poll();
        if (bno_->reportAge(&age) && age <= stale_s_) {
          publishMag();
          publishImu();
          publishImuHeading();
          publishMagUncal();
        }
      } catch (const PyError& e) {
        if (e.kind == Err::Serial || e.kind == Err::OS) {
          noteFailure("stalled", "port lost: " + e.msg);
          return EXIT_PORT;
        }
        bno_->addFrameError();  // damaged frame: resync, keep the yaw origin
      }

      if (!bno_->reportAge(&age)) age = monotonic() - started;
      if (age > stall_timeout_s_) {
        noteFailure("stalled", fmt("no sensor packets for %.1f s", age));
        return EXIT_STALL;
      }
      if (failures_ && monotonic() - started > stable_s_) clearFailures();
      if (bno_->frameErrors() != last_errors) {
        last_errors = bno_->frameErrors();
        ROS_WARN_STREAM_THROTTLE(30.0, "<" << name_ << "> " << last_errors
                                           << " damaged frames so far, resynced");
      }
      r.sleep();
    }
    return EXIT_OK;
  }

  static void fillCovariance(boost::array<double, 9>& c, double v) {
    c = {{v, 0.0, 0.0, 0.0, v, 0.0, 0.0, 0.0, v}};
  }

  // publish_imu and publish_imu_heading: the same message but for the
  // orientation source and the linear acceleration covariance.
  void publishImuMsg(ros::Publisher& pub, bool game, double acc_cov) {
    const double* a = bno_->acceleration();
    const double acc[3] = {a[0], a[1], a[2]};
    const double* g = bno_->gyro();
    const double gyro[3] = {g[0], g[1], g[2]};
    const double* q = game ? bno_->gameQuaternion() : bno_->quaternion();
    sensor_msgs::Imu msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "bno_imu_link";
    msg.orientation.x = q[0];
    msg.orientation.y = q[1];
    msg.orientation.z = q[2];
    msg.orientation.w = q[3];
    fillCovariance(msg.orientation_covariance, 0.025);
    msg.angular_velocity.x = gyro[0];
    msg.angular_velocity.y = gyro[1];
    msg.angular_velocity.z = gyro[2];
    fillCovariance(msg.angular_velocity_covariance, 0.025);
    msg.linear_acceleration.x = acc[0];
    msg.linear_acceleration.y = acc[1];
    msg.linear_acceleration.z = acc[2];
    fillCovariance(msg.linear_acceleration_covariance, acc_cov);
    if (game && fast_on_) {  // one message per new game rotation vector
      const double now = monotonic();
      if (bno_->game_count != imu_sent_) {
        imu_sent_ = bno_->game_count;
        imu_new_t_ = now;
      } else if (now - imu_new_t_ <= FAST_REPEAT_AFTER_S) {
        return;
      } else {  // the report stopped: go on as with the default interval
        ROS_WARN_STREAM_THROTTLE(30.0, "<" << name_ << "> no game rotation vector for "
                                           << fmt("%.2f", now - imu_new_t_)
                                           << " s, /bno085/imu repeats the last orientation");
      }
    }
    if ((q[0] + q[1] + q[2] + q[3]) != 0.0) pub.publish(msg);
  }

  void publishImu() { publishImuMsg(imu_pub_, true, 0.025); }

  void publishImuHeading() { publishImuMsg(navheading_pub_, false, 1000.0); }

  void publishMagMsg(ros::Publisher& pub, const double* ut) {
    sensor_msgs::MagneticField msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "bno_imu_link";
    msg.magnetic_field.x = ut[0] * 0.000001;
    msg.magnetic_field.y = ut[1] * 0.000001;
    msg.magnetic_field.z = ut[2] * 0.000001;
    fillCovariance(msg.magnetic_field_covariance, 0.025);
    pub.publish(msg);
  }

  void publishMag() { publishMagMsg(mag_pub_, bno_->magnetic()); }

  void publishMagUncal() {
    if (mag_uncal_state_ != "on" && mag_uncal_state_ != "no data") return;
    const double now = monotonic();
    if (bno_->mag_uncal_count == mag_uncal_sent_) {
      if (mag_uncal_state_ == "on" && now - mag_uncal_last_t_ > mag_uncal_stale_s_) {
        setMagUncalState("no data");  // 0x0F stopped while the rest streams
        mag_uncal_enabled_t_ = now;
      } else if (mag_uncal_state_ == "no data" && now - mag_uncal_enabled_t_ > mag_uncal_warn_s_) {
        warnMagUncal(fmt("magnetometer report 0x0F enabled but no data for %.0f s", mag_uncal_warn_s_));
      }
      return;
    }
    mag_uncal_sent_ = bno_->mag_uncal_count;
    mag_uncal_last_t_ = now;
    if (mag_uncal_state_ != "on") setMagUncalState("on");
    publishMagMsg(mag_uncal_pub_, bno_->mag_uncal);
  }

  ros::NodeHandle nh_;
  std::string name_;
  std::string port_;
  long baud_ = 3000000;
  double read_timeout_s_, advert_timeout_s_, init_limit_s_, run_limit_s_, stale_s_, stall_timeout_s_;
  double stable_s_, backoff_max_s_, mag_uncal_warn_s_, mag_uncal_stale_s_;
  long connect_attempts_, max_failures_;
  bool want_mag_uncal_;

  ros::Publisher imu_pub_, navheading_pub_, mag_pub_, mag_uncal_pub_, imu_status_pub_, health_pub_;
  // header.seq: roscpp numbers the messages of a topic itself, from 0 (rospy from 1).

  std::unique_ptr<GuardedUart> uart_;
  std::unique_ptr<BoundedBNO> bno_;

  std::mutex publish_m_;
  std::mutex state_m_;  // what publishStatus reads from the other thread:
  std::string state_ = "connecting";
  std::string reason_;
  std::string mag_uncal_state_ = "off";  // on | no data | unavailable | off
  long failures_ = 0;
  long attempt_ = 0;
  double sensor_reset_t_ = 0.0;
  std::shared_ptr<Telemetry> shown_;

  long fast_interval_us_ = DEFAULT_REPORT_INTERVAL;  // ~fast_report_interval_us
  bool fast_on_ = false;                             // the fast reports run at it
  std::string fast_state_ = "pending";               // pending | on | fallback (state_m_)
  unsigned long fast_granted_us_ = DEFAULT_REPORT_INTERVAL;  // (state_m_)
  std::vector<uint8_t> fast_done_;
  long imu_sent_ = 0;
  double imu_new_t_ = 0.0;

  bool mag_uncal_warned_ = false;
  long mag_uncal_sent_ = 0;
  double mag_uncal_enabled_t_ = 0.0;
  double mag_uncal_last_t_ = 0.0;
  Supervisor supervisor_;
};

}  // namespace bno085

int main(int argc, char** argv) {
  // rospy ends on SIGINT and SIGTERM alike, with exit 0.
  ros::init(argc, argv, "bno085_node", ros::init_options::NoSigintHandler);
  int code = bno085::EXIT_CONNECT;
  {
    ros::NodeHandle nh;
    signal(SIGINT, bno085::onSignal);
    signal(SIGTERM, bno085::onSignal);
    ROS_INFO_STREAM("<" << ros::this_node::getName() << "> Initialising bno085_node...");
    static bno085::BNO085Node* node = new bno085::BNO085Node();  // outlives main: the supervisor uses it
    try {
      code = node->run();
    } catch (const bno085::PyError& e) {
      ROS_FATAL_STREAM("<" << ros::this_node::getName() << "> " << e.name << ": " << e.msg);
      code = bno085::EXIT_CONNECT;
    }
    // rospy writes a message to the socket inside publish(); roscpp hands it to
    // another thread.  Give the last health message time to leave.
    if (code != bno085::EXIT_OK) bno085::sleepS(0.2);
  }
  ros::shutdown();
  fflush(nullptr);
  _exit(code);
}
