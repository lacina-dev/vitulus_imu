// Supervisor of bno085_node_cpp: the same watchdog as class Supervisor in
// nodes/bno085_node (Python).  A detached thread that ends the process with
// _exit when the main thread is stuck or does not obey a shutdown; on_tick runs
// about once a second.
#ifndef VITULUS_IMU_BNO085_SUPERVISOR_H
#define VITULUS_IMU_BNO085_SUPERVISOR_H

#include <time.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

namespace bno085 {

enum { EXIT_OK = 0, EXIT_CONNECT = 1, EXIT_STALL = 2, EXIT_PORT = 3, EXIT_HARD = 4 };

inline double monotonic() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

inline void sleepS(double seconds) {
  if (seconds <= 0.0) return;
  struct timespec ts;
  ts.tv_sec = static_cast<time_t>(seconds);
  ts.tv_nsec = static_cast<long>((seconds - static_cast<double>(ts.tv_sec)) * 1e9);
  while (clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, &ts) == EINTR) {
  }
}

class Supervisor {
 public:
  static constexpr double PERIOD_S = 0.2;

  Supervisor(std::function<bool()> is_shutdown, std::function<void()> on_tick,
             std::function<void(const std::string&)> log, double shutdown_grace_s = 2.0)
      : is_shutdown_(std::move(is_shutdown)), on_tick_(std::move(on_tick)), log_(std::move(log)),
        shutdown_grace_s_(shutdown_grace_s), beat_t_(monotonic()) {}

  void start() { std::thread(&Supervisor::run, this).detach(); }

  void beat(double limit_s) {
    std::lock_guard<std::mutex> lock(m_);
    limit_s_ = limit_s;
    beat_t_ = monotonic();
  }

  void leaveWithin(double seconds, int code) {
    std::lock_guard<std::mutex> lock(m_);
    exit_code_ = code;
    exit_t_ = monotonic() + seconds;
    has_exit_t_ = true;
  }

 private:
  void hardExit(int code, const std::string& why) {
    log_(why);
    fflush(nullptr);
    _exit(code);
  }

  void run() {
    double last = monotonic();
    double tick_t = 0.0;
    char text[96];
    while (true) {
      sleepS(PERIOD_S);
      const double now = monotonic();
      bool tick = false;
      {
        std::unique_lock<std::mutex> lock(m_);
        // We stalled too (memory pressure, CPU): the main thread gets a new
        // chance, otherwise a machine hiccup would reset the sensor.
        if (now - last > 1.0) beat_t_ += now - last;
        last = now;
        if (!has_exit_t_) {
          lock.unlock();
          const bool down = is_shutdown_();
          lock.lock();
          if (down && !has_exit_t_) {
            exit_code_ = EXIT_OK;
            exit_t_ = monotonic() + shutdown_grace_s_;
            has_exit_t_ = true;
          }
        }
        if (has_exit_t_) {
          if (now >= exit_t_) {
            const int code = exit_code_;
            lock.unlock();
            snprintf(text, sizeof(text), "bno085_node: forced exit %d", code);
            hardExit(code, text);
          }
        } else if (now - beat_t_ > limit_s_) {
          const double stuck = now - beat_t_;
          lock.unlock();
          snprintf(text, sizeof(text), "bno085_node: main thread stuck for %.0f s, hard exit", stuck);
          hardExit(EXIT_HARD, text);
        }
        if (now - tick_t >= 1.0) {
          tick_t = now;
          tick = true;
        }
      }
      if (tick) {
        try {
          on_tick_();
        } catch (...) {  // must never stop the watchdog
        }
      }
    }
  }

  std::function<bool()> is_shutdown_;
  std::function<void()> on_tick_;
  std::function<void(const std::string&)> log_;
  double shutdown_grace_s_;
  std::mutex m_;
  double beat_t_;
  double limit_s_ = 60.0;
  double exit_t_ = 0.0;
  bool has_exit_t_ = false;
  int exit_code_ = EXIT_OK;
};

}  // namespace bno085

#endif  // VITULUS_IMU_BNO085_SUPERVISOR_H
