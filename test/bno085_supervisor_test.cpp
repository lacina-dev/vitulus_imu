// The watchdog of bno085_node_cpp alone, as the SUPERVISOR snippet of
// test_bno085_recovery.py does for the Python node:
//   bno085_supervisor_test stuck      main thread never beats again -> exit 4
//   bno085_supervisor_test shutdown   shutdown flag set, main does not leave -> exit 0
#include <atomic>
#include <cstdio>
#include <cstring>

#include "../src/bno085_supervisor.h"

int main(int argc, char** argv) {
  std::atomic<bool> flag(false);
  bno085::Supervisor s([&flag] { return flag.load(); }, [] {},
                       [](const std::string& t) { printf("%s\n", t.c_str()); }, 0.5);
  s.beat(1.0);
  s.start();
  if (argc > 1 && strcmp(argv[1], "shutdown") == 0) {
    s.beat(100.0);
    flag = true;
  }
  bno085::sleepS(30.0);
  return 9;
}
