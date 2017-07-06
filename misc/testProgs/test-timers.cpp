#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <signal.h>
#include <sys/time.h>
#include <thread>
#include <unistd.h>
#include <vector>

// Global structures to view elapsed time
static std::chrono::time_point<std::chrono::steady_clock> begin_, end_;
std::vector<double> diff_;
bool shutdown_ = false; // Flag to stop busy work thread
bool warned_ = false;

static uint32_t ROUNDS_1SEC = 200;  // Approximate measured busy loop rounds in 1 second

// Print a failure message with line number (needs at least 2 args)
#define FAIL(msg, ...) fail (__FILE__, __LINE__, false, msg, ##__VA_ARGS__)
#define WARN(msg, ...) fail (__FILE__, __LINE__, true, msg, ##__VA_ARGS__)
static void fail(const char *file, int line, bool warn, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    char *base_msg;
    if (vasprintf(&base_msg, fmt, ap) == -1) {
        fprintf(stdout, "[FAIL] Malloc failed.\n");
        exit(1);
    }
    if (warn) {
        warned_ = true;
        fprintf(stdout, "[WARN] (%s:%d): %s\n", file, line, base_msg);
    } else {
        fprintf(stdout, "[FAIL] (%s:%d): %s\n", file, line, base_msg);
    }
    free(base_msg);
    va_end(ap);
    if (!warn) {
        exit(1);
    }
}

// Verify that one one signal was received
static void checkOneSignal() {
  if (diff_.size() != 1) {
      FAIL("Expected to receive 1 signal (saw %ld)", diff_.size());
  }
}

// Convenience function to register signal handlers
static void Signal(int signum, void (*handler)(int)) {
    struct sigaction action;
    action.sa_handler = handler;
    sigemptyset(&action.sa_mask);
    action.sa_flags = SA_RESTART;
    if (sigaction(signum, &action, nullptr) < 0) {
        perror("Signal error");
        exit(1);
    }
}

// Returns true if the measured value is outside 1% of the expected
static bool notClose(double measured, double expected, double percent = 1.0) {
    if (measured < 0) {
      FAIL("Saw negative measured time: %f", measured);
    }
    if (expected < 0) {
      FAIL("Saw negative expected time: %f", expected);
    }
    double delta = percent * expected;
    if (delta < 0.000001) {
      // Handle expected == 0
      printf("Got small val: %f\n", delta);
      delta = 0.000001;
    }
    if ((measured >= (expected - delta)) && (measured <= (expected + delta))) {
        return false;
    }
    return true;
}

// Busy work. Purpose is to slow down a simulator to magnify timing differences,
// as well as test profiling timers (which count all active threads).
// (The actual algorithm is 'xor shift 128', a quick 32-bit PRNG)
// Run for 'rounds' iterations, or until 'shutdown_' if rounds is 0
void busyWork(uint32_t rounds) {
  uint32_t a, b, c, d;
  a = 0xdeafbeef;
  b = 0xc001cafe;
  c = 0x01234567;
  d = 0x89abcdef;
  bool forever = (rounds == 0);
  while (!shutdown_ && (forever || (rounds > 0))) {
      for (uint32_t i = 0; i < 1000000; i++) {
          uint32_t x = d;
          x ^= x << 11;
          x ^= x >> 8;
          d = c;
          c = b;
          b = a;
          x ^= a;
          x ^= a >> 19;
          a = x;
      }
      rounds--;
  }
}

// Estimate the number of busy work rounds in 1 real second
void benchmarkBusyWork() {
  begin_ = std::chrono::steady_clock::now();
  busyWork(200);
  end_ = std::chrono::steady_clock::now();
  double diff = std::chrono::duration<double>(end_ - begin_).count();
  double rounds = 200.0 / diff;
  ROUNDS_1SEC = static_cast<uint32_t>(rounds);
}

// Primary signal handler
static void handler(int sig) {
    switch (sig) {
      case SIGALRM:
      case SIGVTALRM:
      case SIGPROF:
          end_ = std::chrono::steady_clock::now();
          diff_.push_back(std::chrono::duration<double>(end_ - begin_).count());
          break;
      default:
          FAIL("Received unexpected signal %d", sig);
          break;
    }
}

// Helper function to set an interval timer and check for errors
static void Setitimer(int which, const struct itimerval *new_value, struct itimerval *old_value) {
    if (setitimer(which, new_value, old_value) != 0) {
        perror("setitimer()");
        exit(1);
    }
}

// Test a single alarm() call with spinning instead of sleeping
static void testAlarmSimple() {
    diff_.clear();
    alarm(1);
    begin_ = std::chrono::steady_clock::now();
    end_ = begin_;
    double diff = 0;
    while (diff < 1.1) {
        busyWork(10);
        end_ = std::chrono::steady_clock::now();
        diff = std::chrono::duration<double>(end_ - begin_).count();
    }
    if (diff_.empty()) {
        FAIL("Didn't see SIGALRM", "");
    }
    if (diff_.size() != 1) {
        FAIL("Expected to see 1 signal (saw %lu)", diff_.size());
    }
    if (notClose(diff_.front(), 1.0)) {
        FAIL("Expected 1 second (got %f)", diff_);
    }
    alarm(0);
}

// Test the functionality of alarm().
static void testAlarm() {
    unsigned int prior = alarm(10);
    if (prior != 0) {
        FAIL("Expected a disabled Alarm timer (got %u)", prior);
    }
    // Test 1: Sleep while waiting
    diff_.clear();
    prior = alarm(1);
    begin_ = std::chrono::steady_clock::now();
    if (notClose(prior, 10)) {
        FAIL("Expected 10 seconds (got %u)", prior);
    }
    sleep(2);
    if (diff_.empty()) {
        FAIL("Didn't see SIGALRM", "");
    }
    checkOneSignal();
    if (notClose(diff_.front(), 1.0)) {
        FAIL("Expected 1 second (got %f)", diff_);
    }
    // Test 2: Busy work while waiting
    diff_.clear();
    alarm(1);
    begin_ = std::chrono::steady_clock::now();
    busyWork(ROUNDS_1SEC + 20);
    if (diff_.empty()) {
        // It's possible the processor was too fast. However, if
        // increasing the amount of busy work doesn't help then the
        // test is a failure.
        WARN("Didn't see SIGALRM (try increasing iterations)", "");
    }
    checkOneSignal();
    if (notClose(diff_.front(), 1.0)) {
        FAIL("Expected 1 second (got %f)", diff_.front());
    }
}

// Test the functionality of reading the three interval timers
static void testGetItimer() {
    struct itimerval val, old, zero;
    val.it_interval.tv_sec = 5;
    val.it_interval.tv_usec = 0;
    val.it_value.tv_sec = 3;
    val.it_value.tv_usec = 0;
    memset(&zero, 0, sizeof(zero));
    int which = ITIMER_REAL;

    for (int i = 0; i < 3; i++) {
        diff_.clear();
        if (i == 1) {
            which = ITIMER_VIRTUAL;
        } else if (i == 2) {
            which = ITIMER_PROF;
        }
        Setitimer(which, &val, nullptr);
        Setitimer(which, &zero, &old);
        if (!diff_.empty()) {
            FAIL("Didn't expect timer signal to fire", "");
        }
        if (old.it_interval.tv_sec != 5) {
            FAIL("Expected original value 5 (got %u)", old.it_interval.tv_sec);
        }
        double remain = (double)old.it_value.tv_sec + ((double)old.it_value.tv_usec / 1e6);
        if (notClose(remain, 3.0)) {
            FAIL("Expected 3 seconds (got %f)", remain);
        }
    }
}

// Test the functionality of the real-time interval timer (ITIMER_REAL)
static void testSetReal() {
    struct itimerval val;
    val.it_interval.tv_sec = 2;
    val.it_interval.tv_usec = 0;
    val.it_value.tv_sec = 2;
    val.it_value.tv_usec = 0;
    Setitimer(ITIMER_REAL, &val, nullptr);

    for (int i = 0; i < 2; i++) {
        diff_.clear();
        begin_ = std::chrono::steady_clock::now();
        sleep(3);
        if (diff_.empty()) {
            FAIL("Didn't see SIGALRM", "");
        }
        checkOneSignal();
        if (notClose(diff_.front(), 2.0)) {
            FAIL("Expected 2 seconds (got %f)", diff_.front());
        }
        diff_.clear();
    }
    val.it_interval.tv_sec = 0;
    val.it_value.tv_sec = 0;
    Setitimer(ITIMER_REAL, &val, nullptr);
}

// Test the functionality of the virtual/profiling interval timers (ITIMER_VIRTUAL/ITIMER_PROF)
// In zsim we can't differentiate the kernel time very well anyway, so treat these as the same
//
// TODO: These tests may fail on a busy system since Virt/Prof time may be greater than wall time.
// The call to 'notClose()' should be replaced with something that is more relaxed, and checks
// e.g., >= expected wall time. Meanwhile we set a tolerance of 5% (instead of the default 1%).
static void testSetVirtualProf(int which) {
    struct itimerval val, zero;
    val.it_interval.tv_sec = 1;
    val.it_interval.tv_usec = 0;
    val.it_value.tv_sec = 1;
    val.it_value.tv_usec = 0;
    memset(&zero, 0, sizeof(zero));
    Setitimer(which, &val, nullptr);

    // Test 1: One active thread (the busy thread)
    diff_.clear();
    begin_ = std::chrono::steady_clock::now();
    sleep(4); // A reasonable system should have >= 2 signals
    Setitimer(which, &zero, nullptr);
    if (diff_.empty()) {
        FAIL("Didn't see SIGVTALRM/SIGPROF", "");
    }
    if (diff_.size() < 2) {
        FAIL("Expected 2 or more signals (saw %lu)", diff_.size());
    }
    double last = 0.0;
    for (double val : diff_) {
        if (notClose(val - last, 1.0, 5.0)) {
            FAIL("Expected 1 second (got %f)", val - last);
        }
        last = val;
    }

    // Test 2: Two active threads (signal triggers ~2x as fast)
    Setitimer(which, &val, nullptr);
    diff_.clear();
    begin_ = std::chrono::steady_clock::now();
    busyWork(ROUNDS_1SEC * 4);  // Expect >= 4 signals
    Setitimer(which, &zero, nullptr);
    if (diff_.empty()) {
        FAIL("Didn't see SIGVTALRM/SIGPROF", "");
    }
    if (diff_.size() < 4) {
        FAIL("Expected 4 or more signals (saw %lu)", diff_.size());
    }
    last = 0.0;
    for (double val : diff_) {
        if (notClose(val - last, 0.5, 5.0)) {
            FAIL("Expected 0.5 seconds (got %f)", val - last);
        }
        last = val;
    }
}

int main() {
    std::cout << "Running timer tests. This may take a while...\n";
    benchmarkBusyWork();
    Signal(SIGALRM, handler);
    Signal(SIGVTALRM, handler);
    Signal(SIGPROF, handler);
    std::thread worker(busyWork, 0);  // Run a continuous busy thread
    testAlarmSimple();
    testAlarm();
    testGetItimer();
    testSetReal();
    testSetVirtualProf(ITIMER_VIRTUAL);
    testSetVirtualProf(ITIMER_PROF);
    if (warned_) {
        std::cout << "One or more timer tests failed.\n";
    } else {
        std::cout << "Timer tests passed!\n";
    }
    shutdown_ = true;
    worker.join();
    return 0;
}
