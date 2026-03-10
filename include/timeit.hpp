#ifndef __TIMEIT_HPP // 修正宏命名，避免双下划线
#define __TIMEIT_HPP

#include <chrono>
#include <stdio.h>
#include <string>

class auto_time {
private:
  std::chrono::steady_clock::time_point start;
  int type; // 0:s, 1:ms, 2:us, 3:ns

public:
  auto_time(const std::string &unit = "ms")
      : start(std::chrono::steady_clock::now()) {
    if (unit == "s")
      type = 0;
    else if (unit == "ms")
      type = 1;
    else if (unit == "us")
      type = 2;
    else if (unit == "ns")
      type = 3;
    else {
      type = 1;
    }
  }
  ~auto_time() {
    auto duration = std::chrono::steady_clock::now() - start;
    double elapsed = 0.0;
    switch (type) {
    case 0: // 秒
      elapsed = std::chrono::duration<double>(duration).count();
      printf("Time elapsed: %.6f seconds\n", elapsed);
      break;
    case 1: // 毫秒
      elapsed = std::chrono::duration<double, std::milli>(duration).count();
      printf("Time elapsed: %.3f ms\n", elapsed);
      break;
    case 2: // 微秒
      elapsed = std::chrono::duration<double, std::micro>(duration).count();
      printf("Time elapsed: %.3f us\n", elapsed);
      break;
    case 3: // 纳秒
      elapsed = std::chrono::duration<double, std::nano>(duration).count();
      printf("Time elapsed: %.3f ns\n", elapsed);
      break;
    }
  }
};

class timeit {
public:
  std::chrono::steady_clock::time_point start_time;
  std::chrono::steady_clock::time_point end_time;
  void start() { start_time = std::chrono::steady_clock::now(); }
  void end() { end_time = std::chrono::steady_clock::now(); }
  void print(const std::string &unit = "ms") {
    auto duration = end_time - start_time;
    double elapsed = 0.0;
    if (unit == "s") {
      elapsed = std::chrono::duration<double>(duration).count();
      printf("Time elapsed: %.3f %s\n", elapsed, unit.c_str());
    }

    else if (unit == "ms") {
      elapsed = std::chrono::duration<double, std::milli>(duration).count();
      printf("Time elapsed: %.3f %s\n", elapsed, unit.c_str());
    } else if (unit == "us") {
      elapsed = std::chrono::duration<double, std::micro>(duration).count();
      printf("Time elapsed: %.3f %s\n", elapsed, unit.c_str());
    } else if (unit == "ns") {
      elapsed = std::chrono::duration<double, std::nano>(duration).count();
      printf("Time elapsed: %.3f %s\n", elapsed, unit.c_str());
    } else {
      elapsed = std::chrono::duration<double, std::milli>(duration).count();
      printf("Time elapsed: %.3f ms\n", elapsed);
    }
  }
};

#endif // TIMEIT_HPP