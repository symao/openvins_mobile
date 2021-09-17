/**
 * Copyright (c) 2019-2021 shuyuanmao <maoshuyuan123@gmail.com>. All rights reserved.
 * @Author: shuyuanmao
 * @EMail: maoshuyuan123@gmail.com
 * @Date: 2019-05-19 02:07
 * @Description:
 */
#pragma once
#include <chrono>

namespace vs {

class Timer {
 public:
  Timer() { start(); }

  void start() { t1 = std::chrono::system_clock::now(); }

  double stop() {
    t2 = std::chrono::system_clock::now();
    return getMsec();
  }

  double getSec() { return std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() * ks; }

  double getMsec() { return std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() * kms; }

 private:
  std::chrono::system_clock::time_point t1, t2;
  const static double kms, ks;
};

void tictoc(const char* name);

void tic(const char* name);

/** @brief return time between tic and toc [MS] */
float toc(const char* name);

/** @brief sleep miliseconds */
void msleep(int msec);

/** @brief get software ts, reset to 0 when application start. [SEC] */
double getSoftTs();

/** @brief get system ts start from 1970-01-01 00:00:00. [SEC] */
double getSysTs();

} /* namespace vs */
