/**
 * Copyright (c) 2019-2021 shuyuanmao <maoshuyuan123@gmail.com>. All rights reserved.
 * @Author: shuyuanmao
 * @EMail: maoshuyuan123@gmail.com
 * @Date: 2019-05-19 02:07
 * @Description:
 */
#include "vs_tictoc.h"

#include <stdio.h>

#ifdef WIN32
#include <windows.h>
#else
#include <sys/time.h>
#include <unistd.h>
#endif

#include <map>
#include <string>

#ifdef WIN32
static int gettimeofday(struct timeval* tp, void* tzp) {
  time_t clock;
  struct tm tm;
  SYSTEMTIME wtm;
  GetLocalTime(&wtm);
  tm.tm_year = wtm.wYear - 1900;
  tm.tm_mon = wtm.wMonth - 1;
  tm.tm_mday = wtm.wDay;
  tm.tm_hour = wtm.wHour;
  tm.tm_min = wtm.wMinute;
  tm.tm_sec = wtm.wSecond;
  tm.tm_isdst = -1;
  clock = mktime(&tm);
  tp->tv_sec = clock;
  tp->tv_usec = wtm.wMilliseconds * 1000;
  return 0;
}
#endif

namespace vs {

const double Timer::ks =
    static_cast<double>(std::chrono::microseconds::period::num) / std::chrono::microseconds::period::den;
const double Timer::kms =
    static_cast<double>(std::chrono::microseconds::period::num) / std::chrono::microseconds::period::den * 1000.0;

static Timer p_program_timer;

double getSoftTs() {
  p_program_timer.stop();
  return p_program_timer.getSec();
}

double getSysTs() {
  struct timeval tv;
  if (gettimeofday(&tv, NULL) == 0) return tv.tv_sec + static_cast<double>(tv.tv_usec) * 1e-6;
  printf("[ERROR] getCurTs() failed.\n");
  return 0;
}

#ifdef WIN32
void msleep(int msec) { Sleep(msec); }
#else
void msleep(int msec) { usleep(msec * 1000); }
#endif

} /* namespace vs */

#define IMPL_CHRONO 1
// #define IMPL_OPENCV 1
// #define IMPL_STL 1

#ifdef IMPL_OPENCV
#include <opencv2/opencv.hpp>
namespace vs {

std::map<std::string, int64> g_start_tick;

void tic(const char* name) { g_start_tick[std::string(name)] = cv::getTickCount(); }

/*return time between tic and toc [MS]*/
float toc(const char* name) {
  int64 t = cv::getTickCount();
  std::string s(name);
  auto it = g_start_tick.find(s);
  if (it == g_start_tick.end()) {
    printf("[ERROR] toc(\"%s\"): should call tic first.\n", name);
    return -1;
  } else {
    return static_cast<float>(t - it->second) / cv::getTickFrequency() * 1000.0;
  }
}

void tictoc(const char* name) {
  int64 t = cv::getTickCount();
  std::string s(name);
  auto it = g_start_tick.find(s);
  if (it == g_start_tick.end()) {
    g_start_tick[s] = t;
  } else {
    float ms = static_cast<float>(t - it->second) / cv::getTickFrequency() * 1000.0;
    printf("tictoc: \"%s\" cost %.2f ms\n", name, ms);
    g_start_tick.erase(it);
  }
}

} /* namespace vs */
#elif IMPL_STL
#include <time.h>
namespace vs {

std::map<std::string, clock_t> g_start_tick;

void tic(const char* name) { g_start_tick[std::string(name)] = clock(); }

/*return time between tic and toc [MS]*/
float toc(const char* name) {
  clock_t t = clock();
  std::string s(name);
  auto it = g_start_tick.find(s);
  if (it == g_start_tick.end()) {
    printf("[ERROR] toc(\"%s\"): should call tic first.\n", name);
    return -1;
  } else {
    return static_cast<float>(t - it->second) / CLOCKS_PER_SEC * 1000.0;
  }
}

void tictoc(const char* name) {
  clock_t t = clock();
  std::string s(name);
  auto it = g_start_tick.find(s);
  if (it == g_start_tick.end()) {
    g_start_tick[s] = t;
  } else {
    float ms = static_cast<float>(t - it->second) / CLOCKS_PER_SEC * 1000.0;
    printf("tictoc: \"%s\" cost %.2f ms\n", name, ms);
    g_start_tick.erase(it);
  }
}

} /* namespace vs */
#elif IMPL_CHRONO
#include <chrono>
namespace vs {
std::map<std::string, std::chrono::system_clock::time_point> g_start_tick;

void tic(const char* name) { g_start_tick[std::string(name)] = std::chrono::system_clock::now(); }

/*return time between tic and toc [MS]*/
float toc(const char* name) {
  auto t = std::chrono::system_clock::now();
  std::string s(name);
  auto it = g_start_tick.find(s);
  if (it == g_start_tick.end()) {
    printf("[ERROR] toc(\"%s\"): should call tic first.\n", name);
    return -1;
  } else {
    return static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(t - it->second).count()) *
           std::chrono::microseconds::period::num / std::chrono::microseconds::period::den * 1000.0;
  }
}

void tictoc(const char* name) {
  auto t = std::chrono::system_clock::now();
  std::string s(name);
  auto it = g_start_tick.find(s);
  if (it == g_start_tick.end()) {
    g_start_tick[s] = t;
  } else {
    float ms = static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(t - it->second).count()) *
               std::chrono::microseconds::period::num / std::chrono::microseconds::period::den * 1000.0f;
    printf("tictoc: \"%s\" cost %.2f ms\n", name, ms);
    g_start_tick.erase(it);
  }
}

} /* namespace vs */
#endif
