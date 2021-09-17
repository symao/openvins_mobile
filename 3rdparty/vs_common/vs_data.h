/**
 * Copyright (c) 2019-2021 shuyuanmao <maoshuyuan123@gmail.com>. All rights reserved.
 * @Author: shuyuanmao
 * @EMail: maoshuyuan123@gmail.com
 * @Date: 2019-05-19 02:07
 * @Description:
 */
#pragma once
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "vs_basic.h"
#include "vs_tictoc.h"

namespace vs {

template <class T>
class ElemBuffer {
 public:
  void set(const T& a) {
    m_obj = a;
    m_has = true;
  }

  T get() const { return m_has ? m_obj : T(); }

  bool has() const { return m_has; }

  void clear() { m_has = false; }

 private:
  T m_obj;
  bool m_has = false;
};

template <class T>
class DataBufferMutex {
 public:
  DataBufferMutex() : m_has(false) {}
  void set(const T& a) {
    m_mtx.lock();
    m_obj = a;
    m_has = true;
    m_mtx.unlock();
  }

  T get() const {
    m_mtx.lock();
    T res;
    if (m_has) res = m_obj;
    m_mtx.unlock();
    return res;
  }

  bool has() const {
    m_mtx.lock();
    bool res = m_has;
    m_mtx.unlock();
    return res;
  }

  void clear() {
    m_mtx.lock();
    m_has = false;
    m_mtx.unlock();
  }

  T getAndClear() {
    m_mtx.lock();
    T res;
    if (m_has) {
      res = m_obj;
      m_has = false;
    }
    m_mtx.unlock();
    return res;
  }

 private:
  T m_obj;
  bool m_has;
  mutable std::mutex m_mtx;
};

template <class T>
class DataSaver {
 public:
  typedef std::function<void(FILE* fp, const T& t)> WriteFunction;

  DataSaver(const char* save_file, WriteFunction write_function)
      : m_exit(false),
        m_out_file(save_file),
        m_write_fun(write_function),
        m_thread_ptr(new std::thread(std::bind(&DataSaver<T>::run, this))) {}

  ~DataSaver() {
    m_exit = true;
    join();
  }

  void push(const T& data) {
    m_mtx.lock();
    m_buffer.push_back(data);
    m_mtx.unlock();
  }

  void join() { m_thread_ptr->join(); }

 private:
  bool m_exit;
  std::string m_out_file;
  std::mutex m_mtx;
  std::deque<T> m_buffer;
  std::function<void(FILE* fp, const T& t)> m_write_fun;
  std::shared_ptr<std::thread> m_thread_ptr;

  void run() {
    FILE* fp = NULL;
    while (!m_exit) {
      if (!fp) {  // open file
        m_mtx.lock();
        bool has_data = !m_buffer.empty();
        m_mtx.unlock();
        if (has_data) {
          fp = fopen(m_out_file.c_str(), "w");
          if (!fp) {
            printf("[ERROR]DataSaver: Cannot open '%s', Exit.\n", m_out_file.c_str());
            m_exit = true;
          }
        }
      } else {  // write data
        m_mtx.lock();
        auto buffer = m_buffer;
        m_buffer.clear();
        m_mtx.unlock();
        if (!buffer.empty()) {
          for (const auto& t : buffer) {
            m_write_fun(fp, t);
          }
          fflush(fp);
        }
      }
      msleep(500);
    }
    if (fp) fclose(fp);
  }
};

template <class T>
class TimeBuffer {
 public:
  TimeBuffer() {
    foo_wsum_ = [](double k1, const T& a1, double k2, const T& a2) { return k1 * a1 + k2 * a2; };
  }

  explicit TimeBuffer(std::function<T(double, const T&, double, const T&)> f_weight_sum) { foo_wsum_ = f_weight_sum; }

  T front() { return buffer.front().second; }

  T back() { return buffer.back().second; }

  double frontTs() { return buffer.front().first; }

  double backTs() { return buffer.back().first; }

  bool empty() { return buffer.empty(); }

  void add(double ts, const T& a) { buffer.push_back(std::make_pair(ts, a)); }

  bool get(double ts, T& res) const {
    int nbuf = buffer.size();
    if (nbuf < 2 || ts < buffer[0].first || ts > buffer.back().first) return false;
    int left = 0, right = nbuf;
    while (left < right) {
      int mid = (left + right) / 2;
      const auto& b_mid = buffer[mid];
      if (fequal(b_mid.first, ts)) {
        res = b_mid.second;
        return true;
      } else if (b_mid.first < ts) {
        left = mid + 1;
      } else {
        right = mid;
      }
    }
    if (left != right || right == nbuf) {
      printf("[ERROR] l:%d != r:%d\n", left, right);
      return false;
    }
    const auto& a1 = buffer[left];
    if (fequal(a1.first, ts)) {
      res = a1.second;
      return true;
    }
    const auto& a2 = a1.first > ts ? buffer[left - 1] : buffer[left + 1];
    double dt = a2.first - a1.first;
    if (fequal(dt, 0)) return false;
    double k = (ts - a1.first) / dt;
    if (k < 0 || k > 1) return false;
    res = foo_wsum_(k, a2.second, (1 - k), a1.second);
    return true;
  }

  // return data in time duration (start_ts, end_ts]
  std::vector<T> getRange(double start_ts, double end_ts) {
    // TODO(symao): accelerate with binary search
    std::vector<T> res;
    for (const auto& item : buffer) {
      if (item.first <= start_ts)
        continue;
      else if (item.first > end_ts)
        break;
      res.push_back(item.second);
    }
    return res;
  }

  void dropOld(double old_ts) {
    if (buffer.empty()) return;
    auto it = buffer.begin();
    while (it->first < old_ts && it != buffer.end()) it++;
    buffer.erase(buffer.begin(), it);
  }

  std::deque<std::pair<double, T>> buffer;

 private:
  std::function<T(double, const T&, double, const T&)> foo_wsum_;
};

} /* namespace vs */

#ifdef HAVE_BOOST
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>

namespace vs {

template <class T>
class DataBufferRW {
 public:
  DataBufferRW() : m_has(false) {}
  void set(const T& a) {
    WriteLock w_lock(m_mtx);
    m_obj = a;
    m_has = true;
    w_lock.unlock();
  }

  T get() const {
    ReadLock r_lock(m_mtx);
    T res;
    if (m_has) res = m_obj;
    r_lock.unlock();
    return res;
  }

  bool has() const {
    ReadLock r_lock(m_mtx);
    bool res = m_has;
    r_lock.unlock();
    return res;
  }

  void clear() {
    WriteLock w_lock(m_mtx);
    m_has = false;
    w_lock.unlock();
  }

 private:
  typedef boost::shared_mutex Lock;
  typedef boost::unique_lock<Lock> WriteLock;
  typedef boost::shared_lock<Lock> ReadLock;

  T m_obj;
  bool m_has;
  mutable Lock m_mtx;
};

} /* namespace vs */

#endif  // HAVE_BOOST
