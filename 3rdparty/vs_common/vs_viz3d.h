/**
 * Copyright (c) 2019-2021 shuyuanmao <maoshuyuan123@gmail.com>. All rights reserved.
 * @Author: shuyuanmao
 * @EMail: maoshuyuan123@gmail.com
 * @Date: 2019-05-19 02:07
 * @Description:
 */
#pragma once
#include <map>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include <thread>
#include "vs_tictoc.h"

namespace vs {
class Viz3D {
 public:
  Viz3D() : m_stop(false), m_thread_ptr(new std::thread(std::bind(&Viz3D::run, this))) { m_viz.setBackgroundColor(); }

  ~Viz3D() {
    m_stop = true;
    m_thread_ptr->join();
    reset();
  }

  void reset() {
    m_viz.removeAllWidgets();
    m_widget_table.clear();
  }

  void updateWidget(const std::string& id, const cv::viz::Widget& w) { m_widget_table[id] = w; }

 private:
  bool m_stop;
  cv::viz::Viz3d m_viz;
  std::map<std::string, cv::viz::Widget> m_widget_table;
  std::shared_ptr<std::thread> m_thread_ptr;

  void run() {
    try {
      while (!m_viz.wasStopped() && !m_stop) {
        if (!m_widget_table.empty()) {
          for (const auto& m : m_widget_table) {
            m_viz.showWidget(m.first, m.second);
          }
          m_viz.spinOnce();
        }
        msleep(100);
      }
    } catch (...) {
      printf("[ERROR]:Viz3d thread quit expectedly.\n");
    }
  }
};

} /* namespace vs */