/**
 * Copyright (c) 2019-2021 shuyuanmao <maoshuyuan123@gmail.com>. All rights reserved.
 * @Author: shuyuanmao
 * @EMail: maoshuyuan123@gmail.com
 * @Date: 2019-05-19 02:07
 * @Description:
 */
#pragma once
#include <memory>
#include <opencv2/opencv.hpp>

namespace vs {

namespace vio {

enum DatasetType {
  DATASET_UNKNOW = 0,
  DATASET_EUROC = 1,    ///< The EuRoC MAV Dataset:
                        ///< https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
  DATASET_KITTI = 2,    ///< The KITTI Vision Benchmark: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
  DATASET_UZH_VIO = 3,  ///< The UZH-FPV Drone Racing Dataset: https://fpv.ifi.uzh.ch/
  DATASET_TUM_VIO = 4,  ///< The TUM VI Benchmark: https://vision.in.tum.de/data/datasets/visual-inertial-dataset
  DATASET_ZJU_VIO = 5,  ///< ZJU-SenseTime VISLAM Benchmark: http://www.zjucvg.net/eval-vislam/
  DATASET_SYMAO = 99,   ///< Dataset format of shuyuanmao123@gmail.com
};

enum MessageType {
  MSG_NONE = 0,    ///< imu acc and gyro
  MSG_IMU = 1,     ///< imu acc and gyro
  MSG_CAMERA = 2,  ///< camera images
  MSG_GTPOSE = 3,  ///< ground-truth pose
};

struct ImuData {
  double ts = -1;
  double acc[3] = {0, 0, 0};
  double gyro[3] = {0, 0, 0};
};

struct CameraData {
  double ts = -1;
  std::vector<cv::Mat> imgs;
};

struct PoseData {
  double ts = -1;
  double qw, qx, qy, qz;
  double tx, ty, tz;
};

class VioDataLoader {
 public:
  VioDataLoader() : m_init(false), m_mono(false), m_start_ts(0), m_td(0), m_next_type(MSG_NONE) {}

  virtual ~VioDataLoader() {}

  bool init(const std::vector<std::string>& init_files) {
    m_init = initImpl(init_files);
    if (m_init) {
      readNextImu(m_next_imu);
      readNextCamera(m_next_cam);
      readNextGtPose(m_next_gt_pose);
      m_next_type = checkNextType();
    }
    return m_init;
  }

  MessageType nextType() { return m_next_type; }

  void setTd(double td) { m_td = td; }

  double td() const { return m_td; }

  void setStartTs(double ts) { m_start_ts = ts; }

  double startTs() const { return m_start_ts; }

  void setMonoMode(bool mono) { m_mono = mono; }

  bool mono() const { return m_mono; }

  bool fetchImu(ImuData& imu) {
    if (!m_init || (m_next_type != MSG_IMU && m_next_imu.ts < 0)) return false;
    imu = m_next_imu;
    readNextImu(m_next_imu);
    m_next_type = checkNextType();
    return true;
  }

  bool fetchCamera(CameraData& cam) {
    if (!m_init || (m_next_type != MSG_CAMERA && m_next_cam.ts < 0)) return false;
    cam = m_next_cam;
    readNextCamera(m_next_cam);
    m_next_cam.ts += m_td;  // handle time diff between camera and imu
    m_next_type = checkNextType();
    return true;
  }

  bool fetchGtPose(PoseData& gt_pose) {
    if (!m_init || (m_next_type != MSG_GTPOSE && m_next_gt_pose.ts < 0)) return false;
    gt_pose = m_next_gt_pose;
    readNextGtPose(m_next_gt_pose);
    m_next_type = checkNextType();
    return true;
  }

 protected:
  bool m_init;
  bool m_mono;        ///< whether use mono mode, otherwise use all cameras
  double m_start_ts;  ///< start timestamp in second
  double m_td;        ///< time diff between camera and imu
  MessageType m_next_type;
  ImuData m_next_imu;
  CameraData m_next_cam;
  PoseData m_next_gt_pose;

  virtual bool initImpl(const std::vector<std::string>& init_files) = 0;

  virtual bool readNextImu(ImuData& imu) = 0;

  virtual bool readNextCamera(CameraData& cam) = 0;

  virtual bool readNextGtPose(PoseData& gt_pose) = 0;

  MessageType checkNextType() {
    MessageType type = MSG_NONE;
    double min_ts = DBL_MAX;
    if (m_next_imu.ts > 0 && m_next_imu.ts < min_ts) {
      type = MSG_IMU;
      min_ts = m_next_imu.ts;
    }
    if (m_next_cam.ts > 0 && m_next_cam.ts < min_ts) {
      type = MSG_CAMERA;
      min_ts = m_next_cam.ts;
    }
    if (m_next_gt_pose.ts > 0 && m_next_gt_pose.ts < min_ts) {
      type = MSG_GTPOSE;
      min_ts = m_next_gt_pose.ts;
    }
    return type;
  }
};

std::shared_ptr<VioDataLoader> createVioDataLoader(DatasetType dataset_type);

}  // namespace vio

}  // namespace vs