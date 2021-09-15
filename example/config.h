#pragma once
#include <map>
#include <string>
#include <iostream>
#include <vs_common/vs_vio_data_loader.h>
#include "core/VioManager.h"

static ov_msckf::VioManagerOptions paramEuroc();
static ov_msckf::VioManagerOptions paramTumVio();
static ov_msckf::VioManagerOptions paramZjuVislam();
static ov_msckf::VioManagerOptions paramUzhVio();
static ov_msckf::VioManagerOptions paramDefault();

static double getStartTsByPath(const std::string& datadir) {
  // start from stationary thus makesure msckf init success
  // Note: this start ts is used only for euroc zip data, NOT for rosbag
  const static std::map<std::string, double> start_ts_dict = {
      {"MH_01_easy", 1403636616}, {"MH_02_easy", 1403636886}, {"MH_03_medium", 0},   {"MH_04_difficult", 1403638143},
      {"MH_05_difficult", 0},     {"V1_01_easy", 0},          {"V1_02_medium", 0},   {"V1_03_difficult", 0},
      {"V2_01_easy", 0},          {"V2_02_medium", 0},        {"V2_03_difficult", 0}};
  for (const auto& it : start_ts_dict) {
    if (datadir.find(it.first) != datadir.npos) {
      return it.second;
    }
  }
  return 0;
}

static vs::vio::DatasetType getDatasetTypeByPath(const std::string& datadir) {
  const static std::vector<std::pair<std::vector<std::string>, vs::vio::DatasetType>> table = {
      // EuRoC dataset
      {{"MH_01_easy", "MH_02_easy", "MH_03_medium", "MH_04_difficult", "MH_05_difficult", "V1_01_easy", "V1_02_medium",
        "V1_03_difficult", "V2_01_easy", "V2_02_medium", "V2_03_difficult"},
       vs::vio::DATASET_EUROC},
      //  ZJU_SenseTime VISLAM dataset
      {{"C0_train", "C1_train", "C2_train",  "C3_train",  "C4_train", "C5_train", "C6_train", "C7_train",
        "C8_train", "C9_train", "C10_train", "C11_train", "D0_train", "D1_train", "D2_train", "D3_train",
        "D4_train", "D5_train", "D6_train",  "D7_train",  "D8_train", "D9_train", "D10_train"},
       vs::vio::DATASET_ZJU_VIO},
      // TUM VIO dataset
      {{"dataset-corridor1_512_16",   "dataset-corridor2_512_16",   "dataset-corridor3_512_16",
        "dataset-corridor4_512_16",   "dataset-corridor5_512_16",   "dataset-magistrale1_512_16",
        "dataset-magistrale2_512_16", "dataset-magistrale3_512_16", "dataset-magistrale4_512_16",
        "dataset-magistrale5_512_16", "dataset-magistrale6_512_16", "dataset-outdoors1_512_16",
        "dataset-outdoors2_512_16",   "dataset-outdoors3_512_16",   "dataset-outdoors4_512_16",
        "dataset-outdoors5_512_16",   "dataset-outdoors6_512_16",   "dataset-outdoors7_512_16",
        "dataset-outdoors8_512_16",   "dataset-room1_512_16",       "dataset-room2_512_16",
        "dataset-room3_512_16",       "dataset-room4_512_16",       "dataset-room5_512_16",
        "dataset-room6_512_16",       "dataset-slides1_512_16",     "dataset-slides2_512_16",
        "dataset-slides3_512_16"},
       vs::vio::DATASET_TUM_VIO},
      // UZH VIO dataset
      {{"outdoor_forward_9_snapdragon"}, vs::vio::DATASET_UZH_VIO}
      // other
  };
  for (const auto& it : table) {
    for (const auto& s : it.first) {
      if (datadir.find(s) != datadir.npos) return it.second;
    }
  }
  return vs::vio::DATASET_UNKNOW;
}

struct RunHandler {
  ov_msckf::VioManagerOptions params;
  std::shared_ptr<vs::vio::VioDataLoader> dataset;
  std::shared_ptr<ov_msckf::VioManager> sys;

  bool init(const std::string& datadir, bool mono = false) {
    double start_ts = getStartTsByPath(datadir);
    auto dataset_type = getDatasetTypeByPath(datadir);
    dataset = vs::vio::createVioDataLoader(dataset_type);
    dataset->setMonoMode(mono);
    dataset->setStartTs(start_ts);
    bool ok = dataset->init({datadir});
    if (!ok) {
      printf("[ERROR]Dataset init failed\n");
      return false;
    }

    switch (dataset_type) {
      case vs::vio::DATASET_EUROC:
        params = paramEuroc();
        break;
      case vs::vio::DATASET_ZJU_VIO:
        params = paramZjuVislam();
        break;
      case vs::vio::DATASET_TUM_VIO:
        params = paramTumVio();
        break;
      case vs::vio::DATASET_UZH_VIO:
        params = paramUzhVio();
        break;
      default:
        printf("[ERROR]Unknown dataset.\n");
        return false;
    }

    sys = std::make_shared<ov_msckf::VioManager>(params);
    return true;
  }
};

inline std::vector<cv::Point3f> cvtPts(const std::vector<Eigen::Vector3d>& pts) {
  std::vector<cv::Point3f> res;
  if (pts.empty()) return res;
  res.reserve(pts.size());
  for (const auto& p : pts) res.push_back(cv::Point3f(p.x(), p.y(), p.z()));
  return res;
}

inline Eigen::Isometry3d vec2isom(const Eigen::Matrix<double, 7, 1>& v) {
  // cannot use Eigen quaternion, Eigen use Hamilton while OpenVINS use JPL
  // return vs::isom(v(0, 0), v(1, 0), v(2, 0), v(3, 0), v(4, 0), v(5, 0), v(6, 0));
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = ov_core::quat_2_Rot(v.topRows(4));
  T.translation() = v.bottomRows(3);
  return T;
}

inline cv::Vec2d calcFov(float fx, float fy, const cv::Size& img_size) {
  return cv::Vec2d(std::atan2(img_size.width / 2, fx) * 2, std::atan2(img_size.height / 2, fy) * 2);
}

ov_msckf::VioManagerOptions paramEuroc() {
  ov_msckf::VioManagerOptions params;
  params.use_stereo = false;
  params.state_options.do_fej = true;
  params.state_options.imu_avg = true;
  params.state_options.use_rk4_integration = true;
  params.state_options.do_calib_camera_pose = true;
  params.state_options.do_calib_camera_intrinsics = true;
  params.state_options.do_calib_camera_timeoffset = true;
  params.state_options.max_clone_size = 11;
  params.state_options.max_slam_features = 75;
  params.state_options.max_slam_in_update = 25;
  params.state_options.max_msckf_in_update = 40;
  params.state_options.num_cameras = 1;
  params.dt_slam_delay = 3;
  params.init_window_time = 0.75;
  params.init_imu_thresh = 1.5;
  params.gravity_mag = 9.81;

  params.state_options.feat_rep_msckf = LandmarkRepresentation::Representation::GLOBAL_3D;
  params.state_options.feat_rep_slam = LandmarkRepresentation::Representation::ANCHORED_FULL_INVERSE_DEPTH;
  params.state_options.feat_rep_aruco = LandmarkRepresentation::Representation::ANCHORED_FULL_INVERSE_DEPTH;

  params.try_zupt = true;
  params.zupt_options.chi2_multipler = 0;
  params.zupt_max_velocity = 0.1;
  params.zupt_noise_multiplier = 50;
  params.zupt_max_disparity = 0.5;
  params.zupt_only_at_beginning = false;

  params.record_timing_information = false;
  params.record_timing_filepath = "tmp/ov_timing.txt";

  params.use_klt = true;
  params.num_pts = 250;
  params.fast_threshold = 15;
  params.grid_x = 5;
  params.grid_y = 3;
  params.min_px_dist = 8;
  params.knn_ratio = 0.7;
  params.downsample_cameras = false;
  params.use_multi_threading = false;  // true;
  params.histogram_method = ov_core::TrackBase::HistogramMethod::HISTOGRAM;

  params.use_aruco = false;

  params.msckf_options.sigma_pix = 1;
  params.msckf_options.chi2_multipler = 1;
  params.slam_options.sigma_pix = 1;
  params.slam_options.chi2_multipler = 1;
  params.imu_noises.sigma_w = 1.6968e-4;
  params.imu_noises.sigma_a = 2.0000e-3;
  params.imu_noises.sigma_wb = 1.9393e-5;
  params.imu_noises.sigma_ab = 3.0000e-3;

  params.calib_camimu_dt = 0.0;

  if (1) {
    int id = 0;
    Eigen::Matrix<double, 8, 1> cam_calib;
    cam_calib << 458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05;
    Eigen::Matrix<double, 7, 1> cam_eigen;
    Eigen::Matrix4d T_CtoI;
    T_CtoI << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975, 0.999557249008, 0.0149672133247,
        0.025715529948, -0.064676986768, -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949, 0.0, 0.0,
        0.0, 1.0;
    cam_eigen.block(0, 0, 4, 1) = rot_2_quat(T_CtoI.block(0, 0, 3, 3).transpose());
    cam_eigen.block(4, 0, 3, 1) = -T_CtoI.block(0, 0, 3, 3).transpose() * T_CtoI.block(0, 3, 3, 1);
    params.camera_fisheye.insert({id, false});
    params.camera_intrinsics.insert({id, cam_calib});
    params.camera_extrinsics.insert({id, cam_eigen});
    params.camera_wh.insert({id, std::make_pair(752, 480)});
  }
  if (params.use_stereo) {
    int id = 1;
    Eigen::Matrix<double, 8, 1> cam_calib;
    cam_calib << 457.587, 456.134, 379.999, 255.238, -0.28368365, 0.07451284, -0.00010473, -3.55590700e-05;
    Eigen::Matrix<double, 7, 1> cam_eigen;
    Eigen::Matrix4d T_CtoI;
    T_CtoI << 0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556, 0.999598781151, 0.0130119051815,
        0.0251588363115, 0.0453689425024, -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038, 0.0, 0.0,
        0.0, 1.0;
    cam_eigen.block(0, 0, 4, 1) = rot_2_quat(T_CtoI.block(0, 0, 3, 3).transpose());
    cam_eigen.block(4, 0, 3, 1) = -T_CtoI.block(0, 0, 3, 3).transpose() * T_CtoI.block(0, 3, 3, 1);
    params.camera_fisheye.insert({id, false});
    params.camera_intrinsics.insert({id, cam_calib});
    params.camera_extrinsics.insert({id, cam_eigen});
    params.camera_wh.insert({id, std::make_pair(752, 480)});
  }
  return params;
}

static ov_msckf::VioManagerOptions paramTumVio() {
  ov_msckf::VioManagerOptions params;
  params.use_stereo = false;
  params.state_options.do_fej = true;
  params.state_options.imu_avg = true;
  params.state_options.use_rk4_integration = true;
  params.state_options.do_calib_camera_pose = true;
  params.state_options.do_calib_camera_intrinsics = true;
  params.state_options.do_calib_camera_timeoffset = true;
  params.state_options.max_clone_size = 11;
  params.state_options.max_slam_features = 50;
  params.state_options.max_slam_in_update = 25;
  params.state_options.max_msckf_in_update = 40;
  params.state_options.num_cameras = 1;
  params.dt_slam_delay = 5;
  params.init_window_time = 1.0;
  params.init_imu_thresh = 0.6;  // room1-5:0.60, room6:0.45
  params.gravity_mag = 9.80766;

  params.state_options.feat_rep_msckf = LandmarkRepresentation::Representation::GLOBAL_3D;
  params.state_options.feat_rep_slam = LandmarkRepresentation::Representation::ANCHORED_FULL_INVERSE_DEPTH;
  params.state_options.feat_rep_aruco = LandmarkRepresentation::Representation::ANCHORED_FULL_INVERSE_DEPTH;

  params.try_zupt = false;
  // params.zupt_options.chi2_multipler = 0;
  // params.zupt_max_velocity = 0.1;
  // params.zupt_noise_multiplier = 50;
  // params.zupt_max_disparity = 0.5;
  // params.zupt_only_at_beginning = false;

  params.record_timing_information = false;
  // params.record_timing_filepath = "tmp/ov_timing.txt";

  params.use_klt = true;
  params.num_pts = 300;
  params.fast_threshold = 15;
  params.grid_x = 5;
  params.grid_y = 5;
  params.min_px_dist = 5;
  params.knn_ratio = 0.65;
  params.downsample_cameras = false;
  params.use_multi_threading = false;  // true;
  params.histogram_method = ov_core::TrackBase::HistogramMethod::HISTOGRAM;

  params.use_aruco = false;

  params.msckf_options.sigma_pix = 1;
  params.msckf_options.chi2_multipler = 1;
  params.slam_options.sigma_pix = 1;
  params.slam_options.chi2_multipler = 1;
  params.imu_noises.sigma_w = 0.00016;
  params.imu_noises.sigma_a = 0.0028;
  params.imu_noises.sigma_wb = 0.000022;
  params.imu_noises.sigma_ab = 0.00086;

  params.calib_camimu_dt = 0.0;

  if (1) {
    int id = 0;
    Eigen::Matrix<double, 8, 1> cam_calib;
    cam_calib << 190.97847715128717, 190.9733070521226, 254.93170605935475, 256.8974428996504, 0.0034823894022493434,
        0.0007150348452162257, -0.0020532361418706202, 0.00020293673591811182;
    Eigen::Matrix<double, 7, 1> cam_eigen;
    Eigen::Matrix4d T_CtoI;
    T_CtoI << -0.99952504, 0.00750192, -0.02989013, 0.04557484, 0.02961534, -0.03439736, -0.99896935, -0.07116180,
        -0.00852233, -0.99938008, 0.03415885, -0.04468125, 0.0, 0.0, 0.0, 1.0;
    cam_eigen.block(0, 0, 4, 1) = rot_2_quat(T_CtoI.block(0, 0, 3, 3).transpose());
    cam_eigen.block(4, 0, 3, 1) = -T_CtoI.block(0, 0, 3, 3).transpose() * T_CtoI.block(0, 3, 3, 1);
    params.camera_fisheye.insert({id, true});
    params.camera_intrinsics.insert({id, cam_calib});
    params.camera_extrinsics.insert({id, cam_eigen});
    params.camera_wh.insert({id, std::make_pair(512, 512)});
  }
  if (params.use_stereo) {
    int id = 1;
    Eigen::Matrix<double, 8, 1> cam_calib;
    cam_calib << 190.44236969414825, 190.4344384721956, 252.59949716835982, 254.91723064636983, 0.0034003170790442797,
        0.001766278153469831, -0.00266312569781606, 0.0003299517423931039;
    Eigen::Matrix<double, 7, 1> cam_eigen;
    Eigen::Matrix4d T_CtoI;
    T_CtoI << -0.99951105, 0.00810408, -0.03019914, -0.05545634, 0.03029912, 0.01251164, -0.99946257, -0.06925002,
        -0.00772188, -0.99988889, -0.01275107, -0.04745286, 0.0, 0.0, 0.0, 1.0;
    cam_eigen.block(0, 0, 4, 1) = rot_2_quat(T_CtoI.block(0, 0, 3, 3).transpose());
    cam_eigen.block(4, 0, 3, 1) = -T_CtoI.block(0, 0, 3, 3).transpose() * T_CtoI.block(0, 3, 3, 1);
    params.camera_fisheye.insert({id, true});
    params.camera_intrinsics.insert({id, cam_calib});
    params.camera_extrinsics.insert({id, cam_eigen});
    params.camera_wh.insert({id, std::make_pair(512, 512)});
  }
  return params;
}

static ov_msckf::VioManagerOptions paramZjuVislam() {
  ov_msckf::VioManagerOptions params;
  params.use_stereo = false;
  params.state_options.do_fej = true;
  params.state_options.imu_avg = true;
  params.state_options.use_rk4_integration = true;
  params.state_options.do_calib_camera_pose = true;
  params.state_options.do_calib_camera_intrinsics = true;
  params.state_options.do_calib_camera_timeoffset = true;
  params.state_options.max_clone_size = 11;
  params.state_options.max_slam_features = 75;
  params.state_options.max_slam_in_update = 25;
  params.state_options.max_msckf_in_update = 40;
  params.state_options.num_cameras = 1;
  params.dt_slam_delay = 3;
  params.init_window_time = 0.75;
  params.init_imu_thresh = 1.5;
  params.gravity_mag = 9.81;

  params.state_options.feat_rep_msckf = LandmarkRepresentation::Representation::GLOBAL_3D;
  params.state_options.feat_rep_slam = LandmarkRepresentation::Representation::ANCHORED_FULL_INVERSE_DEPTH;
  params.state_options.feat_rep_aruco = LandmarkRepresentation::Representation::ANCHORED_FULL_INVERSE_DEPTH;

  params.try_zupt = true;
  params.zupt_options.chi2_multipler = 0;
  params.zupt_max_velocity = 0.1;
  params.zupt_noise_multiplier = 50;
  params.zupt_max_disparity = 0.5;
  params.zupt_only_at_beginning = false;

  params.record_timing_information = false;
  params.record_timing_filepath = "tmp/ov_timing.txt";

  params.use_klt = true;
  params.num_pts = 250;
  params.fast_threshold = 15;
  params.grid_x = 5;
  params.grid_y = 3;
  params.min_px_dist = 8;
  params.knn_ratio = 0.7;
  params.downsample_cameras = false;
  params.use_multi_threading = false;  // true;
  params.histogram_method = ov_core::TrackBase::HistogramMethod::HISTOGRAM;

  params.use_aruco = false;

  params.msckf_options.sigma_pix = 1;
  params.msckf_options.chi2_multipler = 1;
  params.slam_options.sigma_pix = 1;
  params.slam_options.chi2_multipler = 1;
  params.imu_noises.sigma_w = 1.6968e-4;
  params.imu_noises.sigma_a = 2.0000e-3;
  params.imu_noises.sigma_wb = 1.9393e-5;
  params.imu_noises.sigma_ab = 3.0000e-3;

  params.calib_camimu_dt = 0.0;

  if (1) {
    int id = 0;
    Eigen::Matrix<double, 8, 1> cam_calib;
    cam_calib << 458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05;
    Eigen::Matrix<double, 7, 1> cam_eigen;
    Eigen::Matrix4d T_CtoI;
    T_CtoI << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975, 0.999557249008, 0.0149672133247,
        0.025715529948, -0.064676986768, -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949, 0.0, 0.0,
        0.0, 1.0;
    cam_eigen.block(0, 0, 4, 1) = rot_2_quat(T_CtoI.block(0, 0, 3, 3).transpose());
    cam_eigen.block(4, 0, 3, 1) = -T_CtoI.block(0, 0, 3, 3).transpose() * T_CtoI.block(0, 3, 3, 1);
    params.camera_fisheye.insert({id, false});
    params.camera_intrinsics.insert({id, cam_calib});
    params.camera_extrinsics.insert({id, cam_eigen});
    params.camera_wh.insert({id, std::make_pair(752, 480)});
  }
  if (params.use_stereo) {
    int id = 1;
    Eigen::Matrix<double, 8, 1> cam_calib;
    cam_calib << 457.587, 456.134, 379.999, 255.238, -0.28368365, 0.07451284, -0.00010473, -3.55590700e-05;
    Eigen::Matrix<double, 7, 1> cam_eigen;
    Eigen::Matrix4d T_CtoI;
    T_CtoI << 0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556, 0.999598781151, 0.0130119051815,
        0.0251588363115, 0.0453689425024, -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038, 0.0, 0.0,
        0.0, 1.0;
    cam_eigen.block(0, 0, 4, 1) = rot_2_quat(T_CtoI.block(0, 0, 3, 3).transpose());
    cam_eigen.block(4, 0, 3, 1) = -T_CtoI.block(0, 0, 3, 3).transpose() * T_CtoI.block(0, 3, 3, 1);
    params.camera_fisheye.insert({id, false});
    params.camera_intrinsics.insert({id, cam_calib});
    params.camera_extrinsics.insert({id, cam_eigen});
    params.camera_wh.insert({id, std::make_pair(752, 480)});
  }
  return params;
}

static ov_msckf::VioManagerOptions paramUzhVio() {
  ov_msckf::VioManagerOptions params;
  params.use_stereo = false;
  params.state_options.do_fej = true;
  params.state_options.imu_avg = true;
  params.state_options.use_rk4_integration = true;
  params.state_options.do_calib_camera_pose = true;
  params.state_options.do_calib_camera_intrinsics = true;
  params.state_options.do_calib_camera_timeoffset = true;
  params.state_options.max_clone_size = 15;
  params.state_options.max_slam_features = 75;
  params.state_options.max_slam_in_update = 25;
  params.state_options.max_msckf_in_update = 40;
  params.state_options.num_cameras = 1;
  params.dt_slam_delay = 1;
  params.init_window_time = 1.0;
  params.init_imu_thresh = 0.5;
  params.gravity_mag = 9.81;

  params.state_options.feat_rep_msckf = LandmarkRepresentation::Representation::GLOBAL_3D;
  params.state_options.feat_rep_slam = LandmarkRepresentation::Representation::ANCHORED_FULL_INVERSE_DEPTH;
  params.state_options.feat_rep_aruco = LandmarkRepresentation::Representation::ANCHORED_FULL_INVERSE_DEPTH;

  params.try_zupt = true;
  params.zupt_options.chi2_multipler = 1;
  params.zupt_max_velocity = 0.5;
  params.zupt_noise_multiplier = 20;
  params.zupt_max_disparity = 0.4;
  params.zupt_only_at_beginning = false;

  params.record_timing_information = false;
  params.record_timing_filepath = "tmp/ov_timing.txt";

  params.use_klt = true;
  params.num_pts = 400;
  params.fast_threshold = 10;
  params.grid_x = 8;
  params.grid_y = 5;
  params.min_px_dist = 8;
  params.knn_ratio = 0.65;
  params.downsample_cameras = false;
  params.use_multi_threading = false;  // true;
  params.histogram_method = ov_core::TrackBase::HistogramMethod::HISTOGRAM;

  params.use_aruco = false;

  params.msckf_options.sigma_pix = 1;
  params.msckf_options.chi2_multipler = 1;
  params.slam_options.sigma_pix = 1;
  params.slam_options.chi2_multipler = 1;
  params.imu_noises.sigma_w = 1.6968e-4;
  params.imu_noises.sigma_a = 2.0000e-3;
  params.imu_noises.sigma_wb = 1.9393e-5;
  params.imu_noises.sigma_ab = 3.0000e-3;

  params.calib_camimu_dt = -0.015;

  if (1) {
    int id = 0;
    Eigen::Matrix<double, 8, 1> cam_calib;
    cam_calib << 278.66723066149086, 278.48991409740296, 319.75221200593535, 241.96858910358173, -0.013721808247486035,
        0.020727425669427896, -0.012786476702685545, 0.0025242267320687625;
    Eigen::Matrix<double, 7, 1> cam_eigen;
    Eigen::Matrix4d T_CtoI;
    T_CtoI << -0.02822879, 0.01440125, 0.99949774, 0.00110212, -0.99960149, -0.00041887, -0.02822568, 0.02170142,
        0.00001218, -0.99989621, 0.01440734, -0.00005928, 0.0, 0.0, 0.0, 1.0;
    cam_eigen.block(0, 0, 4, 1) = rot_2_quat(T_CtoI.block(0, 0, 3, 3).transpose());
    cam_eigen.block(4, 0, 3, 1) = -T_CtoI.block(0, 0, 3, 3).transpose() * T_CtoI.block(0, 3, 3, 1);
    params.camera_fisheye.insert({id, true});
    params.camera_intrinsics.insert({id, cam_calib});
    params.camera_extrinsics.insert({id, cam_eigen});
    params.camera_wh.insert({id, std::make_pair(640, 480)});
  }
  if (params.use_stereo) {
    int id = 1;
    Eigen::Matrix<double, 8, 1> cam_calib;
    cam_calib << 277.61640629770613, 277.63749695723294, 314.8944703346039, 236.04310050462587, -0.008456929295619607,
        0.011407590938612062, -0.006951788325762078, 0.0015368127092821786;
    Eigen::Matrix<double, 7, 1> cam_eigen;
    Eigen::Matrix4d T_CtoI;
    T_CtoI << -0.01182306, 0.01155299, 0.99986336, -0.00029028, -0.99987014, 0.01081377, -0.01194809, -0.05790695,
        -0.01095033, -0.99987479, 0.01142364, -0.0001919, 0.0, 0.0, 0.0, 1.0;
    cam_eigen.block(0, 0, 4, 1) = rot_2_quat(T_CtoI.block(0, 0, 3, 3).transpose());
    cam_eigen.block(4, 0, 3, 1) = -T_CtoI.block(0, 0, 3, 3).transpose() * T_CtoI.block(0, 3, 3, 1);
    params.camera_fisheye.insert({id, true});
    params.camera_intrinsics.insert({id, cam_calib});
    params.camera_extrinsics.insert({id, cam_eigen});
    params.camera_wh.insert({id, std::make_pair(640, 480)});
  }
  return params;
}

static ov_msckf::VioManagerOptions paramDefault() {
  ov_msckf::VioManagerOptions params;
  params.use_stereo = false;
  params.state_options.do_fej = true;
  params.state_options.imu_avg = true;
  params.state_options.use_rk4_integration = true;
  params.state_options.do_calib_camera_pose = true;
  params.state_options.do_calib_camera_intrinsics = true;
  params.state_options.do_calib_camera_timeoffset = true;
  params.state_options.max_clone_size = 11;
  params.state_options.max_slam_features = 75;
  params.state_options.max_slam_in_update = 25;
  params.state_options.max_msckf_in_update = 40;
  params.state_options.num_cameras = 1;
  params.dt_slam_delay = 3;
  params.init_window_time = 0.75;
  params.init_imu_thresh = 1.5;
  params.gravity_mag = 9.81;

  params.state_options.feat_rep_msckf = LandmarkRepresentation::Representation::GLOBAL_3D;
  params.state_options.feat_rep_slam = LandmarkRepresentation::Representation::ANCHORED_FULL_INVERSE_DEPTH;
  params.state_options.feat_rep_aruco = LandmarkRepresentation::Representation::ANCHORED_FULL_INVERSE_DEPTH;

  params.try_zupt = true;
  params.zupt_options.chi2_multipler = 0;
  params.zupt_max_velocity = 0.1;
  params.zupt_noise_multiplier = 50;
  params.zupt_max_disparity = 0.5;
  params.zupt_only_at_beginning = false;

  params.record_timing_information = false;
  params.record_timing_filepath = "tmp/ov_timing.txt";

  params.use_klt = true;
  params.num_pts = 250;
  params.fast_threshold = 15;
  params.grid_x = 5;
  params.grid_y = 3;
  params.min_px_dist = 8;
  params.knn_ratio = 0.7;
  params.downsample_cameras = false;
  params.use_multi_threading = false;  // true;
  params.histogram_method = ov_core::TrackBase::HistogramMethod::HISTOGRAM;

  params.use_aruco = false;

  params.msckf_options.sigma_pix = 1;
  params.msckf_options.chi2_multipler = 1;
  params.slam_options.sigma_pix = 1;
  params.slam_options.chi2_multipler = 1;
  params.imu_noises.sigma_w = 1.6968e-4;
  params.imu_noises.sigma_a = 2.0000e-3;
  params.imu_noises.sigma_wb = 1.9393e-5;
  params.imu_noises.sigma_ab = 3.0000e-3;

  params.calib_camimu_dt = 0.0;

  if (1) {
    int id = 0;
    Eigen::Matrix<double, 8, 1> cam_calib;
    cam_calib << 458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05;
    Eigen::Matrix<double, 7, 1> cam_eigen;
    Eigen::Matrix4d T_CtoI;
    T_CtoI << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975, 0.999557249008, 0.0149672133247,
        0.025715529948, -0.064676986768, -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949, 0.0, 0.0,
        0.0, 1.0;
    cam_eigen.block(0, 0, 4, 1) = rot_2_quat(T_CtoI.block(0, 0, 3, 3).transpose());
    cam_eigen.block(4, 0, 3, 1) = -T_CtoI.block(0, 0, 3, 3).transpose() * T_CtoI.block(0, 3, 3, 1);
    params.camera_fisheye.insert({id, false});
    params.camera_intrinsics.insert({id, cam_calib});
    params.camera_extrinsics.insert({id, cam_eigen});
    params.camera_wh.insert({id, std::make_pair(752, 480)});
  }
  if (params.use_stereo) {
    int id = 1;
    Eigen::Matrix<double, 8, 1> cam_calib;
    cam_calib << 457.587, 456.134, 379.999, 255.238, -0.28368365, 0.07451284, -0.00010473, -3.55590700e-05;
    Eigen::Matrix<double, 7, 1> cam_eigen;
    Eigen::Matrix4d T_CtoI;
    T_CtoI << 0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556, 0.999598781151, 0.0130119051815,
        0.0251588363115, 0.0453689425024, -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038, 0.0, 0.0,
        0.0, 1.0;
    cam_eigen.block(0, 0, 4, 1) = rot_2_quat(T_CtoI.block(0, 0, 3, 3).transpose());
    cam_eigen.block(4, 0, 3, 1) = -T_CtoI.block(0, 0, 3, 3).transpose() * T_CtoI.block(0, 3, 3, 1);
    params.camera_fisheye.insert({id, false});
    params.camera_intrinsics.insert({id, cam_calib});
    params.camera_extrinsics.insert({id, cam_eigen});
    params.camera_wh.insert({id, std::make_pair(752, 480)});
  }
  return params;
}