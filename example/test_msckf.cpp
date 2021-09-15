#include <stdio.h>
#include <memory>
#include <vs_common/vs_common.h>
#include "core/VioManager.h"

#define ENABLE_VIZ 1

using namespace ov_msckf;

VioManagerOptions defaultParamEuroc();

std::map<std::string, double> start_ts_dict = {{"MH_01_easy", 0},
                                               {"MH_02_easy", 0},
                                               {"MH_03_medium", 0},
                                               {"MH_04_difficult", 0},
                                               {"MH_05_difficult", 3 + 1403638518.077829376},
                                               {"V1_01_easy", 5 + 1403715273.262142976},
                                               {"V1_02_medium", 10 + 1403715523.912143104},
                                               {"V1_03_difficult", 6 + 1403715886.584058112},
                                               {"V2_01_easy", 4 + 1413393212.255760384},
                                               {"V2_02_medium", 4 + 1413393886.005760512},
                                               {"V2_03_difficult", 5 + 1413394881.555760384}};

int main(int argc, char** argv) {
  auto params = defaultParamEuroc();
  std::shared_ptr<VioManager> sys = std::make_shared<VioManager>(params);

  double start_ts = 0;
  std::string datadir = argc > 1 ? argv[1] : "/home/symao/data/euroc/zip/MH_04_difficult";
  for (const auto& it : start_ts_dict) {
    if (vs::has(datadir, it.first)) {
      start_ts = it.second;
      break;
    }
  }
  auto dataset = vs::vio::createVioDataLoader(vs::vio::DATASET_EUROC_MONO);
  bool ok = dataset->init({datadir});
  //   auto dataset = vio::createVioDataLoader(vio::DATASET_SYMAO_STEREO);
  //   bool ok = dataset->init({"/media/symao/My Passport/data/VIO/sf_vio/AGV-西部兴围中转场/1/2019-08-16-14-56-30"});
  //   auto dataset = vio::createVioDataLoader(vio::DATASET_UZH_STEREO);
  //   bool ok = dataset->init({"/media/symao/My Passport/data/VIO/uzh_vio/zip/outdoor_forward_9_snapdragon"});
  //   auto dataset = vio::createVioDataLoader(vio::DATASET_ZJU_MONO);
  //   bool ok = dataset->init({"/media/symao/My Passport/data/VIO/zju_vislam/C0_train"});
  if (!ok) {
    printf("[ERROR]Dataset init failed\n");
    return 0;
  }

  const bool draw_img = true;
  vs::vio::ImuData cur_imu;
  vs::vio::CameraData cur_cam;
  vs::vio::PoseData cur_gt_pose;
  std::vector<cv::Affine3f> gt_poses;
  while (1) {
    ok = false;
    switch (dataset->nextType()) {
      case vs::vio::MSG_IMU:
        ok = dataset->fetchImu(cur_imu);
        if (cur_imu.ts < start_ts) break;
        // printf("IMU: %f gyro:(%.3f,%.3f,%.3f) acc:(%.3f,%.3f,%.3f)\n", cur_imu.ts, cur_imu.gyro[0], cur_imu.gyro[1],
        //        cur_imu.gyro[2], cur_imu.acc[0], cur_imu.acc[1], cur_imu.acc[2]);
        if (ok) {
          ov_core::ImuData msg;
          msg.timestamp = cur_imu.ts;
          msg.wm << cur_imu.gyro[0], cur_imu.gyro[1], cur_imu.gyro[2];
          msg.am << cur_imu.acc[0], cur_imu.acc[1], cur_imu.acc[2];
          sys->feed_measurement_imu(msg);
        }
        break;
      case vs::vio::MSG_CAMERA: {
        ok = dataset->fetchCamera(cur_cam);
        if (cur_cam.ts < start_ts) break;
        // printf("CAM: %f %d ", cur_cam.ts, static_cast<int>(cur_cam.imgs.size()));
        // for (const auto& img : cur_cam.imgs) printf(" [%dx%d,%d]", img.cols, img.rows, img.channels());
        // printf("\n");
        if (ok) {
          ov_core::CameraData msg;
          msg.timestamp = cur_cam.ts;
          msg.sensor_ids.push_back(0);
          msg.images.push_back(cur_cam.imgs[0].clone());
          msg.masks.push_back(cv::Mat::zeros(cur_cam.imgs[0].rows, cur_cam.imgs[0].cols, CV_8UC1));
          sys->feed_measurement_camera(msg);
        }
        auto img_show = sys->get_historical_viz_image();
        // auto img_show = cur_cam.imgs[0];
        if (!img_show.empty()) cv::imshow("image", img_show);
        if (sys->initialized()) {
          auto imu_state = sys->get_state()->_imu;
          Eigen::Isometry3d imu_pose =
              vs::isom(imu_state->quat()(3), imu_state->quat()(0), imu_state->quat()(1), imu_state->quat()(2),
                       imu_state->pos()(0), imu_state->pos()(1), imu_state->pos()(2));
#if ENABLE_VIZ
          static vs::Viz3D viz;
          static std::vector<cv::Affine3f> traj;
          cv::Affine3f aff = vs::isom2affine(imu_pose);
          traj.push_back(aff);
          viz.updateWidget("cood", cv::viz::WCoordinateSystem());
          viz.updateWidget("traj", cv::viz::WTrajectory(traj, 2, 1.0, cv::viz::Color::green()));
          if (!gt_poses.empty())
            viz.updateWidget("traj_gt", cv::viz::WTrajectory(gt_poses, 2, 1.0, cv::viz::Color::red()));
          std::vector<cv::Affine3f> temp = {aff};
          viz.updateWidget("cur_pose", cv::viz::WTrajectoryFrustums(temp, cv::Vec2d(60 * 0.017453, 50 * 0.017453), 0.5,
                                                                    cv::viz::Color::red()));
#endif
        }
        auto key = cv::waitKey(10);
        if (key == 27) return 0;
        break;
      }
      case vs::vio::MSG_GTPOSE:
        ok = dataset->fetchGtPose(cur_gt_pose);
        if (cur_gt_pose.ts < start_ts) break;
        if (ok)
          gt_poses.push_back(vs::isom2affine(vs::isom(cur_gt_pose.qw, cur_gt_pose.qx, cur_gt_pose.qy, cur_gt_pose.qz,
                                                      cur_gt_pose.tx, cur_gt_pose.ty, cur_gt_pose.tz)));
        break;
      default:
        break;
    }
    if (!ok) break;
  }
  printf("Finish. Press any key to exit.\n");
  getchar();
  cv::destroyAllWindows();
}

VioManagerOptions defaultParamEuroc() {
  VioManagerOptions params;
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