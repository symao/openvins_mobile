#include <stdio.h>
#include <memory>
#include "config.h"

#define ENABLE_VIZ 1

#if ENABLE_VIZ
#include <vs_common/vs_viz3d.h>
#endif

using namespace ov_msckf;

int main(int argc, char** argv) {
  std::string datadir = argc > 1 ? argv[1] : "/home/symao/data/euroc/zip/MH_04_difficult";
  bool mono = true;
  RunHandler hd;
  bool ok = hd.init(datadir, mono);

  const bool draw_img = true;
  vs::vio::ImuData cur_imu;
  vs::vio::CameraData cur_cam;
  vs::vio::PoseData cur_gt_pose;
  std::vector<cv::Affine3f> gt_poses;
  vs::ElemBuffer<Eigen::Isometry3d> cur_imu_pose, dT;  // for align gt traj and msckf traj
  while (ok) {
    ok = false;
    switch (hd.dataset->nextType()) {
      case vs::vio::MSG_IMU:
        ok = hd.dataset->fetchImu(cur_imu);
        // printf("IMU: %f gyro:(%.3f,%.3f,%.3f) acc:(%.3f,%.3f,%.3f)\n", cur_imu.ts, cur_imu.gyro[0], cur_imu.gyro[1],
        //        cur_imu.gyro[2], cur_imu.acc[0], cur_imu.acc[1], cur_imu.acc[2]);
        if (ok) {
          ov_core::ImuData msg;
          msg.timestamp = cur_imu.ts;
          msg.wm << cur_imu.gyro[0], cur_imu.gyro[1], cur_imu.gyro[2];
          msg.am << cur_imu.acc[0], cur_imu.acc[1], cur_imu.acc[2];
          hd.sys->feed_measurement_imu(msg);
        }
        break;
      case vs::vio::MSG_CAMERA: {
        ok = hd.dataset->fetchCamera(cur_cam);
        printf("CAM: %f %d ", cur_cam.ts, static_cast<int>(cur_cam.imgs.size()));
        for (const auto& img : cur_cam.imgs) printf(" [%dx%d,%d]", img.cols, img.rows, img.channels());
        printf("\n");
        if (ok) {
          ov_core::CameraData msg;
          msg.timestamp = cur_cam.ts;
          msg.sensor_ids.push_back(0);
          msg.images.push_back(cur_cam.imgs[0].clone());
          msg.masks.push_back(cv::Mat::zeros(cur_cam.imgs[0].rows, cur_cam.imgs[0].cols, CV_8UC1));
          hd.sys->feed_measurement_camera(msg);
        }
        auto img_show = hd.sys->get_historical_viz_image();
        // auto img_show = cur_cam.imgs[0];
        if (!img_show.empty()) {
          char str[256] = {0};
          snprintf(str, 256, "%f", cur_cam.ts);
          cv::putText(img_show, str, cv::Point(5, img_show.rows - 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                      cv::Scalar(0, 255, 255), 2);
          cv::imshow("image", img_show);
        }
        if (hd.sys->initialized()) {
          auto imu_state = hd.sys->get_state()->_imu;
          auto imu_pose = vs::isom(imu_state->quat()(3), imu_state->quat()(0), imu_state->quat()(1),
                                   imu_state->quat()(2), imu_state->pos()(0), imu_state->pos()(1), imu_state->pos()(2));
          cur_imu_pose.set(imu_pose);
#if ENABLE_VIZ
          static vs::Viz3D viz;
          static std::vector<cv::Affine3f> traj;
          static const int cam_id = 0;
          static const Eigen::Isometry3d T_CtoI = vec2isom(hd.params.camera_extrinsics[cam_id]).inverse();
          static const auto& cam_intrin = hd.params.camera_intrinsics[cam_id];
          static const auto& cam_wh = hd.params.camera_wh[cam_id];
          cv::Affine3f aff = vs::isom2affine(imu_pose * T_CtoI);
          traj.push_back(aff);
          viz.updateWidget("cood", cv::viz::WCoordinateSystem());
          viz.updateWidget("traj", cv::viz::WTrajectory(traj, 2, 1.0, cv::viz::Color::green()));
          if (!gt_poses.empty())
            viz.updateWidget("traj_gt", cv::viz::WTrajectory(gt_poses, 2, 1.0, cv::viz::Color::red()));
          // draw camera frustum
          std::vector<cv::Affine3f> temp = {aff};
          cv::Vec2d fov = calcFov(cam_intrin(0, 0), cam_intrin(1, 0), cv::Size(cam_wh.first, cam_wh.second));
          viz.updateWidget("cur_pose", cv::viz::WTrajectoryFrustums(temp, fov, 0.5, cv::viz::Color::yellow()));
          // draw pts
          auto pts_slam = cvtPts(hd.sys->get_features_SLAM());
          auto pts_map = cvtPts(hd.sys->get_slam_map());
          if (!pts_map.empty()) viz.updateWidget("pts_map", cv::viz::WCloud(pts_map, cv::viz::Color::blue()));
          if (!pts_slam.empty()) viz.updateWidget("pts_slam", cv::viz::WCloud(pts_slam, cv::viz::Color::rose()));
#endif
        }
        auto key = cv::waitKey(10);
        if (key == 27) return 0;
        break;
      }
      case vs::vio::MSG_GTPOSE:
        ok = hd.dataset->fetchGtPose(cur_gt_pose);
        if (ok) {
          auto gt_isom = vs::isom(cur_gt_pose.qw, cur_gt_pose.qx, cur_gt_pose.qy, cur_gt_pose.qz, cur_gt_pose.tx,
                                  cur_gt_pose.ty, cur_gt_pose.tz);
          if (cur_imu_pose.has() && !dT.has()) dT.set(cur_imu_pose.get() * gt_isom.inverse());
          if (dT.has()) gt_poses.push_back(vs::isom2affine(dT.get() * gt_isom));
        }
        break;
      default:
        break;
    }
  }
  printf("Finish. Press any key to exit.\n");
  getchar();
  cv::destroyAllWindows();
}