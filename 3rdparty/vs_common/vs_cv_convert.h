/**
 * Copyright (c) 2019-2021 shuyuanmao <maoshuyuan123@gmail.com>. All rights reserved.
 * @Author: shuyuanmao
 * @EMail: maoshuyuan123@gmail.com
 * @Date: 2019-05-19 02:07
 * @Description:
 */
#pragma once
#include <opencv2/opencv.hpp>
#ifdef EIGEN_MAJOR_VERSION
#include <opencv2/core/eigen.hpp>
#endif  // EIGEN_MAJOR_VERSION
#include <opencv2/core/affine.hpp>
namespace vs {

inline cv::Mat Rt2T(const cv::Mat& R, const cv::Mat& tvec) {
  cv::Mat T = cv::Mat::eye(4, 4, CV_64FC1);
  R.convertTo(T(cv::Rect(0, 0, 3, 3)), CV_32FC1);
  tvec.convertTo(T(cv::Rect(3, 0, 1, 3)), CV_32FC1);
  return T;
}

inline cv::Mat rt2T(const cv::Mat& rvec, const cv::Mat& tvec) {
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  return Rt2T(R, tvec);
}

inline cv::Mat R2T(const cv::Mat& R) {
  cv::Mat T = cv::Mat::eye(4, 4, CV_64FC1);
  R.copyTo(T.rowRange(0, 3).colRange(0, 3));
  return T;
}

inline void T2Rt(const cv::Mat& T, cv::Mat& R, cv::Mat& t) {
  T.rowRange(0, 3).colRange(0, 3).copyTo(R);
  T.rowRange(0, 3).col(3).copyTo(t);
}

inline cv::Mat vec2T(const std::vector<double>& v) {
  assert(v.size() == 16);
  cv::Mat T(4, 4, CV_64FC1);
  double* Tdata = reinterpret_cast<double*>(T.data);
  for (int i = 0; i < 16; i++) {
    Tdata[i] = v[i];
  }
  return T;
}

inline cv::Mat camMat(double fx, double fy, double cx, double cy) {
  return (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
}

#ifdef EIGEN_MAJOR_VERSION

inline Eigen::Isometry3d isom(double qw, double qx, double qy, double qz, double tx, double ty, double tz) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
  T.translation() << tx, ty, tz;
  return T;
}

inline Eigen::Isometry3d Rt2isom(const cv::Mat& R, const cv::Mat& tvec) {
  Eigen::Matrix3d eigR;
  Eigen::Vector3d eigt;
  cv2eigen(R, eigR);
  cv2eigen(tvec, eigt);
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = eigR;
  T.translation() = eigt;
  return T;
}

inline Eigen::Isometry3d rt2isom(const cv::Mat& rvec, const cv::Mat& tvec) {
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  return Rt2isom(R, tvec);
}

inline void isom2rt(const Eigen::Isometry3d& T, cv::Mat& rvec, cv::Mat& tvec) {
  cv::Mat R;
  eigen2cv(Eigen::Matrix3d(T.linear()), R);
  eigen2cv(Eigen::Vector3d(T.translation()), tvec);
  cv::Rodrigues(R, rvec);
}

inline cv::Affine3f isom2affine(const Eigen::Isometry3d& isom) {
  cv::Mat T;
  eigen2cv(isom.matrix(), T);
  T.convertTo(T, CV_32FC1);
  return cv::Affine3f(T);
}

inline Eigen::Vector3d cvt3d(const cv::Point3f& a) { return Eigen::Vector3d(a.x, a.y, a.z); }
inline cv::Point3f cvt3d(const Eigen::Vector3d& a) { return cv::Point3f(a(0), a(1), a(2)); }
inline Eigen::Vector2d cvt2d(const cv::Point2f& a) { return Eigen::Vector2d(a.x, a.y); }
inline cv::Point2f cvt2d(const Eigen::Vector2d& a) { return cv::Point2f(a(0), a(1)); }

#endif  // EIGEN_MAJOR_VERSION

} /* namespace vs */
