/**
 * Copyright (c) 2019-2021 shuyuanmao <maoshuyuan123@gmail.com>. All rights reserved.
 * @Author: shuyuanmao
 * @EMail: maoshuyuan123@gmail.com
 * @Date: 2019-05-19 02:07
 * @Description:
 */
#include "vs_vio_data_loader.h"
#include <fstream>
#include <string>
#include "vs_os.h"

#if 1
#define VS_VIO_PRINTE(format, args...) \
  do {                                 \
    printf("[ERROR]" format, ##args);  \
  } while (0)
#else
#define VS_VIO_PRINTE(format, args...)
#endif
namespace vs {
namespace vio {

class VioDataLoaderEuroc : public VioDataLoader {
 public:
  VioDataLoaderEuroc() {
    m_name = "VioDataLoaderEuroc";
    m_fimu = "mav0/imu0/data.csv";
    m_fimg = "mav0/cam0/data.csv";
    m_fgt = "mav0/state_groundtruth_estimate0/data.csv";
  }

 protected:
  struct ImgFileInfo {
    double ts;
    std::vector<std::string> fimgs;
  };

  virtual bool initImpl(const std::vector<std::string>& init_files);

  virtual bool readNextImu(ImuData& imu);

  virtual bool readNextCamera(CameraData& cam);

  virtual bool readNextGtPose(PoseData& gt_pose);

  virtual void parseImuLine(const char* line_str, ImuData& imu) {
    sscanf(line_str, "%lf,%lf,%lf,%lf,%lf,%lf,%lf", &imu.ts, &imu.gyro[0], &imu.gyro[1], &imu.gyro[2], &imu.acc[0],
           &imu.acc[1], &imu.acc[2]);
    imu.ts /= 1e9;
  }

  virtual void parseImgLine(const char* line_str, ImgFileInfo& a) {
    char name[128] = {0};
    sscanf(line_str, "%lf,%s", &a.ts, name);
    a.ts /= 1e9;  // ms -> s
    a.fimgs = {vs::join(m_datadir, "mav0/cam0/data", name), vs::join(m_datadir, "mav0/cam1/data", name)};
  }

  virtual void parseGtLine(const char* line_str, PoseData& a) {
    double vx, vy, vz, bw_x, bw_y, bw_z, ba_x, ba_y, ba_z;
    sscanf(line_str, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &a.ts, &a.tx, &a.ty, &a.tz,
           &a.qw, &a.qx, &a.qy, &a.qz, &vx, &vy, &vz, &bw_x, &bw_y, &bw_z, &ba_x, &ba_y, &ba_z);
    a.ts /= 1e9;
  }

 protected:
  std::string m_name;
  std::string m_datadir, m_fimu, m_fimg, m_fgt;
  std::ifstream m_fin_img, m_fin_imu, m_fin_gt;
};

bool VioDataLoaderEuroc::initImpl(const std::vector<std::string>& init_files) {
  if (init_files.empty()) {
    VS_VIO_PRINTE("init files empty\n");
    return false;
  }
  m_datadir = init_files[0];
  std::string img_list = vs::join(m_datadir, m_fimg);
  std::string imu_file = vs::join(m_datadir, m_fimu);
  std::string gt_file = vs::join(m_datadir, m_fgt);
  m_fin_img.open(img_list.c_str());
  m_fin_imu.open(imu_file.c_str());
  m_fin_gt.open(gt_file.c_str());
  if (!m_fin_img.is_open()) {
    VS_VIO_PRINTE("file not exist: %s\n", img_list.c_str());
    return false;
  }
  if (!m_fin_imu.is_open()) {
    VS_VIO_PRINTE("file not exist: %s\n", imu_file.c_str());
    return false;
  }
  printf("Dataset loaded. name:%s dir:%s\n", m_name.c_str(), m_datadir.c_str());
  return true;
}

bool VioDataLoaderEuroc::readNextImu(ImuData& imu) {
  imu.ts = -1;
  std::string line;
  while (getline(m_fin_imu, line)) {
    if (line.length() < 1 || line[0] == '#') continue;  // skip comment line and empty line
    parseImuLine(line.c_str(), imu);
    if (imu.ts < 0)
      return false;
    else if (imu.ts >= m_start_ts)
      return true;
    // else continue;
  }
  return false;
}

bool VioDataLoaderEuroc::readNextGtPose(PoseData& gt_pose) {
  gt_pose.ts = -1;
  std::string line;
  while (getline(m_fin_gt, line)) {
    if (line.length() < 1 || line[0] == '#') continue;  // skip comment line and empty line
    parseGtLine(line.c_str(), gt_pose);
    if (gt_pose.ts < 0)
      return false;
    else if (gt_pose.ts >= m_start_ts)
      return true;
    // else continue;
  }
  return false;
}

bool VioDataLoaderEuroc::readNextCamera(CameraData& cam) {
  cam.ts = -1;
  cam.imgs.clear();
  cam.imgs.reserve(m_mono ? 1 : 2);
  std::string line;
  ImgFileInfo info;
  while (getline(m_fin_img, line)) {
    if (line.length() < 1 || line[0] == '#') continue;  // skip comment line and empty line
    parseImgLine(line.c_str(), info);
    if (info.ts < m_start_ts) continue;
    // write camera
    if (m_mono && info.fimgs.size() > 1) info.fimgs.resize(1);
    cam.ts = info.ts;
    for (const auto& fimg : info.fimgs) {
      cv::Mat img = cv::imread(fimg, cv::IMREAD_GRAYSCALE);
      if (img.empty()) {
        VS_VIO_PRINTE("Cannot open image:%s\n", fimg.c_str());
        return false;
      }
      cam.imgs.push_back(img);
    }
    return true;
  }
  return false;
}

class VioDataLoaderTumVi : public VioDataLoaderEuroc {
 public:
  VioDataLoaderTumVi() {
    m_name = "VioDataLoaderTumVi";
    m_fimu = "mav0/imu0/data.csv";
    m_fimg = "mav0/cam0/data.csv";
    m_fgt = "mav0/mocap0/data.csv";
  }

 protected:
  virtual void parseGtLine(const char* line_str, PoseData& a) {
    sscanf(line_str, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &a.ts, &a.tx, &a.ty, &a.tz, &a.qw, &a.qx, &a.qy, &a.qz);
    a.ts /= 1e9;
  }
};

class VioDataLoaderUzhFpv : public VioDataLoaderEuroc {
 public:
  VioDataLoaderUzhFpv() {
    m_name = "VioDataLoaderUzhFpv";
    m_fimu = "imu.txt";
    m_fimg = "left_images.txt";
    m_fgt = "";
  }

 protected:
  virtual void parseImuLine(const char* line_str, ImuData& a) {
    int id;
    sscanf(line_str, "%d %lf %lf %lf %lf %lf %lf %lf", &id, &a.ts, &a.gyro[0], &a.gyro[1], &a.gyro[2], &a.acc[0],
           &a.acc[1], &a.acc[2]);
  }

  virtual void parseImgLine(const char* line_str, ImgFileInfo& a) {
    char name[128] = {0};
    int id = 0;
    sscanf(line_str, "%d %lf %s", &id, &a.ts, name);
    char fimgl[512] = {0};
    snprintf(fimgl, 512, "%s/img/image_0_%d.png", m_datadir.c_str(), id);
    char fimgr[512] = {0};
    snprintf(fimgr, 512, "%s/img/image_1_%d.png", m_datadir.c_str(), id);
    a.fimgs = {std::string(fimgl), std::string(fimgr)};
  }

  virtual void parseGtLine(const char* line_str, PoseData& a) {}
};

class VioDataLoaderZjuViSlam : public VioDataLoaderEuroc {
 public:
  VioDataLoaderZjuViSlam() {
    m_name = "VioDataLoaderZjuViSlam";
    m_fimu = "imu/data.csv";
    m_fimg = "camera/data.csv";
    m_fgt = "groundtruth/data.csv";
  }

 protected:
  virtual void parseImuLine(const char* line_str, ImuData& a) {
    sscanf(line_str, "%lf,%lf,%lf,%lf,%lf,%lf,%lf", &a.ts, &a.gyro[0], &a.gyro[1], &a.gyro[2], &a.acc[0], &a.acc[1],
           &a.acc[2]);
  }

  virtual void parseImgLine(const char* line_str, ImgFileInfo& a) {
    char name[128] = {0};
    sscanf(line_str, "%lf,%s", &a.ts, name);
    a.fimgs = {vs::join(m_datadir, "camera/images", name)};
  }

  virtual void parseGtLine(const char* line_str, PoseData& a) {
    sscanf(line_str, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &a.ts, &a.qx, &a.qy, &a.qz, &a.qw, &a.tx, &a.ty, &a.tz);
  }
};

class VioDataLoaderKitti : public VioDataLoader {
 public:
  explicit VioDataLoaderKitti() {}

 protected:
  virtual bool initImpl(const std::vector<std::string>& init_files);

  virtual bool readNextImu(ImuData& imu);

  virtual bool readNextCamera(CameraData& cam);

  virtual bool readNextGtPose(PoseData& gt_pose);

};

bool VioDataLoaderKitti::initImpl(const std::vector<std::string>& init_files) {
  if (init_files.empty()) {
    VS_VIO_PRINTE("VioDataLoaderKitti: init files empty\n");
    return false;
  }
  // todo
  return false;
}

bool VioDataLoaderKitti::readNextImu(ImuData& imu) {
  imu.ts = -1;
  // todo
  VS_VIO_PRINTE("VioDataLoaderKitti: not implemented.\n");
  return imu.ts > 0;
}

bool VioDataLoaderKitti::readNextCamera(CameraData& cam) {
  cam.ts = -1;
  cam.imgs.clear();
  // todo
  VS_VIO_PRINTE("VioDataLoaderKitti: not implemented.\n");
  return false;
}

bool VioDataLoaderKitti::readNextGtPose(PoseData& gt_pose) {
  gt_pose.ts = -1;
  // todo
  VS_VIO_PRINTE("VioDataLoaderKitti: not implemented.\n");
  return false;
}

class VioDataLoaderSymao : public VioDataLoader {
 public:
  explicit VioDataLoaderSymao() {}

 protected:
  virtual bool initImpl(const std::vector<std::string>& init_files);

  virtual bool readNextImu(ImuData& imu);

  virtual bool readNextCamera(CameraData& cam);

  virtual bool readNextGtPose(PoseData& gt_pose);

 private:
  std::string m_datadir;
  cv::Mat m_cap_img;
  cv::VideoCapture m_cap;
  std::ifstream m_fin_img;
  std::ifstream m_fin_imu;
};

bool VioDataLoaderSymao::initImpl(const std::vector<std::string>& init_files) {
  if (init_files.empty()) {
    VS_VIO_PRINTE("VioDataLoaderSymao: init files empty.\n");
    return false;
  }
  m_datadir = init_files[0];
  std::string video_file = vs::join(m_datadir, "img.avi");
  std::string img_file = vs::join(m_datadir, "imgts.txt");
  std::string imu_file = vs::join(m_datadir, "imu_meta.txt");
  m_cap.open(video_file);
  if (!m_cap.isOpened()) {
    VS_VIO_PRINTE("VioDataLoaderSymao: cannot open video %s\n", video_file.c_str());
    return false;
  }
  m_fin_img.open(img_file);
  if (!m_fin_img.is_open()) {
    VS_VIO_PRINTE("VioDataLoaderSymao: cannot open file %s\n", img_file.c_str());
    return false;
  }
  m_fin_imu.open(imu_file);
  if (!m_fin_imu.is_open()) {
    VS_VIO_PRINTE("VioDataLoaderSymao: cannot open file %s\n", imu_file.c_str());
    return false;
  }
  return true;
}

bool VioDataLoaderSymao::readNextImu(ImuData& imu) {
  imu.ts = -1;
  std::string line;
  while (getline(m_fin_imu, line)) {
    if (line.length() < 1 || line[0] == '#') continue;  // skip comment line and empty line
    sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf", &imu.ts, &imu.acc[0], &imu.acc[1], &imu.acc[2], &imu.gyro[0],
           &imu.gyro[1], &imu.gyro[2]);
    if (imu.ts < 0)
      return false;
    else if (imu.ts >= m_start_ts)
      return true;
    // else continue;
  }
  return false;
}

bool VioDataLoaderSymao::readNextCamera(CameraData& cam) {
  cam.ts = -1;
  cam.imgs.clear();
  cam.imgs.reserve(m_mono ? 1 : 2);
  if (m_fin_img.eof()) return false;
  m_fin_img >> cam.ts;
  if (cam.ts < 0) return false;
  if (!m_cap.read(m_cap_img) || m_cap_img.empty()) return false;
  cv::cvtColor(m_cap_img, m_cap_img, cv::COLOR_BGR2GRAY);
  int rows = m_cap_img.rows / 2;
  cam.imgs.push_back(m_cap_img.rowRange(0, rows));
  if (!m_mono) cam.imgs.push_back(m_cap_img.rowRange(rows, m_cap_img.rows));
  return true;
}

bool VioDataLoaderSymao::readNextGtPose(PoseData& gt_pose) {
  gt_pose.ts = -1;
  // todo
  return false;
}

std::shared_ptr<VioDataLoader> createVioDataLoader(DatasetType dataset_type) {
  std::shared_ptr<VioDataLoader> ptr;
  switch (dataset_type) {
    case DATASET_EUROC:
      return std::make_shared<VioDataLoaderEuroc>();
    case DATASET_TUM_VIO:
      return std::make_shared<VioDataLoaderTumVi>();
    case DATASET_KITTI:
      return std::make_shared<VioDataLoaderKitti>();
    case DATASET_UZH_VIO:
      return std::make_shared<VioDataLoaderUzhFpv>();
    case DATASET_ZJU_VIO:
      return std::make_shared<VioDataLoaderZjuViSlam>();
    case DATASET_SYMAO:
      return std::make_shared<VioDataLoaderSymao>();
    default:
      break;
  }
  return ptr;
}

}  // namespace vio
}  // namespace vs
