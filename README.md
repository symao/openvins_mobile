# Introduction
fork from https://github.com/rpng/open_vins

Modification:
1. remove ros and pangolin, use cv::Viz3D for visualization
2. remove boost usage
3. support dataset for EuROC, TUM-VIO, UZH-VIO, ZJU_VI

# How to use
test ok on EuROC, TUM-VIO, UZH-VIO.

```
mkdir build
cd build
cmake ..
make -j4
./test_msckf <you dataset dir>
```