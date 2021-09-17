cmake_minimum_required(VERSION 2.8.3)
project(msckf)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")
set(BUILD_DEMO 1)
set(BUILD_VS_COMMON 1)

find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    src/ov_core
    src/ov_msckf
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# build vs_common
if(${BUILD_VS_COMMON})
    # use 3rdparty version
    include_directories(3rdparty)
    file(GLOB VS_COMMON_SRC 3rdparty/vs_common/*.cpp)
    add_library(vs_common ${VS_COMMON_SRC})
endif()

file(GLOB_RECURSE OV_CORE_SRC src/ov_core/*.cpp)
add_library(ov_core SHARED ${OV_CORE_SRC})

file(GLOB_RECURSE OV_MSCKF_SRC src/ov_msckf/*.cpp)
add_library(ov_msckf SHARED ${OV_MSCKF_SRC})

list(APPEND OV_LIBS ov_msckf ov_core)

if(${BUILD_DEMO})
link_directories(${PROJECT_SOURCE_DIR}/lib)
add_executable(test_msckf example/test_msckf.cpp)
target_link_libraries(test_msckf ${OV_LIBS} vs_common ${OpenCV_LIBS} pthread)
endif()
