/**
 * Copyright (c) 2019-2021 shuyuanmao <maoshuyuan123@gmail.com>. All rights reserved.
 * @Author: shuyuanmao
 * @EMail: maoshuyuan123@gmail.com
 * @Date: 2019-05-19 02:07
 * @Description:
 */
#include "vs_os.h"

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <fstream>
#include <string>
#include <thread>

#ifdef __linux__
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <sys/stat.h>
#include <sys/statfs.h>
#include <sys/types.h>
#include <unistd.h>

namespace vs {

int getCpuN() {
  static int n_cpu = sysconf(_SC_NPROCESSORS_CONF);
  return n_cpu;
}

int getCpuId() { return sched_getcpu(); }

int setCpuHighPerformance(int cpu_id) {
  if (cpu_id < 0 || cpu_id >= getCpuN()) {
    printf("[WARN]Failed set cpu performance. invalid id:%d\n", cpu_id);
    return -1;
  }
  char path[256] = {0};
  sprintf(path, "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_governor", cpu_id);

  FILE* fp = fopen(path, "w");
  if (!fp) {
    printf("[ERROR]Failed set high performance. Cannot open file %s\n", path);
    return -1;
  }
  fprintf(fp, "performance");
  fclose(fp);
  return 0;
}

int bindCpu(int cpu_id, std::thread* thread_ptr) {
  if (cpu_id < 0 || cpu_id >= getCpuN()) {
    printf("[WARN]Failed bind thread to cpu. invalid id:%d\n", cpu_id);
    return -1;
  }
  // only CPU i as set.
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(cpu_id, &cpuset);
  if (thread_ptr)
    pthread_setaffinity_np(thread_ptr->native_handle(), sizeof(cpu_set_t), &cpuset);
  else
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
  return 0;
}

// bool access(const char* path, int mode)
// {
//     return access(path, mode);
// }

// void mkdir(const char* path)
// {
//     mkdir(path, S_IRWXU|S_IRWXG|S_IRWXO);
// }

const static char SPLITER = '/';

bool exists(const char* path) { return access(path, 0) != -1; }

void makedirs(const char* path) {
  size_t t = -1;
  std::string s(path);
  while ((t = s.find_first_of("\\/", t + 1)) != s.npos) {
    const char* sub_path = s.substr(0, t).c_str();
    if (!exists(sub_path)) mkdir(sub_path, S_IRWXU | S_IRWXG | S_IRWXO);
  }
  if (!exists(path)) mkdir(path, S_IRWXU | S_IRWXG | S_IRWXO);
}

bool isfile(const char* path) {
  struct stat statbuf;
  if (stat(path, &statbuf) < 0) return false;
  return S_ISREG(statbuf.st_mode);
}

bool isdir(const char* path) {
  struct stat statbuf;
  if (stat(path, &statbuf) < 0) return false;
  return S_ISDIR(statbuf.st_mode);
}

std::string basename(const char* path) {
  std::string a, b;
  split(std::string(path), a, b);
  return b;
}

std::string dirname(const char* path) {
  std::string a, b;
  split(std::string(path), a, b);
  return a;
}

std::string suffix(const char* path) {
  std::string a, b;
  splitext(std::string(path), a, b);
  return b;
}

std::string abspath(const char* path) {
  char abs_path[PATH_MAX + 1];
  if (realpath(path, abs_path))
    return std::string(abs_path);
  else
    return std::string();
}

void split(const std::string& file, std::string& dirname, std::string& basename) {
  auto n = file.find_last_of(SPLITER);
  if (n == file.npos) {
    dirname = file;
    basename = std::string();
  } else {
    dirname = file.substr(0, n);
    basename = file.substr(n + 1);
  }
}

void splitext(const std::string& file, std::string& filename, std::string& suffix) {
  auto n = file.find_last_of('.');
  if (n == file.npos) {
    filename = file;
    suffix = std::string();
  } else {
    filename = file.substr(0, n);
    suffix = file.substr(n);
  }
}

void remove(const char* path) { std::remove(path); }

void rmtree(const char* path) {
  DIR* dp;
  struct dirent* ptr;
  if ((dp = opendir(path)) == NULL) {
    return;
  }
  while ((ptr = readdir(dp)) != NULL) {
    if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0) continue;
    std::string p = join(path, ptr->d_name);

    if (ptr->d_type == 4) {
      rmtree(p.c_str());
    } else {
      std::remove(p.c_str());
    }
  }
  closedir(dp);
  std::remove(path);
}

void rename(const char* src, const char* dst) { move(src, dst); }

void move(const char* src, const char* dst) {
  char cmd[512] = {0};
  snprintf(cmd, sizeof(cmd), "mv %s %s", src, dst);
  int res = system(cmd);
  res = res;
}

void copy(const char* src, const char* dst) {
  std::ifstream fin(src);
  std::ofstream fout(dst);
  if (!fin.is_open() || !fout.is_open()) {
    printf("[ERROR]copy: failed, %s not open.\n", fin.is_open() ? dst : src);
    return;
  }
  fout << fin.rdbuf();
  fin.close();
  fout.close();
}

void copytree(const char* src, const char* dst) {
  char cmd[512] = {0};
  snprintf(cmd, sizeof(cmd), "cp -r %s %s", src, dst);
  int res = system(cmd);
  res = res;
}

uint64_t filesize(const char* file) {
  struct stat statbuf;
  if (stat(file, &statbuf) < 0)
    return -1;
  else
    return statbuf.st_size;
}

uint64_t dirsize(const char* path) {
  DIR* dp;
  struct dirent* ptr;
  struct stat statbuf;
  uint64_t dir_size = 0;
  if ((dp = opendir(path)) == NULL) return -1;
  lstat(path, &statbuf);
  dir_size += statbuf.st_size;
  while ((ptr = readdir(dp)) != NULL) {
    char subdir[256] = {0};
    snprintf(subdir, sizeof(subdir), "%s/%s", path, ptr->d_name);
    lstat(subdir, &statbuf);
    if (S_ISDIR(statbuf.st_mode)) {
      if (strcmp(".", ptr->d_name) == 0 || strcmp("..", ptr->d_name) == 0) continue;
      dir_size += dirsize(subdir);
    } else {
      dir_size += statbuf.st_size;
    }
  }
  closedir(dp);
  return dir_size;
}

uint64_t disksize(const char* path) {
  struct statfs disk_info;
  if (statfs(path, &disk_info) == -1) return -1;
  return (uint64_t)disk_info.f_bsize * disk_info.f_blocks;
}

uint64_t diskfree(const char* path) {
  struct statfs disk_info;
  if (statfs(path, &disk_info) == -1) return -1;
  return (uint64_t)disk_info.f_bsize * disk_info.f_bfree;
}

std::string join(const std::string& f1, const std::string& f2) {
  if (f1.back() == SPLITER) return f1 + f2;
  return f1 + SPLITER + f2;
}

std::string join(const std::string& f1, const std::string& f2, const std::string& f3) { return join(join(f1, f2), f3); }

std::vector<std::string> listdir(const char* path, bool absname, bool recursive) {
  std::vector<std::string> files;
  DIR* dp;
  struct dirent* ptr;
  if ((dp = opendir(path)) == NULL) {
    return files;
  }

  while ((ptr = readdir(dp)) != NULL) {
    if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)
      continue;
    else if (ptr->d_type == 8)  /// file
    {
      files.push_back(!absname ? ptr->d_name : join(path, ptr->d_name));
    } else if (ptr->d_type == 10)  /// link file
    {
      continue;
    } else if (ptr->d_type == 4)  /// dir
    {
      if (recursive) {
        auto subfiles = listdir(join(path, ptr->d_name).c_str(), absname, recursive);
        files.insert(files.end(), subfiles.begin(), subfiles.end());
      }
    }
  }
  closedir(dp);
  return files;
}

void writeFile(const char* file, const char* content, const char* mode) {
  FILE* fp = fopen(file, mode);
  if (!fp) {
    printf("[ERROR] open file '%s' failed.\n", file);
    return;
  }
  fprintf(fp, "%s\n", content);
  fclose(fp);
}

const char* homepath() { return getenv("HOME"); }

} /* namespace vs */

#else   //__linux__
// TODO: implemented in other platform
#endif  //__linux__
