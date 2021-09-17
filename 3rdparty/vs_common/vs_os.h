/**
 * Copyright (c) 2019-2021 shuyuanmao <maoshuyuan123@gmail.com>. All rights reserved.
 * @Author: shuyuanmao
 * @EMail: maoshuyuan123@gmail.com
 * @Date: 2019-05-19 02:07
 * @Description:
 */
#pragma once

#include <string>
#include <thread>
#include <vector>

namespace vs {
/** @brief get count of CPUs in platform */
int getCpuN();

/** @brief get CPU id of current thread */
int getCpuId();

/** @brief set CPU to high performance */
int setCpuHighPerformance(int cpu_id);

/** @brief bind thread to specific CPU */
int bindCpu(int cpu_id, std::thread* thread_ptr = NULL);

// /** @brief check whether file or dir has access*/
// bool access(const char* path, int mode);

// /** @brief make a directory step by step*/
// void mkdir(const char* path);

/** @brief check whether file or dir exists*/
bool exists(const char* path);

/** @brief make a multi-level directory*/
void makedirs(const char* path);

/** @brief check whether path is a file*/
bool isfile(const char* path);

/** @brief check whether path is a dir*/
bool isdir(const char* path);

/** @brief base name. eg: /a/b/c/d/ef.gh => ef.gh */
std::string basename(const char* path);

/** @brief dir name. eg: /a/b/c/d/ef.gh => /a/b/c/d/ */
std::string dirname(const char* path);

/** @brief get suffix of path*/
std::string suffix(const char* path);

/** @brief get absolutely path*/
std::string abspath(const char* path);

/** @brief split path to dirname and basename*/
void split(const std::string& file, std::string& dirname, std::string& basename);

/** @brief split path to name and suffix*/
void splitext(const std::string& file, std::string& filename, std::string& suffix);

/** @brief remove a file or a empty dir*/
void remove(const char* path);

/** @brief remove a file dir recursively*/
void rmtree(const char* path);

/** @brief rename file*/
void rename(const char* src, const char* dst);

/** @brief move file*/
void move(const char* src, const char* dst);

/** @brief copy file*/
void copy(const char* src, const char* dst);

/** @brief copy dir recursively*/
void copytree(const char* src, const char* dst);

/** @brief get file size. [Byte]*/
uint64_t filesize(const char* file);

/** @brief get dir size. [Byte]*/
uint64_t dirsize(const char* path);

/** @brief get disk size, return -1 if failed. [Byte]*/
uint64_t disksize(const char* path);

/** @brief get disk free size, return -1 if failed. [Byte]*/
uint64_t diskfree(const char* path);

/** @brief join two path*/
std::string join(const std::string& f1, const std::string& f2);

/** @brief join three path*/
std::string join(const std::string& f1, const std::string& f2, const std::string& f3);

/** @brief get all files in dir*/
std::vector<std::string> listdir(const char* path, bool absname = false, bool recursive = false);

/** @brief write content to a file.*/
void writeFile(const char* file, const char* content, const char* mode = "w");

/** @brief get home path in linux.*/
const char* homepath();

} /* namespace vs */