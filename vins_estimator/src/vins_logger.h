#pragma once
/**
 * VINS-Mono Diagnostic File Logger
 *
 * 线程安全的文件日志工具，同时输出到 ROS 控制台和日志文件
 * 日志位置: ~/catkin_vins/logs/vins_<node>_<timestamp>.log
 *
 * 用法:
 *   main() 中: VinsFileLogger::instance().init("node_name");
 *   代码中:    VINS_LOG("[TAG] msg=%d", value);
 *             VINS_WARN("[TAG] warning msg");
 */

#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <mutex>
#include <ros/ros.h>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>


class VinsFileLogger {
public:
  static VinsFileLogger &instance() {
    static VinsFileLogger logger;
    return logger;
  }

  void init(const std::string &node_name) {
    std::lock_guard<std::mutex> lock(mtx_);

    const char *home = getenv("HOME");
    if (!home) {
      ROS_ERROR("[VinsLogger] HOME environment variable not set!");
      return;
    }
    // 创建目录 ~/catkin_vins/logs
    std::string dir1 = std::string(home) + "/catkin_vins";
    std::string dir2 = dir1 + "/logs";
    mkdir(dir1.c_str(), 0755);
    mkdir(dir2.c_str(), 0755);

    // 带时间戳的日志文件名
    time_t now = time(nullptr);
    struct tm *t = localtime(&now);
    char filename[512];
    snprintf(filename, sizeof(filename),
             "%s/vins_%s_%04d%02d%02d_%02d%02d%02d.log", dir2.c_str(),
             node_name.c_str(), t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
             t->tm_hour, t->tm_min, t->tm_sec);

    file_.open(filename, std::ios::out);
    if (file_.is_open()) {
      ROS_INFO("[VinsLogger] Log file opened: %s", filename);
      char time_str[64];
      strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", t);
      file_ << "========================================" << std::endl;
      file_ << " VINS-Mono Diagnostic Log" << std::endl;
      file_ << " Node : " << node_name << std::endl;
      file_ << " Start: " << time_str << std::endl;
      file_ << "========================================" << std::endl;
    } else {
      ROS_ERROR("[VinsLogger] Failed to open log file: %s", filename);
    }
  }

  void log(const char *fmt, ...) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!file_.is_open())
      return;

    // 获取毫秒级时间戳
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    struct tm *t = localtime(&ts.tv_sec);

    char time_buf[64];
    snprintf(time_buf, sizeof(time_buf), "[%02d:%02d:%02d.%03ld]", t->tm_hour,
             t->tm_min, t->tm_sec, ts.tv_nsec / 1000000);

    // 格式化消息
    char msg_buf[2048];
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg_buf, sizeof(msg_buf), fmt, args);
    va_end(args);

    file_ << time_buf << " " << msg_buf << "\n";
    file_.flush(); // 立即写入，防止崩溃丢失日志
  }

  ~VinsFileLogger() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (file_.is_open()) {
      file_ << "========== Log End ==========" << std::endl;
      file_.close();
    }
  }

private:
  VinsFileLogger() {}
  VinsFileLogger(const VinsFileLogger &) = delete;
  VinsFileLogger &operator=(const VinsFileLogger &) = delete;

  std::ofstream file_;
  std::mutex mtx_;
};

// 同时输出到 ROS_INFO 和日志文件
#define VINS_LOG(fmt, ...)                                                     \
  do {                                                                         \
    ROS_INFO(fmt, ##__VA_ARGS__);                                              \
    VinsFileLogger::instance().log(fmt, ##__VA_ARGS__);                        \
  } while (0)

// 同时输出到 ROS_WARN 和日志文件 (带 [WARN] 标记)
#define VINS_WARN(fmt, ...)                                                    \
  do {                                                                         \
    ROS_WARN(fmt, ##__VA_ARGS__);                                              \
    VinsFileLogger::instance().log("[WARN] " fmt, ##__VA_ARGS__);              \
  } while (0)
