//
// Created by jerett on 25-5-9.
//

#pragma once
#ifndef LIGHTNING_DATA_DUMPER_H
#define LIGHTNING_DATA_DUMPER_H

#include <fstream>
#include <mutex>
#include <string>

#include <opencv2/opencv.hpp>
#include <pcl/io/ply_io.h>

#include "common/eigen_types.h"
#include "common/nav_state.h"
#include "common/point_def.h"

namespace lightning {

/// 数据dump工具，线程安全
/// 类似 ui_ 的模式：if (dumper_) { dumper_->DumpXxx(...) }
class DataDumper {
   public:
    struct Options {
        bool enable = false;             // 总开关
        bool dump_predict = true;        // dump predict 状态量 (IMU predict 后)
        bool dump_update = true;         // dump update 状态量 (scan matching 后)
        bool dump_keyframe = true;       // dump 关键帧状态量
        bool dump_scans = true;          // dump 点云 (ply)
        bool dump_images = true;         // dump g2p5 地图图像 (png)
        std::string output_dir = "./dump_output";  // 输出目录
    };

    DataDumper(const Options& options);
    ~DataDumper();

    /// 状态量 dump — 追加写入，线程安全
    /// NavState 全量: timestamp, pos, quat, vel, bg, ba, grav, confidence
    void DumpPredictState(const NavState& state);
    void DumpUpdateState(const NavState& state);
    void DumpKeyframeState(const NavState& lio_state, const SE3& opt_pose);

    /// 点云 dump — 每帧一个文件，保留 IMU 系，线程安全
    /// 文件名用时间戳，同时在 scan_index.txt 中记录映射
    void DumpScan(double timestamp, CloudPtr cloud);

    /// 图像 dump — 每次 callback 一个文件，线程安全
    /// 文件名用时间戳，同时在 image_index.txt 中记录映射
    void DumpImage(double timestamp, const cv::Mat& image);

    /// 全局配置 dump — 写入 global_config.txt（一次性）
    /// gravity: 世界系下重力向量
    /// R_LtoI: 从 LiDAR 到 IMU 的旋转矩阵
    /// p_LinI: LiDAR 原点在 IMU 坐标系中的位置
    void DumpGlobalConfig(const Vec3d& gravity, const Mat3d& R_LtoI, const Vec3d& p_LinI);

    bool DumpPredictEnabled() const { return options_.enable && options_.dump_predict; }
    bool DumpUpdateEnabled() const { return options_.enable && options_.dump_update; }
    bool DumpKeyframeEnabled() const { return options_.enable && options_.dump_keyframe; }
    bool DumpScansEnabled() const { return options_.enable && options_.dump_scans; }
    bool DumpImagesEnabled() const { return options_.enable && options_.dump_images; }

   private:
    Options options_;
    std::mutex mtx_;  // 所有写文件操作共用一把锁

    std::ofstream predict_file_;  // predict_state.txt
    std::ofstream update_file_;   // update_state.txt
    std::ofstream kf_lio_file_;   // kf_lio_state.txt
    std::ofstream kf_opt_file_;   // kf_opt_state.txt
    std::ofstream scan_index_;    // scan_index.txt
    std::ofstream image_index_;   // image_index.txt

    /// 写 NavState 全量到文件
    /// 格式: timestamp tx ty tz qx qy qz qw vx vy vz bgx bgy bgz bax bay baz gx gy gz confidence
    void WriteStateLine(std::ofstream& file, const NavState& state);

    /// 写 TUM 格式位姿
    /// 格式: timestamp tx ty tz qx qy qz qw
    void WriteTUMLine(std::ofstream& file, double timestamp, const SE3& pose);

    /// 时间戳转文件名: 使用纳秒精度整数，如 1702345678.123456789 -> 1702345678123456789
    static std::string TimestampToFilename(double timestamp, const std::string& ext);
};

}  // namespace lightning

#endif  // LIGHTNING_DATA_DUMPER_H