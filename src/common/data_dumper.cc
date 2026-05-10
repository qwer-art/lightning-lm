//
// Created by jerett on 25-5-9.
//

#include "common/data_dumper.h"

#include <filesystem>
#include <iomanip>

#include <glog/logging.h>

namespace lightning {

DataDumper::DataDumper(const Options& options) : options_(options) {
    if (!options_.enable) {
        return;
    }

    // 创建输出目录
    namespace fs = std::filesystem;
    fs::create_directories(options_.output_dir);

    if (options_.dump_scans) {
        fs::create_directories(options_.output_dir + "/scans");
    }
    if (options_.dump_images) {
        fs::create_directories(options_.output_dir + "/images");
    }

    // 打开轨迹文件
    if (options_.dump_predict) {
        predict_file_.open(options_.output_dir + "/predict_state.txt");
        CHECK(predict_file_.is_open()) << "Failed to open predict_state.txt";
        // 写表头
        predict_file_ << "# timestamp tx ty tz qx qy qz qw vx vy vz bgx bgy bgz bax bay baz gx gy gz confidence\n";
    }
    if (options_.dump_update) {
        update_file_.open(options_.output_dir + "/update_state.txt");
        CHECK(update_file_.is_open()) << "Failed to open update_state.txt";
        update_file_ << "# timestamp tx ty tz qx qy qz qw vx vy vz bgx bgy bgz bax bay baz gx gy gz confidence\n";
    }
    if (options_.dump_keyframe) {
        kf_lio_file_.open(options_.output_dir + "/kf_lio_state.txt");
        kf_opt_file_.open(options_.output_dir + "/kf_opt_state.txt");
        CHECK(kf_lio_file_.is_open()) << "Failed to open kf_lio_state.txt";
        CHECK(kf_opt_file_.is_open()) << "Failed to open kf_opt_state.txt";
        kf_lio_file_ << "# timestamp tx ty tz qx qy qz qw vx vy vz bgx bgy bgz bax bay baz gx gy gz confidence\n";
        kf_opt_file_ << "# timestamp tx ty tz qx qy qz qw\n";
    }

    // 打开索引文件
    if (options_.dump_scans) {
        scan_index_.open(options_.output_dir + "/scan_index.txt");
        CHECK(scan_index_.is_open()) << "Failed to open scan_index.txt";
        scan_index_ << "# timestamp filename\n";
    }
    if (options_.dump_images) {
        image_index_.open(options_.output_dir + "/image_index.txt");
        CHECK(image_index_.is_open()) << "Failed to open image_index.txt";
        image_index_ << "# timestamp filename\n";
    }

    LOG(INFO) << "DataDumper enabled, output dir: " << options_.output_dir;
}

DataDumper::~DataDumper() {
    if (predict_file_.is_open()) predict_file_.close();
    if (update_file_.is_open()) update_file_.close();
    if (kf_lio_file_.is_open()) kf_lio_file_.close();
    if (kf_opt_file_.is_open()) kf_opt_file_.close();
    if (scan_index_.is_open()) scan_index_.close();
    if (image_index_.is_open()) image_index_.close();
}

std::string DataDumper::TimestampToFilename(double timestamp, const std::string& ext) {
    // ROS/bag 时间戳惯例: 秒.纳秒 -> 整数纳秒
    // e.g. 1702345678.123456789 -> 1702345678123456789
    int64_t ts_ns = static_cast<int64_t>(timestamp * 1e9 + 0.5);
    std::ostringstream ss;
    ss << ts_ns << ext;
    return ss.str();
}

void DataDumper::WriteStateLine(std::ofstream& file, const NavState& state) {
    auto t = state.GetPose().translation();
    auto q = state.GetPose().unit_quaternion();
    auto v = state.vel_;
    auto bg = state.bg_;
    auto ba = state.Getba();
    auto g = state.grav_;

    file << std::fixed << std::setprecision(9) << state.timestamp_ << " " << t.x() << " " << t.y() << " " << t.z()
         << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " " << v.x() << " " << v.y() << " "
         << v.z() << " " << bg.x() << " " << bg.y() << " " << bg.z() << " " << ba.x() << " " << ba.y() << " "
         << ba.z() << " " << g.x() << " " << g.y() << " " << g.z() << " " << state.confidence_ << "\n";
}

void DataDumper::WriteTUMLine(std::ofstream& file, double timestamp, const SE3& pose) {
    auto t = pose.translation();
    auto q = pose.unit_quaternion();
    file << std::fixed << std::setprecision(9) << timestamp << " " << t.x() << " " << t.y() << " " << t.z() << " "
         << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
}

void DataDumper::DumpPredictState(const NavState& state) {
    std::lock_guard<std::mutex> lock(mtx_);
    WriteStateLine(predict_file_, state);
}

void DataDumper::DumpUpdateState(const NavState& state) {
    std::lock_guard<std::mutex> lock(mtx_);
    WriteStateLine(update_file_, state);
}

void DataDumper::DumpKeyframeState(const NavState& lio_state, const SE3& opt_pose) {
    std::lock_guard<std::mutex> lock(mtx_);
    WriteStateLine(kf_lio_file_, lio_state);
    WriteTUMLine(kf_opt_file_, lio_state.timestamp_, opt_pose);
}

void DataDumper::DumpScan(double timestamp, CloudPtr cloud) {
    std::lock_guard<std::mutex> lock(mtx_);

    std::string filename = TimestampToFilename(timestamp, ".ply");
    std::string filepath = options_.output_dir + "/scans/" + filename;

    // 保留 IMU 系，不做变换
    pcl::io::savePLYFileBinary(filepath, *cloud);

    // 写索引
    scan_index_ << std::fixed << std::setprecision(9) << timestamp << " " << filename << "\n";
}

void DataDumper::DumpImage(double timestamp, const cv::Mat& image) {
    std::lock_guard<std::mutex> lock(mtx_);

    std::string filename = TimestampToFilename(timestamp, ".png");
    std::string filepath = options_.output_dir + "/images/" + filename;

    cv::imwrite(filepath, image);

    // 写索引
    image_index_ << std::fixed << std::setprecision(9) << timestamp << " " << filename << "\n";
}

}  // namespace lightning