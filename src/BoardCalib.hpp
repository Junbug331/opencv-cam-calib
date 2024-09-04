#ifndef BOARD_CALIB_HPP
#define BOARD_CALIB_HPP

#include "OpenCVCamCalib.hpp"
#include <thread>
#include <mutex>

class BoardCalib
{
    std::string yaml_file;

    int board_width_;
    int board_height_;
    int img_width_;
    int img_height_;
    int device_id_;
    double fps_;
    double pattern_size_mm_;
    BOARD_PATTERN board_pattern_;
    CAM_MODEL cam_model_;
    cv::VideoCapture cap_;
    std::thread vis_thread_;

public:
    std::vector<std::vector<cv::Point3f>> object_points_;
    std::vector<std::vector<cv::Point2f>> image_points_;
    std::vector<cv::Mat> rvecs_, tvecs_;
    cv::Mat K, dist_coeffs;
    cv::Size img_size;

public:
    BoardCalib(const std::string &a_rYamlFile);
    ~BoardCalib();

    void RunBoardCalibration(const std::string &boardimg_dir);
    std::tuple<cv::Mat, cv::Mat> RunBoardCalibration_debug(const std::string &boardimg_dir);
    void SaveCalibXML(const std::string &calib_file);
    void SaveCalibXML_debug(const std::string &calib_file, const double &encoder_value);
    void LoadCalibXML_debug(const std::string &calib_file, double &encoder_value);
};

#endif
