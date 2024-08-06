#pragma ocne
#ifndef OPENCVCAMCALIB_HPP
#define OPENCVCAMCALIB_HPP

#include <optional>
#include <filesystem>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

enum CAM_MODEL
{
    PINHOLE = 0,
    FISHEYE = 1
};
double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f>> &objectPoints,
                                 const std::vector<std::vector<cv::Point2f>> &imagePoints,
                                 const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs,
                                 const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                                 std::vector<float> &perViewErrors, bool fisheye);

std::optional<std::tuple<std::vector<cv::Point2f>, cv::Mat>> FindCheckerboard(
        std::filesystem::path a_oPath,
        const int32_t a_nBoardWidth,
        const int32_t a_nBoardHeight,
        bool a_bUseFisheye = false);

bool CheckChessBoard(const cv::Mat &image,
                     int a_nBoardWidth,
                     int a_nBoardHeight,
                     float a_fThreshold = 0.6f,
                     float *a_pRatio    = nullptr,
                     bool a_bShowDebug  = false);

double cvCheckerboardCalibration(int a_nBoardWidth,
                                 int a_nBoardHeight,
                                 float a_fSquareSize_mm,
                                 const std::string &a_rImageDirPath,
                                 cv::Mat &a_rK,
                                 cv::Mat &a_rDistCoeffs,
                                 int &a_rWidth,
                                 int &a_rHeight,
                                 CAM_MODEL a_eCamModel = CAM_MODEL::PINHOLE,
                                 const bool a_bDebug   = true);


#endif