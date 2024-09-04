#include <iostream>

#include "FocusCalib.hpp"

using namespace std;

int main()
{
    FocusCalib focus_calib;

    CalibDataCV calib0   = {cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(5, 1, CV_64F)};
    CalibDataCV calib025 = {cv::Mat::eye(3, 3, CV_64F) * 2.0, cv::Mat::ones(5, 1, CV_64F)};
    CalibDataCV calib05  = {cv::Mat::eye(3, 3, CV_64F) * 3.0, cv::Mat::ones(5, 1, CV_64F) * 2};
    CalibDataCV calib075 = {cv::Mat::eye(3, 3, CV_64F) * 4.0, cv::Mat::ones(5, 1, CV_64F) * 3};
    CalibDataCV calib1   = {cv::Mat::eye(3, 3, CV_64F) * 5.0, cv::Mat::ones(5, 1, CV_64F) * 4};

    focus_calib.AddCalibData(0.0, calib0);
    focus_calib.AddCalibData(0.25, calib025);
    focus_calib.AddCalibData(0.5, calib05);
    focus_calib.AddCalibData(0.75, calib075);
    focus_calib.AddCalibData(1.0, calib1);

    focus_calib.SortCalibData();

    double encoder_value                            = 0.47;
    auto [interpolated_K, interpolated_dist_coeffs] = focus_calib.GetInterpolatedCalibData(encoder_value);

    std::cout << "Interpolated K:\n"
              << interpolated_K << std::endl;
    std::cout << "Interpolated dist_coeffs:\n"
              << interpolated_dist_coeffs << std::endl;

    focus_calib.SaveCalibData("save_calib_data.yaml");
    focus_calib.LoadCalibData("save_calib_data.yaml");

    return 0;
}
