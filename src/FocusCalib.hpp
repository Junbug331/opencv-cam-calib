#ifndef FOUCS_CALIB_HPP
#define FOUCS_CALIB_HPP

#include <opencv2/opencv.hpp>
#include <utility>
#include <vector>
#include <tuple>

struct CalibDataCV {
    cv::Mat K;
    cv::Mat dist_coeffs;
};

class FocusCalib
{
    struct ComparePair {
        bool operator()(const std::pair<double, CalibDataCV> &p1, const std::pair<double, CalibDataCV> &p2)
        {
            // Return true if the first pair should be after the second pair
            return p1.first < p2.first;
        }
    };

    bool is_sorted_ = false;
    std::vector<std::pair<double, CalibDataCV>> calib_data_;

public:
    FocusCalib();
    void AddCalibData(double normal_pos, const CalibDataCV &calib_data);
    void DeletePrev();
    void SortCalibData();
    std::tuple<cv::Mat, cv::Mat> GetInterpolatedCalibData(double encoder_value);
    void SaveCalibData(const std::string &filename) const;
    void LoadCalibData(const std::string &filename);
};

#endif