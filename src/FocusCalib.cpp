#include "FocusCalib.hpp"

FocusCalib::FocusCalib()
    : is_sorted_(false)
{
    calib_data_.reserve(20);
}

void FocusCalib::AddCalibData(double normal_pos, const CalibDataCV &calib_data)
{
    calib_data_.push_back(std::make_pair(normal_pos, calib_data));
    return;
}

void FocusCalib::DeletePrev()
{
    calib_data_.pop_back();
    return;
}

void FocusCalib::SortCalibData()
{
    std::sort(calib_data_.begin(), calib_data_.end(), ComparePair());

    std::cout << "[DEBUG] FocusCalib::SortCalibData Printing sorted info\n";
    for (int i = 0; i < calib_data_.size(); i++)
    {
        std::cout << std::setprecision(4);
        std::cout << "FocusCalib::SortCalibData i=" << i << " normal_pos=" << calib_data_[i].first << "\n";
        std::cout << "K=\n"
                  << calib_data_[i].second.K << "\n\n";
        std::cout << "dist_coeffs=\n"
                  << calib_data_[i].second.dist_coeffs << "\n\n";
    }

    return;
}

std::tuple<cv::Mat, cv::Mat> FocusCalib::GetInterpolatedCalibData(double encoder_value)
{
    if (calib_data_.empty())
    {
        throw std::runtime_error("No calibration data available.");
    }

    if (!is_sorted_)
    {
        SortCalibData();
        is_sorted_ = true;
    }

    // If the encoder_value is out of range, clamp it to [min, max]
    encoder_value = std::max(calib_data_.front().first, std::min(calib_data_.back().first, encoder_value));

    // Find the neighboring calibration data
    auto it = std::lower_bound(calib_data_.begin(), calib_data_.end(),
                               std::make_pair(encoder_value, CalibDataCV()), ComparePair());

    if (it == calib_data_.begin())
    {
        // Encoder value is less than the first calibration point
        return std::make_tuple(it->second.K, it->second.dist_coeffs);
    }
    if (it == calib_data_.end())
    {
        // Encoder value is greater than the last calibration point
        return std::make_tuple((it - 1)->second.K, (it - 1)->second.dist_coeffs);
    }

    // Get the neighboring calibration data points
    auto it2 = it - 1;

    double x1 = it2->first;
    double x2 = it->first;

    const CalibDataCV &data1 = it2->second;
    const CalibDataCV &data2 = it->second;

    // Calculate interpolation factor
    double t = (encoder_value - x1) / (x2 - x1);

    // Interpolate K and dist_coeffs
    cv::Mat interpolated_K           = ((1.0 - t) * data1.K) + (t * data2.K);
    cv::Mat interpolated_dist_coeffs = ((1.0 - t) * data1.dist_coeffs) + (t * data2.dist_coeffs);

    return std::make_tuple(interpolated_K, interpolated_dist_coeffs);
}

void FocusCalib::SaveCalibData(const std::string &filename) const
{
    cv::FileStorage cv_fs(filename, cv::FileStorage::WRITE);

    cv_fs << "calib_data"
          << "[";// Begin list
    for (const auto &data: calib_data_)
    {
        cv_fs << "{";// Begin mapping
        cv_fs << "encoder_value" << data.first;
        cv_fs << "K" << data.second.K;
        cv_fs << "dist_coeffs" << data.second.dist_coeffs;
        cv_fs << "}";// End mapping
    }
    cv_fs << "]";// End list

    cv_fs.release();
}


void FocusCalib::LoadCalibData(const std::string &filename)
{
    std::cout << "filename=" << filename << "\n";
    cv::FileStorage cv_fs(filename, cv::FileStorage::READ);

    calib_data_.clear();
    cv::FileNode calibNode = cv_fs["calib_data"];
    for (const auto &node: calibNode)
    {
        double encoder_value;
        cv::Mat K, dist_coeffs;

        node["encoder_value"] >> encoder_value;
        node["K"] >> K;
        node["dist_coeffs"] >> dist_coeffs;

        CalibDataCV calib_data;
        calib_data.K           = K;
        calib_data.dist_coeffs = dist_coeffs;
        calib_data_.emplace_back(encoder_value, calib_data);
    }

    SortCalibData();

    int i = 0;
    for (const auto &data: calib_data_)
    {
        std::cout << std::setprecision(4);
        std::cout << "FocusCalib::LoadCalibData i=" << i++ << "\n";
        std::cout << "FocusCalib::LoadCalibData encoder_value=" << data.first << "\n";
        std::cout << "K=\n"
                  << data.second.K << "\n\n";
        std::cout << "dist_coeffs=\n";
        std::cout << data.second.dist_coeffs << "\n\n";
        std::cout << "-------------------------------------------------\n";
    }

    cv_fs.release();
    return;
}
