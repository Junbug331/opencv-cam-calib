#include <iostream>
#include <filesystem>
#include <vector>
#include <regex>
#include <string>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include "BoardCalib.hpp"
#include "FocusCalib.hpp"
#include "spdlog/spdlog.h"
namespace fs = std::filesystem;

// Function to load images from a directory sorted by the integer prefix in their filenames
std::vector<cv::Mat> loadSortedImages(const fs::path &folder_path)
{
    // Vector to store image paths and their corresponding integer index
    std::vector<std::pair<int, fs::path>> image_files;

    // Regular expression to match image files with integer names
    std::regex image_regex(R"((\d+)\.(jpg|jpeg|png|bmp|tiff))", std::regex::icase);

    // Iterate through the directory
    for (const auto &entry: fs::directory_iterator(folder_path))
    {
        if (entry.is_regular_file())
        {
            std::string filename = entry.path().filename().string();
            std::smatch match;

            // Check if the filename matches the regex pattern
            if (std::regex_match(filename, match, image_regex))
            {
                int index = std::stoi(match[1].str());// Convert the extracted number to an integer
                image_files.emplace_back(index, entry.path());
            }
        }
    }

    // Sort the image files based on the integer index
    std::sort(image_files.begin(), image_files.end(), [](const auto &a, const auto &b) {
        return a.first < b.first;
    });

    // Vector to store the images
    std::vector<cv::Mat> images;

    // Load images in order and store them in the vector
    for (const auto &file: image_files)
    {
        cv::Mat image = cv::imread(file.second.string(), cv::IMREAD_COLOR);
        if (image.empty())
        {
            std::cerr << "Error: Could not load image " << file.second << std::endl;
            continue;
        }
        images.push_back(image);
    }

    return images;
}

int main()
{
    fs::path project_dir_path = fs::path(PROJECT_DIR);
    fs::path res_dir_path     = fs::path(RES_DIR);
    fs::path result_dir_path  = fs::path(RESULT_DIR);
    fs::path yaml_config_path = project_dir_path / fs::path("config.yaml");

    YAML::Node config            = YAML::LoadFile(yaml_config_path.string());
    std::string debug_data_dir   = config["debug_data_dir"].as<std::string>();
    fs::path debug_data_dir_path = project_dir_path / fs::path(debug_data_dir);
    spdlog::info("debug_data_dir_path: {}", debug_data_dir_path.string());
    std::regex float_regex(R"([\d]+(\.[\d]+)?)");
    std::vector<fs::path> folder_paths;


    // Iterate through the entries in the parent folder
    for (const auto &entry: fs::directory_iterator(debug_data_dir_path))
    {
        if (entry.is_directory())
        {
            std::smatch match;
            std::string folder_name = entry.path().filename().string();

            // Check if the folder name contains a floating-point number
            if (std::regex_search(folder_name, match, float_regex))
            {
                folder_paths.push_back(entry.path());
            }
        }
    }

    // Function to extract the first floating-point number from the folder path
    auto extract_float_from_path = [](const fs::path &path) {
        std::smatch match;
        std::regex float_regex(R"([\d]+(\.[\d]+)?)");
        std::string folder_name = path.filename().string();
        if (std::regex_search(folder_name, match, float_regex))
        {
            return std::stof(match[0].str());
        }
        return 0.0f;// Default value if no floating-point number is found
    };

    // Sort the vector of paths based on the extracted floating-point numbers
    std::sort(folder_paths.begin(), folder_paths.end(),
              [&extract_float_from_path](const fs::path &a, const fs::path &b) {
                  return extract_float_from_path(a) < extract_float_from_path(b);
              });

    spdlog::info("Number of folders: {}", folder_paths.size());

    BoardCalib board_calib(yaml_config_path.string());
    std::vector<cv::Mat> board_imgs;

    std::string user_input;

    while (1)
    {
        std::cout << "Select [0-" << folder_paths.size() - 1 << "]:" << std::endl;
        int index = 0;
        std::cin >> index;
        index = std::min(std::max(0, index), (int) folder_paths.size() - 1);

        spdlog::info("Selected folder: {}", folder_paths[index].string());

        fs::path folder_path    = folder_paths[index];
        board_imgs              = loadSortedImages(folder_path);
        fs::path calib_xml_path = folder_path / fs::path("calib.xml");
        double encoder_normal_pos;
        board_calib.LoadCalibXML_debug(calib_xml_path.string(), encoder_normal_pos);

        cv::Mat K                                           = board_calib.K;
        cv::Mat dist_coeffs                                 = board_calib.dist_coeffs;
        std::vector<cv::Mat> rvecs                          = board_calib.rvecs_;
        std::vector<cv::Mat> tvces                          = board_calib.tvecs_;
        std::vector<std::vector<cv::Point3f>> object_points = board_calib.object_points_;
        std::vector<std::vector<cv::Point2f>> image_points  = board_calib.image_points_;
        std::cout << "K:\n"
                  << K << "\n\n";
        std::cout << "D:\n"
                  << dist_coeffs << "\n\n";
        std::cout << "Num of images: " << image_points.size() << "\n";
        std::cout << "object_points.size() = " << object_points.size() << "\n";
        std::cout << "image_points.size() = " << image_points.size() << "\n";
        for (int i = 0; i < image_points.size(); i++)
        {
            const std::vector<cv::Point2f> &img_pts = image_points[i];
            std::vector<cv::Point3f> &obj_pts       = object_points[i];
            const cv::Mat &rvec = rvecs[i];
            const cv::Mat &tvec = tvces[i];
            cv::Mat img;
            board_imgs[i].copyTo(img);
            std::vector<cv::Point2f> proj_pts;

            cv::projectPoints(obj_pts, rvec, tvec, K, dist_coeffs, proj_pts);
            for (int j = 0; j < proj_pts.size(); j++)
            {
                cv::circle(img, proj_pts[j], 3, cv::Scalar(0, 255, 0), -1);
                std::cout << "obj_pts[" << j << "]: " << obj_pts[j] << "\n";
                std::cout << "proj_pts[" << j << "]: " << proj_pts[j] << "\n";
                // cv::circle(img, img_pts[j], 7, cv::Scalar(0, 0, 255), 2);
            }
            cv::imshow("debug img", img);
            cv::waitKey(0);
            cv::destroyAllWindows();
        }


        std::cout << "\n Quit? (y/n): ";
        std::cin >> user_input;
        if (user_input == "y")
            break;
    }


    FocusCalib focus_calib;


    return 0;
}