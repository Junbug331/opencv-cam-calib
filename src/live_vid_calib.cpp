#include <iostream>
using std::cout;
using std::endl;
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <cstring>
#include <chrono>
#include <iomanip>
#include <sstream>
#include "OpenCVCamCalib.hpp"

namespace fs = std::filesystem;

std::string getCurrentDateTime()
{
    auto now             = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_time), "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}


int main()
{
    fs::path project_dir_path = fs::path(PROJECT_DIR);
    fs::path res_dir_path     = fs::path(RES_DIR);
    fs::path result_dir_path  = fs::path(RESULT_DIR);
    fs::path yaml_config_path = project_dir_path / fs::path("config.yaml");

    YAML::Node config = YAML::LoadFile(yaml_config_path.string());

    std::string strCamModel     = config["cam_model"].as<std::string>();
    std::string strCalibXMLName = config["calib_xml_name"].as<std::string>();
    std::string str_board_type  = config["board_pattern"].as<std::string>();
    double balance              = config["balance"].as<double>();
    int board_width             = config["board_width"].as<int>();
    int board_height            = config["board_height"].as<int>();
    int img_width               = config["image_width"].as<int>();
    int img_height              = config["image_height"].as<int>();
    int device_id               = config["device_id"].as<int>();
    double fps                  = config["fps"].as<double>();
    double square_size_mm       = config["square_size_mm"].as<double>();

    std::cout << "Enter name for calibration xml file(.xml): ";
    std::cin >> strCalibXMLName;

    std::string img_dir_name;
    std::cout << "Enter name for board image directory(save under result/): ";
    std::cin >> img_dir_name;

    // Board Pattern
    BOARD_PATTERN board_pattern = BOARD_PATTERN::CHESSBOARD;
    if (str_board_type == "chessboard")
    {
        board_pattern = BOARD_PATTERN::CHESSBOARD;
    }
    else if (str_board_type == "circles_grid")
    {
        board_pattern = BOARD_PATTERN::CIRCLES_GRID;
    }
    else if (str_board_type == "asymmetric_circles_grid")
    {
        board_pattern = BOARD_PATTERN::ASYMMETRIC_CIRCLES_GRID;
    }
    else
    {
        spdlog::error("Invalid board pattern");
        throw std::runtime_error("Invalid board pattern");
    }

    // Camera Model
    CAM_MODEL cam_model = CAM_MODEL::PINHOLE;
    if (strCamModel == "fisheye")
    {
        cam_model = CAM_MODEL::FISHEYE;
    }
    else if (strCamModel == "pinhole")
    {
        cam_model = CAM_MODEL::PINHOLE;
    }
    else
    {
        spdlog::error("Invalid camera model");
        throw std::runtime_error("Invalid camera model");
    }

    fs::path oCalibXMLFilePath = res_dir_path / fs::path(strCalibXMLName);
    cv::Mat K, dist_coeffs;
    cv::Mat map1, map2;
    cv::VideoCapture cap(device_id, cv::CAP_V4L2);

    // Create result directory of current data-time
    std::string curr_date_time = getCurrentDateTime();
    spdlog::info("Current date-time: {}", curr_date_time);

    if (!cap.isOpened())
    {
        spdlog::error("Error opening video stream or file");
        throw std::runtime_error("Error opening video stream or file");
    }

    // Set camera properties
    if (!cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G')))
    {
        spdlog::error("Error setting FOURCC");
        throw std::runtime_error("Error setting FOURCC");
    }
    if (!cap.set(cv::CAP_PROP_FRAME_WIDTH, img_width))
    {
        spdlog::error("Error setting frame width");
        throw std::runtime_error("Error setting frame width");
    }
    if (!cap.set(cv::CAP_PROP_FRAME_HEIGHT, img_height))
    {
        spdlog::error("Error setting frame height");
        throw std::runtime_error("Error setting frame height");
    }
    if (!cap.set(cv::CAP_PROP_FPS, fps))
    {
        spdlog::error("Error setting frame rate");
        throw std::runtime_error("Error setting frame rate");
    }

    cv::Mat img, show_img, gray;
    cv::Mat show_img_gray_res, show_img_binary_res;
    int image_counter = 0;
    bool is_recording = false;
    cv::VideoWriter video_writer;
    cv::Size img_size;

    double _fps = cap.get(cv::CAP_PROP_FPS);
    spdlog::info("FPS: {}", _fps);

    cap >> img;
    spdlog::info("Image size: {}x{}", img.cols, img.rows);
    img_size = cv::Size(img.cols, img.rows);

    std::vector<std::vector<cv::Point2f>> image_points;

    fs::path image_dir_path = result_dir_path / img_dir_name;
    if (std::filesystem::create_directory(image_dir_path))
        std::cout << "Directory created: " << image_dir_path << std::endl;
    else
        std::cout << "Failed to create directory: " << image_dir_path << std::endl;

    spdlog::info("board pattern: {}", str_board_type);
    spdlog::info("board width: {}", board_width);
    spdlog::info("board height: {}", board_height);

    cv::Ptr<cv::ORB> orb = cv::ORB::create(1000);
    cv::SimpleBlobDetector::Params params;
    // Lower the starting threshld value since the image is dark
    params.thresholdStep = 5;  // Decrease the step for more precise thresholding
    params.minThreshold  = 10; // Lower value to start detecting darker blobs
    params.maxThreshold  = 200;// Adjust if necessary

    // Filter by color (0 for dark blobs)
    params.filterByColor = true;
    params.blobColor     = 0;// Detect dark blobs

    // Filter by area (you can adjust these values based on your specific blob size)
    params.filterByArea = true;
    params.minArea      = 50;  // Minimum area of the blob
    params.maxArea      = 5000;// Maximum area of the blob

    // Filter by circularity (optional, if you want only circular blobs)
    params.filterByCircularity = true;
    params.minCircularity      = 0.7f;// Adjust based on how circular your blobs are

    // Filter by convexity (optional)
    params.filterByConvexity = true;
    params.minConvexity      = 0.8f;

    // Filter by inertia (optional)
    params.filterByInertia = true;
    params.minInertiaRatio = 0.5f;

    // Create the SimpleBlobDetector with the parameters
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    spdlog::info("XML path: {}", oCalibXMLFilePath.string());

    while (1)
    {
        cap >> img;

        if (img.empty())
        {
            spdlog::error("No frame");
            break;
        }

        // show_img (BGR)
        if (img.channels() < 3)
            cv::cvtColor(img, show_img, cv::COLOR_GRAY2BGR);
        else
        {
            img.copyTo(show_img);
            img.copyTo(show_img_gray_res);
        }

        // gray (GRAY)
        if (img.channels() >= 3)
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        else
            gray = img.clone();

        // Add "RECORDING" text if recording
        if (is_recording)
        {
            cv::putText(show_img, "RECORDING", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        }

        // Find checkerboard
        std::vector<cv::Point2f> corner_points;
        if (board_pattern == BOARD_PATTERN::CHESSBOARD &&
            cv::findChessboardCorners(gray,
                                      cv::Size(board_width, board_height),
                                      corner_points,
                                      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK))
        {
            cv::drawChessboardCorners(show_img,
                                      cv::Size(board_width, board_height),
                                      corner_points,
                                      true);
        }
        else if (board_pattern == BOARD_PATTERN::CIRCLES_GRID)
        {
            bool gray_result = cv::findCirclesGrid(gray,
                                                   cv::Size(board_width, board_height),
                                                   corner_points,
                                                   cv::CALIB_CB_SYMMETRIC_GRID,
                                                   detector);
            if (gray_result)
            {
                cv::drawChessboardCorners(show_img_gray_res,
                                          cv::Size(board_width, board_height),
                                          corner_points,
                                          true);
            }
        }

        cv::imshow("Original", show_img);
        // cv::imshow("Gray Result", show_img_gray_res);
        // cv::imshow("Undistorted", imgUndistorted);


        int key = cv::waitKey(1);

        if (key == 27)
        {
            break;
        }
        else if (key == 32 /* space bar */)
        {
            if (corner_points.size() == board_height * board_width)
            {
                std::string img_name = std::to_string(image_points.size()) + ".png";
                fs::path image_path  = image_dir_path / fs::path(img_name);
                cv::imwrite(image_path.string(), img);
                spdlog::info("Image saved to: {}", image_path.string());

                image_points.push_back(corner_points);
                spdlog::info("Image points: {}", image_points.size());
            }
            // fs::path image_path = result_dir_path / fs::path(std::to_string(image_counter) + ".png");
            // cv::imwrite(image_path.string(), img);
            // ++image_counter;
            // spdlog::info("Image saved to: {}", image_path.string());
        }
        else if (key == 'r')// 'r' key to start recording
        {
            if (!is_recording)
            {
                fs::path video_path = result_dir_path / fs::path("recorded_video.avi");
                video_writer.open(video_path.string(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, img.size());
                if (!video_writer.isOpened())
                {
                    spdlog::error("Error opening video writer");
                }
                else
                {
                    is_recording = true;
                    spdlog::info("Started recording video to: {}", video_path.string());
                }
            }
        }
        else if (key == 's')// 's' key to stop recording
        {
            if (is_recording)
            {
                is_recording = false;
                video_writer.release();
                spdlog::info("Stopped recording video");
            }
        }

        if (is_recording)
        {
            video_writer.write(img);
        }
    }

    cap.release();

    if (image_points.size() < 10)
    {
        spdlog::error("Not enough images");
    }
    else
    {
        std::vector<cv::Mat> rvecs, tvecs;
        double rms = ChessBoardCalibration(board_width,
                                           board_height,
                                           square_size_mm,
                                           image_points,
                                           img_size,
                                           K,
                                           dist_coeffs,
                                           rvecs, tvecs,
                                           cam_model,
                                           true);
        spdlog::info("RMS: {}", rms);
        spdlog::info("XML path: {}", oCalibXMLFilePath.string());
        cv::FileStorage fileStorage(oCalibXMLFilePath.string(), cv::FileStorage::WRITE);

        fileStorage << "K" << K;
        fileStorage << "dist_coeffs" << dist_coeffs;
        fileStorage << "image_width" << img_size.width;
        fileStorage << "image_height" << img_size.height;

        // Release the file storage
        fileStorage.release();

        spdlog::info("Checker board calib finished");
        cout << "K: \n"
             << K << endl
             << endl;
        cout << "dist_coeffs: \n"
             << dist_coeffs << endl;
    }

    return 0;
}