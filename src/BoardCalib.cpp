#include <iostream>
using std::cout;
using std::endl;
#include <filesystem>
namespace fs = std::filesystem;

#include <yaml-cpp/yaml.h>
#include "spdlog/spdlog.h"

#include "BoardCalib.hpp"

BoardCalib::BoardCalib(const std::string &a_rYamlFile)
{
    YAML::Node config = YAML::LoadFile(a_rYamlFile);

    yaml_file        = a_rYamlFile;
    board_width_     = config["board_width"].as<int>();
    board_height_    = config["board_height"].as<int>();
    img_width_       = config["img_width"].as<int>();
    img_height_      = config["img_height"].as<int>();
    device_id_       = config["device_id"].as<int>();
    fps_             = config["fps"].as<double>();
    pattern_size_mm_ = config["pattern_size_mm"].as<double>();// diameter for circles grid

    std::string str_board_pattern = config["board_pattern"].as<std::string>();
    if (str_board_pattern == "circles_grid")
        board_pattern_ = BOARD_PATTERN::CIRCLES_GRID;
    else if (str_board_pattern == "asymmetric_circles_grid")
        board_pattern_ = BOARD_PATTERN::ASYMMETRIC_CIRCLES_GRID;
    else
        board_pattern_ = BOARD_PATTERN::CHESSBOARD;

    std::string str_cam_model = config["cam_model"].as<std::string>();
    if (str_cam_model == "pinhole")
        cam_model_ = CAM_MODEL::PINHOLE;
    else
        cam_model_ = CAM_MODEL::FISHEYE;


    spdlog::info("Camera device opened: {}", device_id_);
    spdlog::info("Camera model: {}", str_cam_model);
    spdlog::info("Board pattern: {}", str_board_pattern);
    spdlog::info("Board width: {}", board_width_);
    spdlog::info("Board height: {}", board_height_);
}

BoardCalib::~BoardCalib()
{
}

void BoardCalib::RunBoardCalibration(const std::string &boardimg_dir)
{
    cap_ = cv::VideoCapture(device_id_, cv::CAP_V4L);

    if (!cap_.isOpened())
    {
        spdlog::error("Cannot open camera device.");
        throw std::runtime_error("Cannot open camera device.");
    }

    // Set camera properties
    if (!cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G')))
    {
        spdlog::error("Error setting FOURCC");
        throw std::runtime_error("Error setting FOURCC");
    }
    if (!cap_.set(cv::CAP_PROP_FRAME_WIDTH, img_width_))
    {
        spdlog::error("Error setting frame width");
        throw std::runtime_error("Error setting frame width");
    }
    if (!cap_.set(cv::CAP_PROP_FRAME_HEIGHT, img_height_))
    {
        spdlog::error("Error setting frame height");
        throw std::runtime_error("Error setting frame height");
    }
    if (!cap_.set(cv::CAP_PROP_FPS, fps_))
    {
        spdlog::error("Error setting frame rate");
        throw std::runtime_error("Error setting frame rate");
    }

    spdlog::info("frame width: {}", cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    spdlog::info("frame height: {}", cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    spdlog::info("FPS: {}", cap_.get(cv::CAP_PROP_FPS));


    fs::path image_dir_path = fs::path(boardimg_dir);
    std::filesystem::create_directory(image_dir_path);

    cv::Mat img, gray, show_img;
    int image_count = 0;

    cap_ >> img;
    img_size = cv::Size(img.cols, img.rows);
    std::vector<std::vector<cv::Point2f>> image_points;
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

    cv::namedWindow("Calibration");

    while (1)
    {
        cap_ >> img;
        if (img.empty())
        {
            spdlog::error("Cannot capture image.");
            break;
        }
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(img, show_img, cv::COLOR_BGR2RGB);

        std::vector<cv::Point2f> corners;// or centers for circles grid

        if (board_pattern_ == BOARD_PATTERN::CHESSBOARD &&
            cv::findChessboardCorners(gray,
                                      cv::Size(board_width_, board_height_),
                                      corners,
                                      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK))
        {
            cv::drawChessboardCorners(show_img,
                                      cv::Size(board_width_, board_height_),
                                      corners,
                                      true);
        }
        else if (board_pattern_ == BOARD_PATTERN::CIRCLES_GRID &&
                 cv::findCirclesGrid(gray,
                                     cv::Size(board_width_, board_height_),
                                     corners,
                                     cv::CALIB_CB_SYMMETRIC_GRID,
                                     detector))
        {
            cv::drawChessboardCorners(show_img,
                                      cv::Size(board_width_, board_height_),
                                      corners,
                                      true);
        }

        cv::imshow("Calibration", show_img);

        int key = cv::waitKey(5);

        if (key == 27)
            break;
        else if (key == 32 /*Space Bar*/)
        {
            if (corners.size() == board_width_ * board_height_)
            {
                std::string img_name = std::to_string(image_points.size()) + ".png";
                fs::path image_path  = image_dir_path / img_name;
                cv::imwrite(image_path.string(), img);
                spdlog::info("{}, Image saved: {}", image_points.size(), image_path.string());

                image_points.push_back(corners);
            }
        }
    }

    cv::destroyAllWindows();

    cap_.release();

    if (image_points.size() < 3)
    {
        spdlog::error("Not enough images for calibration.");
        return;
    }

    double rms = ChessBoardCalibration(board_width_,
                                       board_height_,
                                       pattern_size_mm_,
                                       image_points,
                                       this->img_size,
                                       this->K,
                                       this->dist_coeffs,
                                       this->rvecs_,
                                       this->tvecs_,
                                       cam_model_,
                                       false);
    spdlog::info("Board calibration RMS: {}", rms);
    cout << "img_size: " << img_size << "\n\n";
    cout << "K:\n"
         << this->K << "\n\n";
    cout << "dist_coeffs:\n"
         << this->dist_coeffs << "\n\n";
}

std::tuple<cv::Mat, cv::Mat> BoardCalib::RunBoardCalibration_debug(const std::string &boardimg_dir)
{
    cap_ = cv::VideoCapture(device_id_, cv::CAP_V4L);

    if (!cap_.isOpened())
    {
        spdlog::error("Cannot open camera device.");
        throw std::runtime_error("Cannot open camera device.");
    }

    // Set camera properties
    if (!cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G')))
    {
        spdlog::error("Error setting FOURCC");
        throw std::runtime_error("Error setting FOURCC");
    }
    if (!cap_.set(cv::CAP_PROP_FRAME_WIDTH, img_width_))
    {
        spdlog::error("Error setting frame width");
        throw std::runtime_error("Error setting frame width");
    }
    if (!cap_.set(cv::CAP_PROP_FRAME_HEIGHT, img_height_))
    {
        spdlog::error("Error setting frame height");
        throw std::runtime_error("Error setting frame height");
    }
    if (!cap_.set(cv::CAP_PROP_FPS, fps_))
    {
        spdlog::error("Error setting frame rate");
        throw std::runtime_error("Error setting frame rate");
    }

    spdlog::info("frame width: {}", cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    spdlog::info("frame height: {}", cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    spdlog::info("FPS: {}", cap_.get(cv::CAP_PROP_FPS));


    fs::path image_dir_path = fs::path(boardimg_dir);
    std::filesystem::create_directory(image_dir_path);

    cv::Mat img, gray, show_img;
    int image_count = 0;

    cap_ >> img;
    img_size = cv::Size(img.cols, img.rows);
    std::vector<std::vector<cv::Point2f>> image_points;
    cv::Ptr<cv::ORB> orb = cv::ORB::create(1000);
    cv::SimpleBlobDetector::Params params;
    // Lower the starting threshld value since the image is dark
    params.thresholdStep = 2;  // Decrease the step for more precise thresholding
    params.minThreshold  = 5;  // Lower value to start detecting darker blobs
    params.maxThreshold  = 150;// Adjust if necessary

    // Filter by color (0 for dark blobs)
    params.filterByColor = true;
    params.blobColor     = 0;// Detect dark blobs

    // Filter by area (you can adjust these values based on your specific blob size)
    params.filterByArea = true;
    params.minArea      = 50;  // Minimum area of the blob
    params.maxArea      = 5000;// Maximum area of the blob

    // Filter by circularity (optional, if you want only circular blobs)
    params.filterByCircularity = true;
    params.minCircularity      = 0.5f;// Adjust based on how circular your blobs are

    // Filter by convexity (optional)
    params.filterByConvexity = true;
    params.minConvexity      = 0.6f;

    // Filter by inertia (optional)
    params.filterByInertia = false;
    params.minInertiaRatio = 0.5f;

    // Create the SimpleBlobDetector with the parameters
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    cv::namedWindow("Calibration");

    while (1)
    {
        cap_ >> img;
        cv::flip(img, img, 0);
        cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
        if (img.empty())
        {
            spdlog::error("Cannot capture image.");
            break;
        }
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(img, show_img, cv::COLOR_BGR2RGB);

        std::vector<cv::Point2f> corners;// or centers for circles grid

        if (board_pattern_ == BOARD_PATTERN::CHESSBOARD &&
            cv::findChessboardCorners(gray,
                                      cv::Size(board_width_, board_height_),
                                      corners,
                                      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK))
        {
            cv::drawChessboardCorners(show_img,
                                      cv::Size(board_width_, board_height_),
                                      corners,
                                      true);
        }
        else if (board_pattern_ == BOARD_PATTERN::CIRCLES_GRID &&
                 cv::findCirclesGrid(gray,
                                     cv::Size(board_width_, board_height_),
                                     corners,
                                     cv::CALIB_CB_SYMMETRIC_GRID,
                                     detector))
        {
            cv::drawChessboardCorners(show_img,
                                      cv::Size(board_width_, board_height_),
                                      corners,
                                      true);
        }

        cv::imshow("Calibration", show_img);

        int key = cv::waitKey(5);

        if (key == 27)
            break;
        else if (key == 32 /*Space Bar*/)
        {
            if (corners.size() == board_width_ * board_height_)
            {
                std::string img_name = std::to_string(image_points.size()) + ".png";
                fs::path image_path  = image_dir_path / img_name;
                cv::imwrite(image_path.string(), img);
                spdlog::info("{}, Image saved: {}", image_points.size(), image_path.string());

                image_points.push_back(corners);
            }
        }
    }

    cv::destroyAllWindows();

    cap_.release();

    if (image_points.size() < 3)
    {
        spdlog::error("Not enough images for calibration.");
        return {cv::Mat(), cv::Mat()};
    }

    std::vector<cv::Point3f> objPoint;
    for (int i = board_height_ - 1; i >= 0; --i)
    {
        for (int j = 0; j < board_width_; ++j)
        {
            objPoint.emplace_back(j * pattern_size_mm_, i * pattern_size_mm_, 0.0);
        }
    }
    std::vector<std::vector<cv::Point3f>> objPoints(image_points.size(), objPoint);

    cv::Mat res_K, res_dist_coeffs;
    double rms = ChessBoardCalibration(board_width_,
                                       board_height_,
                                       pattern_size_mm_,
                                       image_points,
                                       this->img_size,
                                       res_K,
                                       res_dist_coeffs,
                                       this->rvecs_,
                                       this->tvecs_,
                                       cam_model_,
                                       false);
    spdlog::info("Board calibration RMS: {}", rms);

    object_points_ = std::move(objPoints);
    image_points_  = std::move(image_points);

    res_K.copyTo(this->K);
    res_dist_coeffs.copyTo(this->dist_coeffs);

    cout << "img_size: " << img_size << "\n\n";
    cout << "K:\n"
         << this->K << "\n\n";
    cout << "dist_coeffs:\n"
         << this->dist_coeffs << "\n\n";

    return {res_K, res_dist_coeffs};
}

void BoardCalib::SaveCalibXML(const std::string &calib_file)
{
    if (K.empty() || dist_coeffs.empty())
    {
        spdlog::error("Calibration parameters are empty.");
        return;
    }

    cv::FileStorage cv_fs(calib_file, cv::FileStorage::WRITE);
    cv_fs << "K" << K;
    cv_fs << "dist_coeffs" << dist_coeffs;
    cv_fs << "image_width" << img_size.width;
    cv_fs << "image_height" << img_size.height;
    cv_fs.release();
}

void BoardCalib::LoadCalibXML_debug(const std::string &calib_file, double &encoder_value)
{
    cv::FileStorage cv_fs(calib_file, cv::FileStorage::READ);

    if (!cv_fs.isOpened())
    {
        spdlog::error("Failed to open the calibration file: {}", calib_file);
        return;
    }

    K           = cv::Mat();
    dist_coeffs = cv::Mat();

    // Load basic calibration parameters
    cv_fs["encoder_value"] >> encoder_value;
    cv_fs["K"] >> K;
    cv_fs["dist_coeffs"] >> dist_coeffs;
    cv_fs["image_width"] >> img_size.width;
    cv_fs["image_height"] >> img_size.height;

    // Load rotation vectors
    rvecs_.clear();
    cv::FileNode rvecsNode = cv_fs["rvecs"];
    for (const auto &rvecNode: rvecsNode)
    {
        cv::Mat rvec;
        rvecNode >> rvec;
        rvecs_.push_back(rvec);
    }

    // Load translation vectors
    tvecs_.clear();
    cv::FileNode tvecsNode = cv_fs["tvecs"];
    for (const auto &tvecNode: tvecsNode)
    {
        cv::Mat tvec;
        tvecNode >> tvec;
        tvecs_.push_back(tvec);
    }

    // Load object points
    object_points_.clear();
    cv::FileNode objectPointsNode = cv_fs["object_points"];
    for (const auto &objectPointSetNode: objectPointsNode)
    {
        std::vector<cv::Point3f> objectPoints;
        for (const auto &pointNode: objectPointSetNode)
        {
            cv::Point3f point;
            pointNode >> point;
            objectPoints.push_back(point);
        }
        object_points_.push_back(objectPoints);
    }

    // Load image points
    image_points_.clear();
    cv::FileNode imagePointsNode = cv_fs["image_points"];
    for (const auto &imagePointSetNode: imagePointsNode)
    {
        std::vector<cv::Point2f> imagePoints;
        for (const auto &pointNode: imagePointSetNode)
        {
            cv::Point2f point;
            pointNode >> point;
            imagePoints.push_back(point);
        }
        image_points_.push_back(imagePoints);
    }

    std::cout << "Loading calibration parameters from " << calib_file << std::endl;
    std::cout << "encoder_value: " << encoder_value << std::endl;
    std::cout << "K:\n"
              << K << "\n\n";
    std::cout << "Distortion coefficients:\n"
              << dist_coeffs << "\n\n";
    std::cout << "Number of rvecs: " << rvecs_.size() << std::endl;
    std::cout << "rvects_[0]:\n"
              << rvecs_[0] << "\n\n";
    std::cout << "Number of tvecs: " << tvecs_.size() << std::endl;
    std::cout << "tvecs_.back() = \n"
              << tvecs_.back() << "\n\n";
    std::cout << "Number of object points: " << object_points_.size() << std::endl;
    std::cout << "Number of Image points: " << image_points_.size() << std::endl;

    cv_fs.release();
}

void BoardCalib::SaveCalibXML_debug(const std::string &calib_file, const double &encoder_value)
{
    if (K.empty() || dist_coeffs.empty())
    {
        spdlog::error("Calibration parameters are empty.");
        return;
    }

    cv::FileStorage cv_fs(calib_file, cv::FileStorage::WRITE);

    // Save basic calibration parameters
    cv_fs << "encoder_value" << encoder_value;
    cv_fs << "K" << K;
    cv_fs << "dist_coeffs" << dist_coeffs;
    cv_fs << "image_width" << img_size.width;
    cv_fs << "image_height" << img_size.height;

    // Save rotation vectors
    cv_fs << "rvecs"
          << "[";
    for (const auto &rvec: rvecs_)
    {
        cv_fs << rvec;
    }
    cv_fs << "]";

    // Save translation vectors
    cv_fs << "tvecs"
          << "[";
    for (const auto &tvec: tvecs_)
    {
        cv_fs << tvec;
    }
    cv_fs << "]";

    // Save object points
    cv_fs << "object_points"
          << "[";
    for (const auto &object_point_set: object_points_)
    {
        cv_fs << "[";
        for (const auto &point: object_point_set)
        {
            cv_fs << point;
        }
        cv_fs << "]";
    }
    cv_fs << "]";

    // Save image points
    cv_fs << "image_points"
          << "[";
    for (const auto &image_point_set: image_points_)
    {
        cv_fs << "[";
        for (const auto &point: image_point_set)
        {
            cv_fs << point;
        }
        cv_fs << "]";
    }
    cv_fs << "]";

    cv_fs.release();
}
