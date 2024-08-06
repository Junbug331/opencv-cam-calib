#include <iostream>
using std::cout;
using std::endl;
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <cstring>
#include "OpenCVCamCalib.hpp"

namespace fs = std::filesystem;


int main()
{
    fs::path project_dir_path = fs::path(PROJECT_DIR);
    fs::path res_dir_path     = fs::path(RES_DIR);
    fs::path result_dir_path  = fs::path(RESULT_DIR);
    fs::path yaml_config_path = project_dir_path / fs::path("config.yaml");

    YAML::Node config = YAML::LoadFile(yaml_config_path.string());

    std::string strCamModel     = config["cam_model"].as<std::string>();
    std::string strCalibXMLName = config["calib_xml_name"].as<std::string>();
    spdlog::info("strCalibXMLName: {}", strCalibXMLName);

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
    int img_width, img_height;

    if (fs::exists(oCalibXMLFilePath))
    {

        cv::FileStorage fileStorage(oCalibXMLFilePath.string(), cv::FileStorage::READ);
        fileStorage["K"] >> K;
        fileStorage["dist_coeffs"] >> dist_coeffs;
        fileStorage["image_width"] >> img_width;
        fileStorage["image_height"] >> img_height;
        // Release the file storage
        fileStorage.release();
        cout << "K: \n"
             << K << endl
             << endl;
        cout << "distCoeffs:\n"
             << dist_coeffs << endl
             << endl;
        cout << "Image Size:\n";
        cout << "\t Width: " << img_width << ", Height: " << img_height << endl;
        cv::Size oImageSize(img_width, img_height);

        if (cam_model == CAM_MODEL::FISHEYE)
        {
            cv::Mat new_K;
            cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K, dist_coeffs, oImageSize, cv::Matx33d::eye(), new_K, 1.0);
            cout << "new_K\n";
            cout << new_K << endl
                 << endl;
            cv::fisheye::initUndistortRectifyMap(K, dist_coeffs, cv::Mat(), new_K, oImageSize, CV_16SC2, map1, map2);
        }
    }

    cv::VideoCapture cap(0, cv::CAP_V4L2);

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
    if (!cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280))
    {
        spdlog::error("Error setting frame width");
        throw std::runtime_error("Error setting frame width");
    }
    if (!cap.set(cv::CAP_PROP_FRAME_HEIGHT, 800))
    {
        spdlog::error("Error setting frame height");
        throw std::runtime_error("Error setting frame height");
    }
    if (!cap.set(cv::CAP_PROP_FPS, 120))
    {
        spdlog::error("Error setting frame rate");
        throw std::runtime_error("Error setting frame rate");
    }

    cv::Mat img, show_img, undist_img;
    int image_counter = 0;
    bool is_recording = false;
    cv::VideoWriter video_writer;

    double fps = cap.get(cv::CAP_PROP_FPS);
    spdlog::info("FPS: {}", fps);

    cap >> img;
    spdlog::info("Image size: {}x{}", img.cols, img.rows);

    while (1)
    {
        cap >> img;

        if (img.empty())
        {
            spdlog::error("No frame");
            break;
        }


        // cv::Mat imgUndistorted;
        // cv::initUndistortRectifyMap(K, distCoeffs, cv::Mat(), K, cv::Size(nWidth, nHeight), CV_32FC1, mapX, mapY);
        // cv::remap(img, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);
        if (img.channels() < 3)
        {
            cv::cvtColor(img, show_img, cv::COLOR_GRAY2BGR);
        }
        else
        {
            img.copyTo(show_img);
        }

        if (!map1.empty() && !map2.empty())
        {
            img.copyTo(undist_img);
            cv::remap(img, undist_img, map1, map2, cv::INTER_LINEAR);
            cv::imshow("Undistorted", undist_img);
        }

        // Add "RECORDING" text if recording
        if (is_recording)
        {
            cv::putText(show_img, "RECORDING", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        }


        cv::imshow("Original", show_img);
        // cv::imshow("Undistorted", imgUndistorted);


        int key = cv::waitKey(1);

        if (key == 27)
        {
            break;
        }
        else if (key == 32 /* space bar */)
        {
            fs::path image_path = result_dir_path / fs::path(std::to_string(image_counter) + ".png");
            cv::imwrite(image_path.string(), img);
            ++image_counter;

            spdlog::info("Image saved to: {}", image_path.string());
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

    return 0;
}