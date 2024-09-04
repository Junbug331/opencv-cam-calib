#include <iostream>
using std::cout;
using std::endl;
#include "OpenCVCamCalib.hpp"

#include <filesystem>
namespace fs = std::filesystem;
#include <iostream>
#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>
#include <spdlog/spdlog.h>


int main()
{
    // Define path
    fs::path oProjectDirPath(PROJECT_DIR);
    fs::path oRESdirPath(RES_DIR);
    fs::path oYAMLconfigPath                  = oProjectDirPath / fs::path("config.yaml");
    YAML::Node config                         = YAML::LoadFile(oYAMLconfigPath.string());
    int nCheckerboardWidth                    = config["checkerboard_width"].as<int>();
    int nCheckerboardHeight                   = config["checkerboard_height"].as<int>();
    double dSqureSize_mm                      = config["square_size_mm"].as<double>();
    double circle_diameter                    = config["circle_diameter"].as<double>();
    std::string strCheckerBoardsImagesDirName = config["checkerboard_images_dir"].as<std::string>();
    std::string strCamModel                   = config["cam_model"].as<std::string>();
    std::string strCalibXMLName               = config["calib_xml_name"].as<std::string>();
    int circle_grid_rows                      = config["circle_grid_rows"].as<int>();
    int circle_grid_cols                      = config["circle_grid_cols"].as<int>();
    bool bShowDebug                           = config["show_debug"].as<bool>();
    fs::path oCheckBoardImagesDirPath         = oRESdirPath / fs::path(strCheckerBoardsImagesDirName);
    fs::path oCalibXMLFilePath                = oRESdirPath / fs::path(strCalibXMLName);
    std::string str_board_pattern             = config["board_pattern"].as<std::string>();
    spdlog::info("oCheckBoardImagesDirPath: {}", oCheckBoardImagesDirPath.string());
    spdlog::info("oCalibXMLFilePath: {}", oCalibXMLFilePath.string());
    spdlog::info("strCamModel: {}", strCamModel);
    spdlog::info("show_debug: {}", bShowDebug);

    // Board Pattern
    BOARD_PATTERN board_pattern = BOARD_PATTERN::CHESSBOARD;
    if (str_board_pattern == "chessboard")
    {
        board_pattern = BOARD_PATTERN::CHESSBOARD;
    }
    else if (str_board_pattern == "circles_grid")
    {
        board_pattern = BOARD_PATTERN::CIRCLES_GRID;
    }
    else if (str_board_pattern == "asymmetric_circles_grid")
    {
        board_pattern = BOARD_PATTERN::ASYMMETRIC_CIRCLES_GRID;
    }
    else
    {
        spdlog::error("Invalid board pattern");
        throw std::runtime_error("Invalid board pattern");
    }


    CAM_MODEL eCamModel = CAM_MODEL::PINHOLE;

    if (strCamModel == "fisheye")
    {
        eCamModel = CAM_MODEL::FISHEYE;
    }
    else if (strCamModel == "pinhole")
    {
        eCamModel = CAM_MODEL::PINHOLE;
    }
    else
    {
        spdlog::error("Invalid camera model");
        throw std::runtime_error("Invalid camera model");
    }

    cv::Mat K, mapX, mapY;
    int nWidth, nHeight;
    cv::Mat distCoeffs;
    double dRmsReprojectionError;

    if (fs::exists(oCheckBoardImagesDirPath))
    {
        dRmsReprojectionError =
                cvCheckerboardCalibration(board_pattern,
                                          (board_pattern == BOARD_PATTERN::CHESSBOARD) ? nCheckerboardWidth : circle_grid_cols,
                                          (board_pattern == BOARD_PATTERN::CHESSBOARD) ? nCheckerboardHeight : circle_grid_rows,
                                          (board_pattern == BOARD_PATTERN::CHESSBOARD) ? dSqureSize_mm : circle_diameter,
                                          oCheckBoardImagesDirPath.string(),
                                          K, distCoeffs,
                                          nWidth,
                                          nHeight,
                                          eCamModel,
                                          bShowDebug);

        cv::FileStorage fileStorage(oCalibXMLFilePath.string(), cv::FileStorage::WRITE);
        fileStorage << "K" << K;
        fileStorage << "dist_coeffs" << distCoeffs;
        fileStorage << "image_width" << nWidth;
        fileStorage << "image_height" << nHeight;

        // Release the file storage
        fileStorage.release();

        spdlog::info("Checker board calib finished");
        cout << "K: \n"
             << K << endl
             << endl;
        cout << "distCoeffs:\n"
             << distCoeffs << endl
             << endl;
        cout << "Image Size:\n";
        cout << "\t Width: " << nWidth << ", Height: " << nHeight << endl;
        cout << "RMS Reprojection Error: " << dRmsReprojectionError << endl;
    }


    return 0;
}