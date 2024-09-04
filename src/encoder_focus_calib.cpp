#include <iostream>
using std::cin;
using std::cout;
using std::endl;
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>
#include <filesystem>
namespace fs = std::filesystem;
#include <cstring>
#include <chrono>
#include <iomanip>
#include <sstream>

#include "SerialCommEncoder.hpp"
#include "BoardCalib.hpp"
#include "FocusCalib.hpp"

using namespace Vive;

std::string getCurrentDateTime();

void RunCalibrationViaBoard(int idx, double normal_pos);

int main()
{
    fs::path project_dir_path = fs::path(PROJECT_DIR);
    fs::path res_dir_path     = fs::path(RES_DIR);
    fs::path result_dir_path  = fs::path(RESULT_DIR);
    fs::path yaml_config_path = project_dir_path / fs::path("config.yaml");

    YAML::Node config = YAML::LoadFile(yaml_config_path.string());
    BoardCalib board_calib(yaml_config_path.string());

    FocusCalib focus_calib;

    // Encoder Setting
    SerialCommEncoder encoder_serial;
    encoder_serial.Init("/dev/ttyUSB0", B115200);
    encoder_serial.StartSerialComm();

    cout << "Encoder Focus Calibration Program\n\n\n";

    std::string user_input = "";
    while (true)
    {
        std::cout << "Command: ";
        cin >> user_input;
        if (user_input == "q" || user_input == "Q")
        {
            encoder_serial.StopSerialComm();
            break;
        }
        else if (user_input == "r" || user_input == "R")
        {
            std::cout << "Raw Encoder value: " << encoder_serial.GetRawEncoderValue() << std::endl;
        }
        else if (user_input == "s" || user_input == "S")
        {
            encoder_serial.SetMin();
            std::cout << "Set min value: " << encoder_serial.GetRawEncoderValue() << std::endl;
        }
        else if (user_input == "e" || user_input == "E")
        {
            encoder_serial.SetMax();
            std::cout << "Set max value: " << encoder_serial.GetRawEncoderValue() << std::endl;
        }
        else if (user_input == "n" || user_input == "N")
        {
            encoder_serial.NormalizeEncoderValue();
            std::cout << "Normalizing Encoder value..." << std::endl;
        }
        else if (user_input == "pos" || user_input == "POS")
        {
            int pos;
            double normalized_encoder_value = encoder_serial.GetEncoderValue();
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2) << normalized_encoder_value;

            cout << "\n-----------------------------------------------------\n";
            std::cout << "Board Calibration at normalized encoder position: " << oss.str() << "\n\n";

            // Calibrate Camera
            std::string boardimgs_dir_name = "boardimgs_encoder_" + oss.str();
            fs::path boardiamgd_dir_path   = result_dir_path / fs::path(boardimgs_dir_name);

            auto [K, dist_coeffs] = board_calib.RunBoardCalibration_debug(boardiamgd_dir_path.string());

            fs::path calib_file_path = boardiamgd_dir_path / fs::path("calib.xml");
            board_calib.SaveCalibXML_debug(calib_file_path.string(), normalized_encoder_value);
            board_calib.LoadCalibXML_debug(calib_file_path.string(), normalized_encoder_value);

            // Add encoder value and calib data to focus_calib
            focus_calib.AddCalibData(encoder_serial.GetEncoderValue(), {K, dist_coeffs});

            std::cout << "\nBoard calibration is done.\n\n\n\n";
        }
        else if (user_input == "delete")
        {
            focus_calib.DeletePrev();
            std::cout << "Previous calibration data is deleted.\n";
        }
        else if (user_input == "calib")
        {
            focus_calib.SortCalibData();
            fs::path calib_data_path = result_dir_path / fs::path("calib_data.yaml");
            focus_calib.SaveCalibData(calib_data_path.string());
            std::cout << "Calibration data is saved.\n";
        }
        else if (user_input == "interpol")
        {
            auto [interpolated_K, interpolated_dist_coeffs] = focus_calib.GetInterpolatedCalibData(encoder_serial.GetEncoderValue());
            std::cout << "Interpolated K:\n"
                      << interpolated_K << std::endl;
            std::cout << "Interpolated dist_coeffs:\n"
                      << interpolated_dist_coeffs << std::endl;
        }
        else if (user_input == "load")
        {
            fs::path calib_data_path = res_dir_path / fs::path("calib_data.yaml");
            focus_calib.LoadCalibData(calib_data_path.string());
            std::cout << "Calibration data is loaded.\n";
        }
        else
        {
            encoder_serial.PrintMinMax();
            std::cout << std::setprecision(9);
            std::cout << "Normalized value: " << encoder_serial.GetEncoderValue() << " "
                      << "Raw value: " << encoder_serial.GetRawEncoderValue() << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    encoder_serial.SaveDataToFiles("encoder_data.yaml");

    spdlog::info("Prgram is terminated.");

    return 0;
}

std::string getCurrentDateTime()
{
    auto now             = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_time), "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}