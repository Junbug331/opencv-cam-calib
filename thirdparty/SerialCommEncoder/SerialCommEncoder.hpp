#ifndef SERIAL_COMM_ENCODER_HPP
#define SERIAL_COMM_ENCODER_HPP

#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <atomic>
#include <thread>
#include <vector>
#include <mutex>
#include <utility>

#include <deque>
#include <iomanip>
#include <yaml-cpp/yaml.h>

namespace Vive
{

    class SerialCommEncoder
    {
    private:
        static const std::string serial_port_name_;// Change to your port
        static const int baud_rate_;

        termios options_;
        int fd_;
        std::atomic<bool> running_;
        std::thread serial_thread_;
        std::thread input_thread_;
        std::mutex encoder_val_mtx_;

        uint32_t encoder_value_;
        uint32_t encoder_value_min_;
        uint32_t encoder_value_max_;
        double nomalization_multiplier_ = 1.0;
        bool encoder_value_normalized_  = false;

        std::vector<std::pair<uint32_t, double>> encoder_positions_;


    private:
        void set_terminal_mode(bool enable);
        bool process_serial_data(const std::deque<uint8_t> &buffer);
        void serial_thread_callback(int fd);
        void input_thread_callback();

    public:
        SerialCommEncoder();
        ~SerialCommEncoder();
        void Init(std::string port_name = "/tty/USB0", int baud_rate = 115200);
        void StartSerialComm();
        void StopSerialComm();
        uint32_t GetRawEncoderValue();
        double GetEncoderValue();
        void NormalizeEncoderValue();
        inline void SetMin() { encoder_value_min_ = GetRawEncoderValue(); }
        inline void SetMax() { encoder_value_max_ = GetRawEncoderValue(); }
        void PrintMinMax();
        void SaveDataToFiles(const std::string &filename);
        void SetEncoderPosition(uint32_t pos);
    };
}// namespace Vive


#endif