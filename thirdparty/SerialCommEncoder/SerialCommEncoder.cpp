#include "SerialCommEncoder.hpp"

namespace Vive
{
    // Static variables
    const std::string SerialCommEncoder::serial_port_name_ = "/dev/ttyUSB1";
    const int SerialCommEncoder::baud_rate_                = B115200;

    void SerialCommEncoder::set_terminal_mode(bool enable)
    {
        static struct termios oldt, newt;
        if (enable)
        {
            tcgetattr(STDIN_FILENO, &oldt);// Save old terminal settings
            newt = oldt;
            newt.c_lflag &= ~(ICANON | ECHO);       // Disable canonical mode and echo
            tcsetattr(STDIN_FILENO, TCSANOW, &newt);// Set new terminal settings
        }
        else
        {
            tcsetattr(STDIN_FILENO, TCSANOW, &oldt);// Restore old terminal settings
        }
        return;
    }

    bool SerialCommEncoder::process_serial_data(const std::deque<uint8_t> &buffer)
    {
        uint32_t integer_value = (buffer[2] << 32) | (buffer[3] << 24) | (buffer[4] << 16) | (buffer[5] << 8) | buffer[6];

        uint8_t checksum = buffer[7];

        // Display raw hex values in one line
        double one_tick_degree      = 360.0 / 65536.0000;
        uint8_t calculated_checksum = buffer[0] + buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5] + buffer[6];
        int mod_result              = integer_value % 65536;
        double degree_value         = static_cast<double>(mod_result) * one_tick_degree;
        // std::cout << "Degree value: " << degree_value << std::endl;

        {
            std::lock_guard<std::mutex> lock(encoder_val_mtx_);
            encoder_value_ = integer_value;
        }

        return true;
    }

    void SerialCommEncoder::serial_thread_callback(int fd)
    {
        std::deque<uint8_t> buffer;
        uint8_t incoming_byte[256];

        while (running_)
        {
            // std::this_thread::sleep_for(std::chrono::milliseconds(90));  // Adjust sleep duration as needed
            int n = read(fd, &incoming_byte, 8);
            if (n > 0)
            {
                // printf("readed %d size UART data \n", n);
                for (int i = 0; i < n; i++)
                    buffer.push_back(incoming_byte[i]);
                if (buffer.size() < 8)
                    continue;
                if (buffer[0] == 0xfd && buffer[1] == 0xfd)
                {//Checking header

                    bool is_good = process_serial_data(buffer);
                    // buffer.clear();
                    tcflush(fd, TCIFLUSH);
                    if (is_good)
                    {
                        buffer.pop_front();
                        buffer.pop_front();
                        buffer.pop_front();
                        buffer.pop_front();
                        buffer.pop_front();
                        buffer.pop_front();
                        buffer.pop_front();
                        buffer.pop_front();
                        // printf("pop 8 buffer. current buffer size: %d\n", buffer.size());
                    }
                    while (!buffer.empty() && buffer[0] != 0xfd)
                    {
                        buffer.pop_front();
                        // printf("pop buffer \n");
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
                else
                {
                    while (!buffer.empty() && buffer[0] != 0xfd)
                    {
                        buffer.pop_front();
                        // printf("pop buffer \n");
                    }
                }
            }
        }
        return;
    }

    void SerialCommEncoder::input_thread_callback()
    {
        set_terminal_mode(true);
        while (running_)
        {
            int ch = getchar();
            if (ch == 'q')
            {
                running_ = false;
            }
        }
        set_terminal_mode(false);
    }

    SerialCommEncoder::SerialCommEncoder()
    {
        encoder_positions_        = std::vector<std::pair<uint32_t, double>>(5);
        encoder_value_normalized_ = false;
        running_                  = false;
        return;
    }

    SerialCommEncoder::~SerialCommEncoder()
    {
        if (serial_thread_.joinable())
        {
            serial_thread_.join();
        }

        return;
    }

    void SerialCommEncoder::Init(std::string port_name, int baud_rate)
    {
        port_name = (port_name.empty()) ? SerialCommEncoder::serial_port_name_ : port_name;
        baud_rate = (baud_rate == 0) ? SerialCommEncoder::baud_rate_ : baud_rate;

        try
        {
            std::cout << "Opening serial port: " << port_name << std::endl;
            if ((fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0)
            {
                std::cerr << "Failed to open the device..." << std::endl;
            }

            memset(&options_, 0, sizeof(options_));
            cfsetospeed(&options_, baud_rate);
            cfsetispeed(&options_, baud_rate);
            options_.c_cflag = baud_rate | CS8 | CREAD | CLOCAL;
            options_.c_iflag = IGNPAR | ICRNL;
            tcflush(fd_, TCIFLUSH);
            tcsetattr(fd_, TCSANOW, &options_);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    void SerialCommEncoder::StartSerialComm()
    {
        running_ = true;

        serial_thread_ = std::thread(&SerialCommEncoder::serial_thread_callback, this, fd_);

        return;
    }

    void SerialCommEncoder::StopSerialComm()
    {
        running_ = false;
        if (serial_thread_.joinable())
        {
            serial_thread_.join();
        }
        std::cout << "Serial thread stopped" << std::endl;
        close(fd_);
        return;
    }

    uint32_t SerialCommEncoder::GetRawEncoderValue()
    {
        std::lock_guard<std::mutex> lock(encoder_val_mtx_);

        return encoder_value_;
    }

    double SerialCommEncoder::GetEncoderValue()
    {
        if (!encoder_value_normalized_)
            return -1.0;

        auto raw_encoder_value = GetRawEncoderValue();

        if (raw_encoder_value < encoder_value_min_)
            return 0.0;

        if (raw_encoder_value > encoder_value_max_)
            return 1.0;

        raw_encoder_value = std::clamp(raw_encoder_value, encoder_value_min_, encoder_value_max_);

        return (raw_encoder_value - encoder_value_min_) * nomalization_multiplier_;
    }

    void SerialCommEncoder::NormalizeEncoderValue()
    {
        if (encoder_value_min_ > encoder_value_max_)
            std::swap(encoder_value_min_, encoder_value_max_);
        encoder_value_normalized_ = true;
        nomalization_multiplier_  = 1.0 / static_cast<double>(encoder_value_max_ - encoder_value_min_);

        encoder_value_normalized_ = true;
        return;
    }

    void SerialCommEncoder::PrintMinMax()
    {
        std::cout << "Min: " << encoder_value_min_ << " Max: " << encoder_value_max_ << std::endl;
        return;
    }

    void SerialCommEncoder::SaveDataToFiles(const std::string &filename)
    {

        /*
            |     |     |     |     | 
            0.0   0.25  0.5   0.75  1.0
        */

        YAML::Node node;
        // Populate the node with data
        node["encoder_value_min"] = encoder_value_min_;
        node["encoder_value_max"] = encoder_value_max_;

        YAML::Node positionsNode;
        for (const auto &position: encoder_positions_)
        {
            YAML::Node posNode;
            posNode["raw_value"]        = position.first;
            posNode["normalized_value"] = position.second;
            positionsNode.push_back(posNode);
        }

        node["encoder_positions"] = positionsNode;

        // Write the YAML data to a file
        std::ofstream fout("encoder_data.yaml");
        fout << node;
        fout.close();

        return;
    }

    void SerialCommEncoder::SetEncoderPosition(uint32_t pos)
    {
        if (pos < 0 || pos >= encoder_positions_.size())
        {
            std::cerr << "Invalid position" << std::endl;
            return;
        }

        encoder_positions_[pos] = std::make_pair(GetRawEncoderValue(), GetEncoderValue());
        std::cout << "Encoder position " << pos << " set to [" << GetRawEncoderValue() << ", " << GetEncoderValue() << "]" << std::endl;

        return;
    }

}// namespace Vive