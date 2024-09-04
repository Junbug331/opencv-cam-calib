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

#include <deque>
#include <iomanip>

// Serial port settings
const std::string serial_port_name_ = "/dev/ttyUSB0";  // Change to your port
const int baudRate = B115200;
std::atomic<bool> running(true);

void set_terminal_mode(bool enable) {
    static struct termios oldt, newt;
    if (enable) {
        tcgetattr(STDIN_FILENO, &oldt);  // Save old terminal settings
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);  // Disable canonical mode and echo
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // Set new terminal settings
    } else {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // Restore old terminal settings
    }
}

bool process_serial_data(const std::deque<uint8_t>& buffer) {
    // std::cout << "RAW hex: "
    //           << std::hex << std::setw(2) << std::setfill('0') << (int)buffer[0] << " "
    //           << std::setw(2) << std::setfill('0') << (int)buffer[1] << " "
    //           << std::setw(2) << std::setfill('0') << (int)buffer[2] << " "
    //           << std::setw(2) << std::setfill('0') << (int)buffer[3] << " "
    //           << std::setw(2) << std::setfill('0') << (int)buffer[4] << " "
    //           << std::setw(2) << std::setfill('0') << (int)buffer[5] << " "
    //           << std::setw(2) << std::setfill('0') << (int)buffer[6] << " "
    //           << std::setw(2) << std::setfill('0') << (int)buffer[7] << std::dec << std::endl;
    uint32_t integer_value = (buffer[2] << 32) | (buffer[3] << 24) | (buffer[4] << 16) | (buffer[5] << 8) | buffer[6];
    uint8_t checksum = buffer[7];
// Display raw hex values in one line
    double one_tick_degree = 360.0 / 65536.0000;
    uint8_t calculated_checksum = buffer[0] + buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5] + buffer[6];
    // if (calculated_checksum == checksum) {
        // std::cout << "Integer value: " << integer_value << std::endl;
        int mod_result = integer_value % 65536;
        std::cout << "Degree value: " << (double)mod_result*one_tick_degree  << std::endl;
    // } else {
    //     int mod_result = integer_value % 65536;
    //     std::cout << "Degree value: " << (double)mod_result*one_tick_degree  << std::endl;
    //     printf("[0]0x%x [1]0x%x  [2]0x%x [3]0x%x [4]0x%x [5]0x%x [6]0x%x [7]0x%x \n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7]);
    //     printf("Calculated Checksum : 0x%x, 0x%x \n", calculated_checksum, calculated_checksum-buffer[7]);
    //     std::cerr << "Checksum error!" << std::endl;

    //     getchar();
    // }
    return true;
}

void serial_read_thread(int fd) {
    std::deque<uint8_t> buffer;
    uint8_t incoming_byte[256];

    while (running) {
        // std::this_thread::sleep_for(std::chrono::milliseconds(90));  // Adjust sleep duration as needed
        int n = read(fd, &incoming_byte, 8);
        if (n > 0) {
            // printf("readed %d size UART data \n", n);
            for(int i = 0; i < n; i++)
                buffer.push_back(incoming_byte[i]);
            if (buffer.size() < 8)
                continue;
            if (buffer[0] == 0xfd && buffer[1] == 0xfd) { //Checking header
                
                bool is_good = process_serial_data(buffer);
                // buffer.clear();
                tcflush(fd, TCIFLUSH);
                if(is_good) {
                    buffer.pop_front();
                    buffer.pop_front();
                    buffer.pop_front();
                    buffer.pop_front();
                    buffer.pop_front();
                    buffer.pop_front();
                    buffer.pop_front();
                    buffer.pop_front();
                    printf("pop 8 buffer. current buffer size: %d\n", buffer.size());
                }
                while(!buffer.empty() && buffer[0] != 0xfd) {
                    buffer.pop_front();
                    printf("pop buffer \n");
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } else {
                while(!buffer.empty() && buffer[0] != 0xfd) {
                    buffer.pop_front();
                    printf("pop buffer \n");
                }
            }
        }
    }
}

void input_thread() {
    set_terminal_mode(true);
    while (running) {
        int ch = getchar();
        if (ch == 'q') {
            running = false;
        }
    }
    set_terminal_mode(false);
}

int main() {
    termios options_;
    int fd_;
    if ((fd_ = open(serial_port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0) {
        throw std::runtime_error("Failed to open the device...");
    }

    memset(&options_, 0, sizeof(options_));
    cfsetospeed(&options_, baudRate);
    cfsetispeed(&options_, baudRate);
    options_.c_cflag = baudRate | CS8 | CREAD | CLOCAL;
    options_.c_iflag = IGNPAR | ICRNL;
    tcflush(fd_, TCIFLUSH);
    tcsetattr(fd_, TCSANOW, &options_);

    std::cout << "start" << std::endl;
    // std::thread inputThread(input_thread);
    std::thread serialThread(serial_read_thread, fd_);

    // inputThread.join();
    serialThread.join();

    close(fd_);
    return 0;
}
