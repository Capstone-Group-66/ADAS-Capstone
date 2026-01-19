#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include <csignal>
#include <iomanip>

// CONFIG: Confirmed ID 058b:0058 (Infineon) usually maps to /dev/ttyACM0
const char* PORT_NAME = "/dev/ttyACM0"; 
const int BAUD_RATE = B921600; 

bool keep_running = true;

// Handle Ctrl+C to exit gracefully
void signal_handler(int signum) {
    std::cout << "\n[STOP] Interrupt signal (" << signum << ") received.\n";
    keep_running = false;
}

int setup_serial_port(const char* port_name) {
    int serial_fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd == -1) {
        std::cerr << "[ERROR] Unable to open " << port_name << ": " << strerror(errno) << std::endl;
        return -1;
    }

    struct termios tty;
    if (tcgetattr(serial_fd, &tty) != 0) return -1;

    cfsetospeed(&tty, BAUD_RATE);
    cfsetispeed(&tty, BAUD_RATE);

    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // 1 Stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;     // 8 bits
    tty.c_cflag |= CREAD | CLOCAL;

    // Raw mode
    cfmakeraw(&tty);

    // Non-blocking read setup
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1; // 0.1s timeout

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) return -1;
    
    // Clear any junk in the buffer
    tcflush(serial_fd, TCIOFLUSH);
    
    return serial_fd;
}

int main() {
    signal(SIGINT, signal_handler);

    std::cout << "[INIT] Opening Infineon Radar at " << PORT_NAME << "..." << std::endl;
    int fd = setup_serial_port(PORT_NAME);
    if (fd < 0) return 1;

    std::cout << "[TEST] Starting Raw Stream & Frequency Check." << std::endl;
    std::cout << "[INFO] Press Ctrl+C to stop.\n" << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    auto last_report_time = start_time;
    long long total_bytes = 0;
    int packet_count = 0; 

    unsigned char buf[4096]; 

    while (keep_running) {
        int n = read(fd, buf, sizeof(buf));

        if (n > 0) {
            total_bytes += n;
            packet_count++;

            // --- OUTPUT RAW HEX PACKETS ---
            // Prints the raw bytes so you can see the data change when objects are detected.
            std::cout << "[RAW] (" << std::dec << n << " bytes): ";
            for (int i = 0; i < n; i++) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') 
                          << (int)buf[i] << " ";
            }
            std::cout << std::dec << std::endl;
        }

        // --- 5 SECOND REPORT ---
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - last_report_time;

        if (elapsed.count() >= 5.0) {
            double seconds = elapsed.count();
            double hz = packet_count / seconds;
            double kbps = (total_bytes / 1024.0) / seconds;

            std::cout << "\n================ [REPORT] ================" << std::endl;
            std::cout << "Data Rate:    " << kbps << " KB/s" << std::endl;
            std::cout << "Packet Rate:  " << hz << " Hz" << std::endl;

            // Feasibility Logic
            if (hz > 19.0) {
                std::cout << "[PASS] Perfect 20Hz operation." << std::endl;
            } else if (hz >= 17.0) {
                // Lowered threshold based on your observation
                std::cout << "[PASS] ~18Hz detected. Acceptable for Idle/No-Detection state." << std::endl;
            } else if (hz == 0.0) {
                 std::cout << "[FAIL] No data received." << std::endl;
            } else {
                std::cout << "[WARN] Rate is low (" << hz << " Hz). Check USB bandwidth or CPU load." << std::endl;
            }
            std::cout << "==========================================\n" << std::endl;

            packet_count = 0;
            total_bytes = 0;
            last_report_time = now;
        }

        if (n <= 0) usleep(1000); 
    }

    close(fd);
    std::cout << "\n[DONE] Port closed." << std::endl;
    return 0;
}
