// File: src/stage_a/RadarIngest.cpp
// Serial radar reader implementation based on radar_freq_test.cpp
#include "adas/stage_a/RadarIngest.hpp"

#include <cstring>
#include <iostream>

#ifdef __linux__
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace adas {

RadarIngest::RadarIngest(Mount mount, const std::string &port, SPSCQueue<RadarTargets, 8> &queue,
                         const RadarConfig &config)
    : mount_(mount), port_(port), queue_(queue), config_(config) {}

RadarIngest::~RadarIngest() { stop(); }

void RadarIngest::start() {
    if (running_.load(std::memory_order_relaxed)) {
        return;
    }

    running_.store(true, std::memory_order_relaxed);
    thread_ = std::thread(&RadarIngest::run, this);
}

void RadarIngest::stop() {
    running_.store(false, std::memory_order_relaxed);

#ifdef __linux__
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
#endif

    if (thread_.joinable()) {
        thread_.join();
    }
}

void RadarIngest::run() {
    std::cout << "[RadarIngest] Starting " << mountToString(mount_) << " on " << port_ << " at "
              << config_.baud_rate << " baud" << std::endl;

#ifdef __linux__
    if (!setupSerialPort()) {
        std::cerr << "[RadarIngest] Failed to setup serial port\n";
        healthy_.store(false, std::memory_order_relaxed);
        running_.store(false, std::memory_order_relaxed);
        return;
    }

    healthy_.store(true, std::memory_order_relaxed);
    last_rate_time_ = Clock::now_ns();

    std::vector<uint8_t> buffer;
    buffer.reserve(4096);

    while (running_.load(std::memory_order_relaxed)) {
        // Read available data
        if (!readFrame(buffer)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // Timestamp immediately after successful read
        uint64_t t_ingest = Clock::now_ns();

        // Parse and push to queue
        if (!buffer.empty()) {
            RadarTargets targets = parseFrame(buffer.data(), buffer.size(), t_ingest);
            queue_.try_push(std::move(targets));

            frames_received_.fetch_add(1, std::memory_order_relaxed);
            bytes_received_.fetch_add(buffer.size(), std::memory_order_relaxed);
            frames_in_window_++;
        }

        // Calculate rate every 5 seconds (matching radar_freq_test.cpp)
        if (Clock::elapsed_ms(last_rate_time_) >= 5000) {
            double elapsed_sec = Clock::ns_to_sec(Clock::now_ns() - last_rate_time_);
            double hz = static_cast<double>(frames_in_window_) / elapsed_sec;
            rate_hz_.store(hz, std::memory_order_relaxed);

            // Parse latest frame to get target count for debug
            RadarTargets latest = parseFrame(buffer.data(), buffer.size(), Clock::now_ns());
            
            std::cout << "[RadarIngest] " << mountToString(mount_) << " rate: " << hz << " Hz"
                      << " | Targets: " << latest.targets.size();
            if (!latest.targets.empty()) {
                std::cout << " | Closest: range=" << latest.targets[0].range_m 
                          << "m, vel=" << latest.targets[0].radial_vel_mps << "m/s";
            }
            std::cout << std::endl;

            // Only warn if below pipeline rate (20Hz)
            if (hz < 20.0) {
                std::cerr << "  [WARN] Below 20Hz pipeline rate - may cause data drops\n";
                if (hz < 10.0) {
                    healthy_.store(false, std::memory_order_relaxed);
                }
            }

            frames_in_window_ = 0;
            last_rate_time_ = Clock::now_ns();
        }
    }

    close(fd_);
    fd_ = -1;
#else
    std::cerr << "[RadarIngest] Serial port only supported on Linux\n";
    while (running_.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
#endif

    std::cout << "[RadarIngest] " << mountToString(mount_) << " stopped\n";
}

bool RadarIngest::setupSerialPort() {
#ifdef __linux__
    // From radar_freq_test.cpp: open serial port
    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        perror("[RadarIngest] open");
        return false;
    }

    // Configure termios
    struct termios tty {};
    if (tcgetattr(fd_, &tty) != 0) {
        perror("[RadarIngest] tcgetattr");
        close(fd_);
        fd_ = -1;
        return false;
    }

    // Set baud rate (921600)
    speed_t baud = B921600;
    switch (config_.baud_rate) {
    case 115200:
        baud = B115200;
        break;
    case 230400:
        baud = B230400;
        break;
    case 460800:
        baud = B460800;
        break;
    case 921600:
        baud = B921600;
        break;
    default:
        std::cerr << "[RadarIngest] Unsupported baud rate, using 921600\n";
        baud = B921600;
    }
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    // 8N1, no flow control (from radar_freq_test.cpp)
    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;            // 8 bits
    tty.c_cflag &= ~CRTSCTS;       // No flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable read, ignore carrier

    // Raw mode
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
    tty.c_oflag &= ~OPOST;

    // Non-blocking read
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        perror("[RadarIngest] tcsetattr");
        close(fd_);
        fd_ = -1;
        return false;
    }

    // Flush buffers
    tcflush(fd_, TCIOFLUSH);

    std::cout << "[RadarIngest] Serial port configured: " << port_ << " @ " << config_.baud_rate
              << " baud\n";
    return true;
#else
    return false;
#endif
}

bool RadarIngest::readFrame(std::vector<uint8_t> &buffer) {
#ifdef __linux__
    buffer.clear();

    // Use select with timeout
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fd_, &read_fds);

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = config_.poll_timeout_ms * 1000;

    int ret = select(fd_ + 1, &read_fds, nullptr, nullptr, &tv);
    if (ret <= 0) {
        return false; // Timeout or error
    }

    // Read available data
    uint8_t temp[1024];
    ssize_t n = read(fd_, temp, sizeof(temp));
    if (n <= 0) {
        return false;
    }

    buffer.assign(temp, temp + n);
    return true;
#else
    return false;
#endif
}

RadarTargets RadarIngest::parseFrame(const uint8_t *data, size_t len, uint64_t t_ingest) {
    RadarTargets targets;

    // Build header
    uint32_t s = seq_.fetch_add(1, std::memory_order_relaxed);
    targets.h = Header(t_ingest, mount_, s, true);

    // OPS243-A API JSON output parsing
    // The radar outputs JSON lines like:
    // {"product_id":"OPS243-A","targets":[{"range":1.5,"speed":-0.3},...]}
    //
    // For simplicity, we do pattern matching on the raw bytes
    // A full implementation would use a JSON parser
    //
    // Look for "range": and "speed": patterns

    std::string str(reinterpret_cast<const char *>(data), len);

    // Simple extraction - find range values
    size_t pos = 0;
    while ((pos = str.find("\"range\":", pos)) != std::string::npos) {
        pos += 8; // Skip "range":

        // Extract number
        size_t num_start = pos;
        while (pos < str.size() && (std::isdigit(str[pos]) || str[pos] == '.' || str[pos] == '-')) {
            ++pos;
        }

        if (pos > num_start) {
            try {
                float range = std::stof(str.substr(num_start, pos - num_start));

                // Look for speed in same object
                float speed = 0.0f;
                size_t speed_pos = str.find("\"speed\":", pos);
                if (speed_pos != std::string::npos && speed_pos < pos + 50) {
                    speed_pos += 8;
                    size_t speed_end = speed_pos;
                    while (speed_end < str.size() &&
                           (std::isdigit(str[speed_end]) || str[speed_end] == '.' ||
                            str[speed_end] == '-')) {
                        ++speed_end;
                    }
                    if (speed_end > speed_pos) {
                        speed = std::stof(str.substr(speed_pos, speed_end - speed_pos));
                    }
                }

                RadarTarget target;
                target.range_m = range;
                target.radial_vel_mps = speed;
                target.azimuth_rad = 0.0f; // OPS243-A doesn't provide azimuth
                target.rcs_db = 0.0f;
                target.sigma_r = 0.1f;
                target.sigma_v = 0.05f;
                target.sigma_az = 0.5f;

                targets.targets.push_back(target);
            } catch (...) {
                // Parse error, skip
            }
        }
    }

    if (targets.targets.empty()) {
        // No targets detected - still valid, just empty
        targets.h.healthy = true;
    }

    return targets;
}

RadarIngest::Stats RadarIngest::getStats() const {
    Stats s;
    s.frames_received = frames_received_.load(std::memory_order_relaxed);
    s.bytes_received = bytes_received_.load(std::memory_order_relaxed);
    s.errors = errors_.load(std::memory_order_relaxed);
    s.rate_hz = rate_hz_.load(std::memory_order_relaxed);
    return s;
}

} // namespace adas
