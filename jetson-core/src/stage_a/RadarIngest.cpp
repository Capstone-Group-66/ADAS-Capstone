// File: src/stage_a/RadarIngest.cpp
// Serial radar reader implementation based on radar_freq_test.cpp
#include "adas/stage_a/RadarIngest.hpp"
#include "adas/recording/Recorder.hpp"
#include <nlohmann/json.hpp>

#include <cstring>
#include <iostream>

#ifdef __linux__
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace adas {

RadarIngest::RadarIngest(Mount mount, const std::string &port,
                         SPSCQueue<RadarTargets, 8> &queue,
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

  if (raw_csv_file_.is_open()) {
    raw_csv_file_.close();
  }
}

void RadarIngest::run() {
  std::cout << "[RadarIngest] Starting " << mountToString(mount_) << " on "
            << port_ << " at " << config_.baud_rate << " baud" << std::endl;

#ifdef __linux__
  if (!setupSerialPort()) {
    std::cerr << "[RadarIngest] Failed to setup serial port\n";
    healthy_.store(false, std::memory_order_relaxed);
    running_.store(false, std::memory_order_relaxed);
    return;
  }

  healthy_.store(true, std::memory_order_relaxed);
  last_rate_time_ = Clock::now_ns();

  // Open CSV file for raw data dump
  std::string log_name =
      "ops243c_raw_" + std::to_string(Clock::now_ms()) + ".csv";
  raw_csv_file_.open(log_name);
  if (raw_csv_file_.is_open()) {
    raw_csv_file_ << "t_ingest_ns,range_m,speed_mps,magnitude,time,direction\n";
  } else {
    std::cerr << "[RadarIngest] Failed to open raw CSV log: " << log_name
              << "\n";
  }

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

      if (!targets.targets.empty()) {
        // Record before pushing to queue (if recording active)
        if (recorder_) {
          recorder_->recordRadar(targets);
        }
        queue_.try_push(std::move(targets));
      }

      frames_received_.fetch_add(1, std::memory_order_relaxed);
      bytes_received_.fetch_add(buffer.size(), std::memory_order_relaxed);
      frames_in_window_++;
    }

    // Calculate rate every 5 seconds (matching radar_freq_test.cpp)
    if (Clock::elapsed_ms(last_rate_time_) >= 5000) {
      double elapsed_sec = Clock::ns_to_sec(Clock::now_ns() - last_rate_time_);
      double hz = static_cast<double>(frames_in_window_) / elapsed_sec;
      rate_hz_.store(hz, std::memory_order_relaxed);

      uint64_t re = range_events_window_.exchange(0, std::memory_order_relaxed);
      uint64_t se = speed_events_window_.exchange(0, std::memory_order_relaxed);
      uint64_t ff = fused_fresh_window_.exchange(0, std::memory_order_relaxed);
      uint64_t fs = fused_stale_window_.exchange(0, std::memory_order_relaxed);
      uint64_t errs = parse_error_count_.load(std::memory_order_relaxed);

      range_hz_.store(re / elapsed_sec, std::memory_order_relaxed);
      speed_event_hz_.store(se / elapsed_sec, std::memory_order_relaxed);

      uint64_t total_fused = ff + fs;
      if (total_fused > 0) {
        fused_with_fresh_speed_ratio_.store(
            static_cast<double>(ff) / total_fused, std::memory_order_relaxed);
        stale_speed_ratio_.store(static_cast<double>(fs) / total_fused,
                                 std::memory_order_relaxed);
      } else {
        fused_with_fresh_speed_ratio_.store(0.0, std::memory_order_relaxed);
        stale_speed_ratio_.store(0.0, std::memory_order_relaxed);
      }

      std::cout << "[RadarIngest] " << mountToString(mount_) << " rate: " << hz
                << " Hz | RNG: " << (re / elapsed_sec)
                << " Hz | SPD: " << (se / elapsed_sec) << " Hz"
                << " | Fresh Ratio: "
                << (total_fused > 0 ? (ff * 100.0 / total_fused) : 0) << "%"
                << " | Errs: " << errs;
      std::cout << std::endl;

      // Only warn if below pipeline rate (20Hz)
      if (hz < 20.0) {
        std::cerr
            << "  [WARN] Below 20Hz pipeline rate - may cause data drops\n";
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

  // Send config commands for OPS243-C
  const char *init_cmds[] = {"GX\r\n", "OS\r\n", "oD\r\n", "OJ\r\n", "UM\r\n",
                             "uM\r\n", "SX\r\n", "S[\r\n", "s[\r\n"};
  for (const char *cmd : init_cmds) {
    write(fd_, cmd, strlen(cmd));
    usleep(50000); // 50ms wait
  }

  // Custom threshold commands based on configuration
  std::string mag_speed =
      "M>" + std::to_string(config_.speed_mag_threshold) + "\r\n";
  std::string mag_range =
      "m>" + std::to_string(config_.range_mag_threshold) + "\r\n";
  write(fd_, mag_speed.c_str(), mag_speed.length());
  usleep(50000);
  write(fd_, mag_range.c_str(), mag_range.length());
  usleep(50000);

  // Flush again to clear config echoing
  tcflush(fd_, TCIOFLUSH);

  std::cout << "[RadarIngest] Serial port configured: " << port_ << " @ "
            << config_.baud_rate << " baud\n";
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

RadarTargets RadarIngest::parseFrame(const uint8_t *data, size_t len,
                                     uint64_t t_ingest) {
  RadarTargets targets;

  // Build header
  uint32_t s = seq_.fetch_add(1, std::memory_order_relaxed);
  targets.h = Header(t_ingest, mount_, s, true);

  line_buffer_.append(reinterpret_cast<const char *>(data), len);

  size_t pos;
  while ((pos = line_buffer_.find("\r\n")) != std::string::npos) {
    std::string line = line_buffer_.substr(0, pos);
    line_buffer_.erase(0, pos + 2);

    if (line.empty())
      continue;

    try {
      auto j = nlohmann::json::parse(line);

      if (raw_csv_file_.is_open()) {
        raw_csv_file_ << t_ingest << ",";
        if (j.contains("range"))
          raw_csv_file_ << j["range"];
        raw_csv_file_ << ",";
        if (j.contains("speed"))
          raw_csv_file_ << j["speed"];
        raw_csv_file_ << ",";
        if (j.contains("magnitude"))
          raw_csv_file_ << j["magnitude"];
        raw_csv_file_ << ",";
        if (j.contains("time"))
          raw_csv_file_ << j["time"];
        raw_csv_file_ << ",";
        if (j.contains("direction"))
          raw_csv_file_ << j["direction"];
        raw_csv_file_ << "\n";
        raw_csv_file_.flush();
      }

      if (j.contains("unit")) {
        std::string unit = j["unit"];
        if (unit == "mps" && j.contains("speed")) {
          float speed = j["speed"];
          last_speed_mps_ = speed;
          last_speed_ts_monotonic_ = t_ingest;
          speed_events_window_.fetch_add(1, std::memory_order_relaxed);
        } else if (unit == "m" && j.contains("range")) {
          float range = j["range"];
          if (range > 0.1f && range <= 100.0f) { // Ignore anomalies
            RadarTarget target;
            target.range_m = range;
            target.azimuth_rad = 0.0f;
            target.rcs_db = 0.0f;
            target.sigma_r = 0.1f;
            target.sigma_v = 0.05f;
            target.sigma_az = 0.5f;

            range_events_window_.fetch_add(1, std::memory_order_relaxed);

            // TTL Evaluation
            uint64_t age_ns = t_ingest > last_speed_ts_monotonic_
                                  ? (t_ingest - last_speed_ts_monotonic_)
                                  : 0;
            uint32_t age_ms = static_cast<uint32_t>(age_ns / 1000000);

            if (age_ms <= static_cast<uint32_t>(config_.speed_ttl_ms)) {
              target.radial_vel_mps = last_speed_mps_;
              target.speed_fresh = true;
              target.speed_age_ms = age_ms;
              fused_fresh_window_.fetch_add(1, std::memory_order_relaxed);
            } else {
              target.radial_vel_mps = 0.0f;
              target.speed_fresh = false;
              target.speed_age_ms = age_ms;
              fused_stale_window_.fetch_add(1, std::memory_order_relaxed);
            }

            targets.targets.push_back(target);
          }
        }
      }
    } catch (...) {
      parse_error_count_.fetch_add(1, std::memory_order_relaxed);
    }
  }

  return targets;
}

RadarIngest::Stats RadarIngest::getStats() const {
  Stats s;
  s.frames_received = frames_received_.load(std::memory_order_relaxed);
  s.bytes_received = bytes_received_.load(std::memory_order_relaxed);
  s.errors = errors_.load(std::memory_order_relaxed);
  s.rate_hz = rate_hz_.load(std::memory_order_relaxed);
  s.range_hz = range_hz_.load(std::memory_order_relaxed);
  s.speed_event_hz = speed_event_hz_.load(std::memory_order_relaxed);
  s.fused_with_fresh_speed_ratio =
      fused_with_fresh_speed_ratio_.load(std::memory_order_relaxed);
  s.stale_speed_ratio = stale_speed_ratio_.load(std::memory_order_relaxed);
  s.parse_error_count = parse_error_count_.load(std::memory_order_relaxed);
  return s;
}

} // namespace adas
