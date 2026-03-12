// File: src/stage_a/RadarIngest.cpp
// Serial radar reader implementation based on radar_freq_test.cpp
#include "adas/stage_a/RadarIngest.hpp"
#include "adas/recording/Recorder.hpp"
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <initializer_list>
#include <iostream>

#ifdef __linux__
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace adas {

namespace {

std::string toLowerAscii(std::string value) {
  std::transform(
      value.begin(), value.end(), value.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

bool isCombinedNativeMode(std::string mode) {
  mode = toLowerAscii(mode);
  return mode == "combined_native";
}

bool tryReadFloatField(const nlohmann::json &packet, const char *key,
                       float &out_value) {
  auto it = packet.find(key);
  if (it == packet.end()) {
    return false;
  }
  if (it->is_number_float() || it->is_number_integer() ||
      it->is_number_unsigned()) {
    out_value = it->get<float>();
    return true;
  }
  if (it->is_string()) {
    try {
      out_value = std::stof(it->get<std::string>());
      return true;
    } catch (...) {
      return false;
    }
  }
  return false;
}

bool tryReadFloatAny(const nlohmann::json &packet,
                     std::initializer_list<const char *> keys,
                     float &out_value) {
  for (const char *key : keys) {
    if (tryReadFloatField(packet, key, out_value)) {
      return true;
    }
  }
  return false;
}

std::string readUnitField(const nlohmann::json &packet) {
  auto it = packet.find("unit");
  if (it == packet.end()) {
    return "";
  }
  if (it->is_string()) {
    return toLowerAscii(it->get<std::string>());
  }
  return toLowerAscii(it->dump());
}

bool unitIndicatesSpeed(const std::string &unit) {
  return unit == "mps" || unit == "m/s" || unit == "meters_per_second";
}

bool unitIndicatesRange(const std::string &unit) {
  return unit == "m" || unit == "meter" || unit == "meters";
}

#ifdef __linux__
bool writeAllCommand(int fd, const std::string &cmd) {
  size_t offset = 0;
  while (offset < cmd.size()) {
    const ssize_t n = write(fd, cmd.data() + offset, cmd.size() - offset);
    if (n > 0) {
      offset += static_cast<size_t>(n);
      continue;
    }
    if (n < 0 && (errno == EINTR)) {
      continue;
    }
    return false;
  }
  return true;
}
#endif

float normalizeTowardPositiveSpeed(float raw_speed_mps,
                                   const nlohmann::json &packet) {
  // Project convention: positive = toward/inward, negative = away/outward.
  if (!packet.contains("direction") && !packet.contains("dir")) {
    return raw_speed_mps;
  }

  std::string direction_text;
  const auto &direction_value =
      packet.contains("direction") ? packet["direction"] : packet["dir"];
  if (direction_value.is_string()) {
    direction_text = direction_value.get<std::string>();
  } else {
    direction_text = direction_value.dump();
  }
  direction_text = toLowerAscii(direction_text);

  const float magnitude = std::abs(raw_speed_mps);
  if (direction_text.find("toward") != std::string::npos ||
      direction_text.find("inward") != std::string::npos ||
      direction_text.find("approach") != std::string::npos ||
      direction_text == "in" || direction_text == "+") {
    return magnitude;
  }

  if (direction_text.find("away") != std::string::npos ||
      direction_text.find("outward") != std::string::npos ||
      direction_text.find("recede") != std::string::npos ||
      direction_text.find("depart") != std::string::npos ||
      direction_text == "out" || direction_text == "-") {
    return -magnitude;
  }

  return raw_speed_mps;
}

uint32_t computeDynamicSpeedTtlMs(int base_ttl_ms, float speed_mps) {
  const float abs_speed_mps = std::abs(speed_mps);

  // Slower motion keeps speed longer; faster motion expires sooner.
  float ttl_scale = 1.0f;
  if (abs_speed_mps < 0.5f) {
    ttl_scale = 2.2f;
  } else if (abs_speed_mps < 1.5f) {
    ttl_scale = 1.8f;
  } else if (abs_speed_mps < 3.0f) {
    ttl_scale = 1.4f;
  } else if (abs_speed_mps < 6.0f) {
    ttl_scale = 1.0f;
  } else if (abs_speed_mps < 10.0f) {
    ttl_scale = 0.8f;
  } else {
    ttl_scale = 0.65f;
  }

  const int min_ttl_ms = std::max(450, base_ttl_ms / 2);
  const int max_ttl_ms =
      std::max(base_ttl_ms, static_cast<int>(base_ttl_ms * 2.4f));
  const int scaled_ttl_ms =
      static_cast<int>(std::round(base_ttl_ms * ttl_scale));
  return static_cast<uint32_t>(
      std::clamp(scaled_ttl_ms, min_ttl_ms, max_ttl_ms));
}

} // namespace

RadarIngest::RadarIngest(Mount mount, const std::string &port,
                         SPSCQueue<RadarTargets, 8> &queue,
                         const RadarConfig &config)
    : mount_(mount), port_(port), queue_(queue), config_(config) {
  combined_native_mode_ = isCombinedNativeMode(config_.output_mode);
  desired_baud_rate_ = config_.baud_rate;
}

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
  closeSerialFd();
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
            << port_ << " at " << config_.baud_rate << " baud" << " (mode="
            << (combined_native_mode_ ? "combined_native" : "split_range")
            << ")" << std::endl;

#ifdef __linux__
  last_rate_time_ = Clock::now_ns();
  last_data_time_ns_ = last_rate_time_;

  std::vector<uint8_t> buffer;
  buffer.reserve(4096);

  while (running_.load(std::memory_order_relaxed)) {
    if (fd_ < 0) {
      const uint64_t now_ns = Clock::now_ns();
      const uint64_t elapsed_ms =
          (now_ns > last_connect_attempt_ns_)
              ? (now_ns - last_connect_attempt_ns_) / 1000000ULL
              : 0;
      if (elapsed_ms < reconnect_backoff_ms_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      last_connect_attempt_ns_ = now_ns;
      if (!setupSerialPort()) {
        healthy_.store(false, std::memory_order_relaxed);
        reconnect_backoff_ms_ =
            std::min<uint32_t>(reconnect_backoff_ms_ * 2, 2000);
        std::this_thread::sleep_for(
            std::chrono::milliseconds(reconnect_backoff_ms_));
        continue;
      }

      reconnect_backoff_ms_ = 200;
      healthy_.store(true, std::memory_order_relaxed);
      last_data_time_ns_ = Clock::now_ns();
      connected_since_ns_ = last_data_time_ns_;
      frames_at_connect_ = frames_received_.load(std::memory_order_relaxed);
      active_baud_rate_ = desired_baud_rate_;
      startup_baud_probe_done_ = false;
      line_buffer_.clear();

      if (!raw_csv_file_.is_open()) {
        std::string log_name =
            "ops243c_raw_" + std::to_string(Clock::now_ms()) + ".csv";
        raw_csv_file_.open(log_name);
        if (raw_csv_file_.is_open()) {
          raw_csv_file_
              << "t_ingest_ns,range_m,speed_mps,magnitude,time,direction\n";
        } else {
          std::cerr << "[RadarIngest] Failed to open raw CSV log: " << log_name
                    << "\n";
        }
      }
      continue;
    }

    const int read_status = readFrame(buffer);
    if (read_status == 1) {
      // Timestamp immediately after successful read
      uint64_t t_ingest = Clock::now_ns();
      last_data_time_ns_ = t_ingest;

      // Parse and push to queue
      if (!buffer.empty()) {
        RadarTargets targets =
            parseFrame(buffer.data(), buffer.size(), t_ingest);

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
    } else if (read_status == -2) {
      std::cerr << "[RadarIngest] Serial disconnect detected on " << port_
                << ", reconnecting...\n";
      errors_.fetch_add(1, std::memory_order_relaxed);
      healthy_.store(false, std::memory_order_relaxed);
      closeSerialFd();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    } else if (read_status == -1) {
      errors_.fetch_add(1, std::memory_order_relaxed);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    } else {
      const uint64_t now_ns = Clock::now_ns();
      const uint64_t connected_elapsed_ns =
          (connected_since_ns_ > 0 && now_ns > connected_since_ns_)
              ? (now_ns - connected_since_ns_)
              : 0;
      const uint64_t total_frames =
          frames_received_.load(std::memory_order_relaxed);
      const uint64_t frames_since_connect =
          (total_frames >= frames_at_connect_)
              ? (total_frames - frames_at_connect_)
              : 0;
      // One-time startup baud probe for "few pings then dead" behavior.
      if (!startup_baud_probe_done_ && frames_since_connect >= 1 &&
          frames_since_connect <= 4 && connected_elapsed_ns > 2000000000ULL) {
        startup_baud_probe_done_ = true;
        if (active_baud_rate_ == 921600) {
          desired_baud_rate_ = 115200;
        } else {
          desired_baud_rate_ = 921600;
        }
        std::cerr << "[RadarIngest] Startup stream stalled after "
                  << frames_since_connect << " frames, probing alternate baud "
                  << desired_baud_rate_ << "\n";
        healthy_.store(false, std::memory_order_relaxed);
        closeSerialFd();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }
      // No-data windows can be valid (target/threshold dependent); reconnect
      // only on explicit serial disconnect/error signals.
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
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

      // Mode-aware health thresholds: combined mode runs ~13-14Hz by design.
      const double warn_hz = combined_native_mode_ ? 10.0 : 20.0;
      const double unhealthy_hz = combined_native_mode_ ? 6.0 : 10.0;
      if (hz < warn_hz) {
        std::cerr << "  [WARN] Below expected radar rate for mode="
                  << (combined_native_mode_ ? "combined_native" : "split_range")
                  << " (hz=" << hz << ")\n";
        if (hz < unhealthy_hz) {
          healthy_.store(false, std::memory_order_relaxed);
        }
      } else {
        healthy_.store(true, std::memory_order_relaxed);
      }

      frames_in_window_ = 0;
      last_rate_time_ = Clock::now_ns();
    }
  }

  closeSerialFd();
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
  // Open blocking for reliable init writes; switch to non-blocking afterward.
  fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0) {
    perror("[RadarIngest] open");
    return false;
  }

  // Configure termios
  struct termios tty {};
  if (tcgetattr(fd_, &tty) != 0) {
    perror("[RadarIngest] tcgetattr");
    closeSerialFd();
    return false;
  }

  // Set baud rate
  speed_t baud = B921600;
  switch (desired_baud_rate_) {
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
    std::cerr << "[RadarIngest] Unsupported baud rate (" << desired_baud_rate_
              << "), using 921600\n";
    desired_baud_rate_ = 921600;
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
    closeSerialFd();
    return false;
  }

  // Flush buffers
  tcflush(fd_, TCIOFLUSH);

  // Send mode + common commands for OPS243.
  std::vector<std::string> init_cmds;
  if (combined_native_mode_) {
    init_cmds.push_back("OY\r\n"); // Combined speed+range output.
  } else {
    init_cmds.push_back("GX\r\n"); // Legacy split stream mode.
    init_cmds.push_back("OS\r\n");
    init_cmds.push_back("oD\r\n");
  }
  init_cmds.push_back("OJ\r\n"); // JSON output.
  init_cmds.push_back("UM\r\n"); // Speed units: m/s.
  init_cmds.push_back("uM\r\n"); // Range units: meters.
  init_cmds.push_back("SX\r\n");
  init_cmds.push_back("S[\r\n");
  init_cmds.push_back("s[\r\n");
  for (const std::string &cmd : init_cmds) {
    if (!writeAllCommand(fd_, cmd)) {
      std::cerr << "[RadarIngest] Failed to send radar command: " << cmd;
      closeSerialFd();
      return false;
    }
    usleep(50000); // 50ms wait
  }

  // Custom threshold commands based on configuration
  std::string mag_speed =
      "M>" + std::to_string(config_.speed_mag_threshold) + "\r\n";
  std::string mag_range =
      "m>" + std::to_string(config_.range_mag_threshold) + "\r\n";
  if (!writeAllCommand(fd_, mag_speed)) {
    std::cerr << "[RadarIngest] Failed to send speed magnitude threshold\n";
    closeSerialFd();
    return false;
  }
  usleep(50000);
  if (!writeAllCommand(fd_, mag_range)) {
    std::cerr << "[RadarIngest] Failed to send range magnitude threshold\n";
    closeSerialFd();
    return false;
  }
  usleep(50000);

  // Flush again to clear config echoing
  tcflush(fd_, TCIOFLUSH);

  // Runtime loop uses non-blocking select/read.
  const int flags = fcntl(fd_, F_GETFL, 0);
  if (flags < 0 || fcntl(fd_, F_SETFL, flags | O_NONBLOCK) != 0) {
    perror("[RadarIngest] fcntl(O_NONBLOCK)");
    closeSerialFd();
    return false;
  }

  std::cout << "[RadarIngest] Serial port configured: " << port_ << " @ "
            << desired_baud_rate_ << " baud" << " (mode="
            << (combined_native_mode_ ? "combined_native" : "split_range")
            << ")\n";
  return true;
#else
  return false;
#endif
}

int RadarIngest::readFrame(std::vector<uint8_t> &buffer) {
#ifdef __linux__
  buffer.clear();

  if (fd_ < 0) {
    return -2;
  }

  // Use select with timeout
  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(fd_, &read_fds);

  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = config_.poll_timeout_ms * 1000;

  int ret = select(fd_ + 1, &read_fds, nullptr, nullptr, &tv);
  if (ret == 0) {
    return 0; // Timeout
  }
  if (ret < 0) {
    if (errno == EINTR) {
      return 0;
    }
    if (errno == EBADF || errno == ENODEV || errno == EIO) {
      return -2;
    }
    return -1;
  }

  // Drain all currently available bytes in non-blocking mode.
  uint8_t temp[1024];
  bool saw_disconnect = false;
  while (true) {
    ssize_t n = read(fd_, temp, sizeof(temp));
    if (n > 0) {
      buffer.insert(buffer.end(), temp, temp + n);
      continue;
    }
    if (n == 0) {
      // Zero-byte read on non-blocking serial is not a hard disconnect.
      break;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      break;
    }
    if (errno == EINTR) {
      continue;
    }
    if (errno == EIO || errno == ENODEV || errno == EBADF) {
      saw_disconnect = true;
      break;
    }
    return -1;
  }

  if (!buffer.empty()) {
    return 1;
  }
  return saw_disconnect ? -2 : 0;
#else
  return -2;
#endif
}

void RadarIngest::closeSerialFd() {
#ifdef __linux__
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
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

      float parsed_range_m = 0.0f;
      float parsed_speed_mps = 0.0f;
      const bool has_range = tryReadFloatAny(
          j, {"range", "dist", "distance", "rng"}, parsed_range_m);
      const bool has_speed = tryReadFloatAny(
          j, {"speed", "vel", "velocity", "spd"}, parsed_speed_mps);
      const std::string unit = readUnitField(j);
      const bool unit_is_speed = unitIndicatesSpeed(unit);
      const bool unit_is_range = unitIndicatesRange(unit);

      if (raw_csv_file_.is_open()) {
        raw_csv_file_ << t_ingest << ",";
        if (has_range) {
          raw_csv_file_ << parsed_range_m;
        }
        raw_csv_file_ << ",";
        if (has_speed) {
          raw_csv_file_ << parsed_speed_mps;
        }
        raw_csv_file_ << ",";
        if (j.contains("magnitude")) {
          raw_csv_file_ << j["magnitude"];
        }
        raw_csv_file_ << ",";
        if (j.contains("time")) {
          raw_csv_file_ << j["time"];
        }
        raw_csv_file_ << ",";
        if (j.contains("direction")) {
          raw_csv_file_ << j["direction"];
        } else if (j.contains("dir")) {
          raw_csv_file_ << j["dir"];
        }
        raw_csv_file_ << "\n";
        raw_csv_file_.flush();
      }

      float normalized_speed_mps = last_speed_mps_;
      if (has_speed) {
        normalized_speed_mps =
            normalizeTowardPositiveSpeed(parsed_speed_mps, j);
        last_speed_mps_ = normalized_speed_mps;
        last_speed_ts_monotonic_ = t_ingest;
        speed_events_window_.fetch_add(1, std::memory_order_relaxed);
      }

      // Auto-detect support:
      // - split stream (unit mps / unit m)
      // - combined stream (speed + range in one packet)
      const bool should_emit_range_target =
          has_range &&
          (unit.empty() || unit_is_range || has_speed || !unit_is_speed);
      if (should_emit_range_target && parsed_range_m > 0.1f &&
          parsed_range_m <= 100.0f) {
        RadarTarget target;
        target.range_m = parsed_range_m;
        target.azimuth_rad = 0.0f;
        target.rcs_db = 0.0f;
        target.sigma_r = 0.1f;
        target.sigma_v = 0.05f;
        target.sigma_az = 0.5f;

        range_events_window_.fetch_add(1, std::memory_order_relaxed);

        if (has_speed) {
          target.radial_vel_mps = normalized_speed_mps;
          target.speed_fresh = true;
          target.speed_age_ms = 0;
          fused_fresh_window_.fetch_add(1, std::memory_order_relaxed);
        } else {
          // TTL evaluation for range-only packets.
          uint64_t age_ns = t_ingest > last_speed_ts_monotonic_
                                ? (t_ingest - last_speed_ts_monotonic_)
                                : 0;
          uint32_t age_ms = static_cast<uint32_t>(age_ns / 1000000);
          const uint32_t dynamic_speed_ttl_ms =
              computeDynamicSpeedTtlMs(config_.speed_ttl_ms, last_speed_mps_);

          if (age_ms <= dynamic_speed_ttl_ms) {
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
        }

        targets.targets.push_back(target);
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
