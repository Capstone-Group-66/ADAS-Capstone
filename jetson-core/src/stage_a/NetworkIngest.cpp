// File: src/stage_a/NetworkIngest.cpp
// TCP receiver implementation for Raspberry Pi 4 rear sector
#include "adas/stage_a/NetworkIngest.hpp"

#include <opencv2/imgcodecs.hpp>

#include <cstring>
#include <iostream>

#ifdef __linux__
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#else
// Windows socket stubs for development
#define SOCKET int
#define INVALID_SOCKET (-1)
#define close(x) (void)(x)
#endif

namespace adas {

NetworkIngest::NetworkIngest(SPSCQueue<CameraFrameData, 8> &cam_queue,
                             SPSCQueue<RadarTargets, 8> &radar_l_queue,
                             SPSCQueue<RadarTargets, 8> &radar_r_queue,
                             const NetworkConfig &config)
    : cam_queue_(cam_queue), radar_l_queue_(radar_l_queue),
      radar_r_queue_(radar_r_queue), config_(config),
      latency_offset_ns_(Clock::ms_to_ns(config.latency_correction_ms)) {}

NetworkIngest::~NetworkIngest() { stop(); }

void NetworkIngest::start() {
    if (running_.load(std::memory_order_relaxed)) {
        return;
    }

    running_.store(true, std::memory_order_relaxed);
    thread_ = std::thread(&NetworkIngest::run, this);
}

void NetworkIngest::stop() {
    running_.store(false, std::memory_order_relaxed);

    closeClient();
    closeServer();

    if (thread_.joinable()) {
        thread_.join();
    }
}

void NetworkIngest::run() {
    std::cout << "[NetworkIngest] Starting on port " << config_.port << std::endl;
    std::cout << "[NetworkIngest] Latency correction: "
              << config_.latency_correction_ms << " ms" << std::endl;

#ifdef __linux__
    while (running_.load(std::memory_order_relaxed)) {
        // Create server socket if needed
        if (server_fd_ < 0) {
            if (!createServerSocket()) {
                std::cerr
                    << "[NetworkIngest] Failed to create server socket, retrying...\n";
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(config_.reconnect_timeout_ms));
                continue;
            }
        }

        // Accept connection if not connected
        if (client_fd_ < 0) {
            std::cout << "[NetworkIngest] Waiting for Pi4 connection...\n";
            if (!acceptConnection()) {
                continue; // Timeout or error, loop again
            }
            connected_.store(true, std::memory_order_relaxed);
            reconnects_.fetch_add(1, std::memory_order_relaxed);
            std::cout << "[NetworkIngest] Pi4 connected!\n";
        }

        // Read packet header
        NetPacketHeader header;
        if (!readExact(reinterpret_cast<uint8_t *>(&header), sizeof(header))) {
            std::cerr << "[NetworkIngest] Failed to read header, reconnecting...\n";
            closeClient();
            connected_.store(false, std::memory_order_relaxed);
            healthy_.store(false, std::memory_order_relaxed);
            continue;
        }

        // Record arrival time IMMEDIATELY
        uint64_t t_arrival = Clock::now_ns();

        // Validate magic word
        if (header.magic != NET_MAGIC_WORD) {
            std::cerr << "[NetworkIngest] Invalid magic word: 0x" << std::hex
                      << header.magic << std::dec << "\n";
            errors_.fetch_add(1, std::memory_order_relaxed);
            // Try to resync by reading 1 byte at a time until we find magic
            continue;
        }

        // Read payload
        std::vector<uint8_t> payload(header.payload_size);
        if (header.payload_size > 0) {
            if (!readExact(payload.data(), header.payload_size)) {
                std::cerr << "[NetworkIngest] Failed to read payload\n";
                closeClient();
                connected_.store(false, std::memory_order_relaxed);
                continue;
            }
        }

        // Update stats
        packets_received_.fetch_add(1, std::memory_order_relaxed);
        bytes_received_.fetch_add(sizeof(header) + header.payload_size,
                                  std::memory_order_relaxed);
        healthy_.store(true, std::memory_order_relaxed);

        // Process packet
        handlePacket(header, payload, t_arrival);
    }
#else
    std::cerr << "[NetworkIngest] Network ingest only supported on Linux\n";
    while (running_.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
#endif

    std::cout << "[NetworkIngest] Stopped\n";
}

bool NetworkIngest::createServerSocket() {
#ifdef __linux__
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
        perror("[NetworkIngest] socket");
        return false;
    }

    // Allow port reuse
    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // Bind
    struct sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(config_.port);

    if (bind(server_fd_, reinterpret_cast<struct sockaddr *>(&addr),
             sizeof(addr)) < 0) {
        perror("[NetworkIngest] bind");
        close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    // Listen
    if (listen(server_fd_, 1) < 0) {
        perror("[NetworkIngest] listen");
        close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    std::cout << "[NetworkIngest] Listening on port " << config_.port << std::endl;
    return true;
#else
    return false;
#endif
}

bool NetworkIngest::acceptConnection() {
#ifdef __linux__
    // Use poll for timeout
    struct pollfd pfd {};
    pfd.fd = server_fd_;
    pfd.events = POLLIN;

    int ret = poll(&pfd, 1, 1000); // 1 second timeout
    if (ret <= 0) {
        return false; // Timeout or error
    }

    struct sockaddr_in client_addr {};
    socklen_t addr_len = sizeof(client_addr);
    client_fd_ = accept(server_fd_,
                        reinterpret_cast<struct sockaddr *>(&client_addr),
                        &addr_len);

    if (client_fd_ < 0) {
        perror("[NetworkIngest] accept");
        return false;
    }

    // Set receive timeout
    struct timeval tv;
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    setsockopt(client_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    return true;
#else
    return false;
#endif
}

bool NetworkIngest::readExact(uint8_t *buffer, size_t length) {
#ifdef __linux__
    size_t total_read = 0;
    while (total_read < length && running_.load(std::memory_order_relaxed)) {
        ssize_t n = recv(client_fd_, buffer + total_read, length - total_read, 0);
        if (n <= 0) {
            return false;
        }
        total_read += n;
    }
    return total_read == length;
#else
    return false;
#endif
}

void NetworkIngest::handlePacket(const NetPacketHeader &header,
                                 const std::vector<uint8_t> &payload,
                                 uint64_t t_arrival) {
    // Apply timestamp correction: shift back by known latency
    uint64_t t_ingest = t_arrival - latency_offset_ns_;

    NetPacketType type = static_cast<NetPacketType>(header.type);

    switch (type) {
    case NetPacketType::RearCamera: {
        CameraFrameData frame =
            decodeCameraPacket(payload.data(), payload.size(), t_ingest);
        cam_queue_.try_push(std::move(frame));
        cam_frames_.fetch_add(1, std::memory_order_relaxed);
        break;
    }

    case NetPacketType::RearRadarL: {
        RadarTargets targets = parseRadarPacket(
            payload.data(), payload.size(), Mount::RearCornerRadarL, t_ingest);
        radar_l_queue_.try_push(std::move(targets));
        radar_l_frames_.fetch_add(1, std::memory_order_relaxed);
        break;
    }

    case NetPacketType::RearRadarR: {
        RadarTargets targets = parseRadarPacket(
            payload.data(), payload.size(), Mount::RearCornerRadarR, t_ingest);
        radar_r_queue_.try_push(std::move(targets));
        radar_r_frames_.fetch_add(1, std::memory_order_relaxed);
        break;
    }

    case NetPacketType::Heartbeat:
        // Just keep-alive, no action needed
        break;

    default:
        std::cerr << "[NetworkIngest] Unknown packet type: "
                  << static_cast<int>(header.type) << "\n";
        errors_.fetch_add(1, std::memory_order_relaxed);
        break;
    }
}

CameraFrameData NetworkIngest::decodeCameraPacket(const uint8_t *payload,
                                                  size_t size,
                                                  uint64_t t_ingest) {
    CameraFrameData frame_data;

    // Build header
    uint32_t seq = cam_seq_.fetch_add(1, std::memory_order_relaxed);
    frame_data.h = Header(t_ingest, Mount::RearCam, seq, true);

    // Decode MJPEG to BGR using OpenCV
    std::vector<uint8_t> mjpeg_data(payload, payload + size);
    cv::Mat decoded = cv::imdecode(mjpeg_data, cv::IMREAD_COLOR);

    if (decoded.empty()) {
        frame_data.h.healthy = false;
        frame_data.width = 0;
        frame_data.height = 0;
        return frame_data;
    }

    frame_data.width = decoded.cols;
    frame_data.height = decoded.rows;
    frame_data.channels = decoded.channels();

    // Copy pixel data
    size_t data_size = decoded.total() * decoded.elemSize();
    frame_data.data.resize(data_size);
    std::memcpy(frame_data.data.data(), decoded.data, data_size);

    return frame_data;
}

RadarTargets NetworkIngest::parseRadarPacket(const uint8_t *payload, size_t size,
                                             Mount mount, uint64_t t_ingest) {
    RadarTargets targets;

    // Build header
    uint32_t seq = (mount == Mount::RearCornerRadarL)
                       ? radar_l_seq_.fetch_add(1, std::memory_order_relaxed)
                       : radar_r_seq_.fetch_add(1, std::memory_order_relaxed);
    targets.h = Header(t_ingest, mount, seq, true);

    // DFRobot C4001 mmWave sensor format (simplified):
    // Payload contains presence detection and approximate range
    // Format: [presence (1 byte) | range_cm (2 bytes LE) | .....]
    //
    // For full implementation, refer to sensor datasheet
    // This is a simplified parser for presence + range

    if (size >= 3) {
        uint8_t presence = payload[0];
        uint16_t range_cm = payload[1] | (payload[2] << 8);

        if (presence > 0 && range_cm > 0) {
            RadarTarget target;
            target.range_m = static_cast<float>(range_cm) / 100.0f;
            target.azimuth_rad = 0.0f;     // C4001 doesn't provide azimuth
            target.radial_vel_mps = 0.0f;  // C4001 doesn't provide velocity
            target.rcs_db = 0.0f;
            target.sigma_r = 0.1f;  // ~10cm accuracy
            target.sigma_az = 0.5f; // Wide beam
            target.sigma_v = 1.0f;

            targets.targets.push_back(target);
        }
    }

    return targets;
}

void NetworkIngest::closeClient() {
#ifdef __linux__
    if (client_fd_ >= 0) {
        close(client_fd_);
        client_fd_ = -1;
    }
#endif
}

void NetworkIngest::closeServer() {
#ifdef __linux__
    if (server_fd_ >= 0) {
        close(server_fd_);
        server_fd_ = -1;
    }
#endif
}

NetworkIngest::Stats NetworkIngest::getStats() const {
    Stats s;
    s.packets_received = packets_received_.load(std::memory_order_relaxed);
    s.bytes_received = bytes_received_.load(std::memory_order_relaxed);
    s.cam_frames = cam_frames_.load(std::memory_order_relaxed);
    s.radar_l_frames = radar_l_frames_.load(std::memory_order_relaxed);
    s.radar_r_frames = radar_r_frames_.load(std::memory_order_relaxed);
    s.errors = errors_.load(std::memory_order_relaxed);
    s.reconnects = reconnects_.load(std::memory_order_relaxed);
    return s;
}

} // namespace adas
