// File: scripts/jetson_receiver.cpp
// Jetson-side receiver for Pi4 sensor data
// Matches protocol defined in include/adas/common/PiProtocol.hpp

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <zmq.h>

// Include shared protocol definitions
// Note: When building standalone, copy PiProtocol.hpp or use relative path
#ifdef STANDALONE_BUILD
#include "PiProtocol.hpp"
#else
#include "adas/common/PiProtocol.hpp"
#endif

using namespace adas::protocol;

namespace {

struct Options {
    std::string pi_ip;  // Required: Pi IP address to connect to
    double stats_interval = 2.0;
    std::string save_dir;
    int save_every = 30;
    bool verbose = false;
    std::string log_file;
};

std::atomic<bool> g_stop{false};

void handle_signal(int) {
    g_stop.store(true);
}

void print_usage(const char* prog) {
    std::cout
        << "Usage: " << prog << " --pi-ip <ip> [options]\n"
        << "  --pi-ip <ip>             Pi IP address to connect to (REQUIRED)\n"
        << "  --stats-interval <sec>   Stats interval (default 2.0)\n"
        << "  --save-dir <path>        Save camera frames to directory\n"
        << "  --save-every <n>         Save every N frames (default 30)\n"
        << "  --verbose                Print each received message\n"
        << "  --log <path>             Log all data to file\n"
        << "  --help                   Show this help\n";
}

bool parse_args(int argc, char** argv, Options& out) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto next = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << name << "\n";
                std::exit(1);
            }
            return argv[++i];
        };

        if (arg == "--pi-ip") {
            out.pi_ip = next("--pi-ip");
        } else if (arg == "--stats-interval") {
            out.stats_interval = std::stod(next("--stats-interval"));
        } else if (arg == "--save-dir") {
            out.save_dir = next("--save-dir");
        } else if (arg == "--save-every") {
            out.save_every = std::stoi(next("--save-every"));
        } else if (arg == "--verbose") {
            out.verbose = true;
        } else if (arg == "--log") {
            out.log_file = next("--log");
        } else if (arg == "--help") {
            print_usage(argv[0]);
            return false;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            print_usage(argv[0]);
            return false;
        }
    }
    return true;
}

std::string build_addr(const std::string& ip, int port) {
    return "tcp://" + ip + ":" + std::to_string(port);
}

const char* msg_type_name(uint16_t type) {
    switch (static_cast<MessageType>(type)) {
        case MessageType::REAR_CAM_FRAME: return "REAR_CAM";
        case MessageType::REAR_RADAR_L:   return "RADAR_L";
        case MessageType::REAR_RADAR_R:   return "RADAR_R";
        case MessageType::IMU_SAMPLE:     return "IMU";
        case MessageType::HEARTBEAT:      return "HEARTBEAT";
        case MessageType::DISCOVERY_REQ:  return "DISCOVERY_REQ";
        case MessageType::DISCOVERY_RSP:  return "DISCOVERY_RSP";
        default: return "UNKNOWN";
    }
}

// Statistics per stream
struct StreamStats {
    uint64_t count = 0;
    uint64_t bytes = 0;
    uint32_t last_seq = 0;
    uint64_t drops = 0;
    uint64_t last_ts = 0;
};

void receiver_thread(void* ctx, const std::string& addr, MessageType expected_type,
                     StreamStats& stats, const Options& opt) {
    void* sock = zmq_socket(ctx, ZMQ_PULL);
    int hwm = 10;
    int timeout = 200;
    zmq_setsockopt(sock, ZMQ_RCVHWM, &hwm, sizeof(hwm));
    zmq_setsockopt(sock, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));

    int rc = zmq_connect(sock, addr.c_str());
    if (rc != 0) {
        std::cerr << "[" << msg_type_name(static_cast<uint16_t>(expected_type)) 
                  << "] Failed to connect to " << addr << ": " << zmq_strerror(errno) << "\n";
        zmq_close(sock);
        return;
    }

    std::cout << "[" << msg_type_name(static_cast<uint16_t>(expected_type)) 
              << "] Connected to " << addr << "\n";

    // Create save directory if needed
    if (!opt.save_dir.empty() && expected_type == MessageType::REAR_CAM_FRAME) {
        std::filesystem::create_directories(opt.save_dir);
    }

    std::vector<uint8_t> buffer(1024 * 1024);  // 1MB buffer

    while (!g_stop.load()) {
        int recv_len = zmq_recv(sock, buffer.data(), buffer.size(), 0);
        if (recv_len < 0) {
            continue;  // Timeout, try again
        }

        if (static_cast<size_t>(recv_len) < sizeof(PiMessageHeader)) {
            std::cerr << "[WARN] Received undersized message: " << recv_len << " bytes\n";
            continue;
        }

        // Parse header
        PiMessageHeader header;
        std::memcpy(&header, buffer.data(), sizeof(header));

        // Validate
        if (!validateHeader(header)) {
            std::cerr << "[WARN] Invalid header magic/version\n";
            continue;
        }

        if (static_cast<MessageType>(header.msg_type) != expected_type) {
            std::cerr << "[WARN] Unexpected message type: " << header.msg_type << "\n";
            continue;
        }

        // Update stats
        stats.count++;
        stats.bytes += recv_len;
        stats.last_ts = header.timestamp_ns;

        // Check for drops (sequence gap)
        if (stats.count > 1 && header.sequence != stats.last_seq + 1) {
            uint32_t gap = header.sequence - stats.last_seq - 1;
            stats.drops += gap;
        }
        stats.last_seq = header.sequence;

        // Verbose logging
        if (opt.verbose) {
            double ts_sec = header.timestamp_ns / 1e9;
            std::cout << "[" << msg_type_name(header.msg_type) << "] "
                      << "seq=" << header.sequence 
                      << " size=" << header.payload_size
                      << " ts=" << std::fixed << std::setprecision(3) << ts_sec << "\n";
        }

        // Save camera frames
        if (!opt.save_dir.empty() && 
            expected_type == MessageType::REAR_CAM_FRAME &&
            stats.count % opt.save_every == 0) {
            
            CameraPayloadHeader cam_header;
            std::memcpy(&cam_header, buffer.data() + sizeof(PiMessageHeader), sizeof(cam_header));
            
            size_t jpeg_offset = sizeof(PiMessageHeader) + sizeof(CameraPayloadHeader);
            size_t jpeg_size = header.payload_size - sizeof(CameraPayloadHeader);
            
            std::string path = opt.save_dir + "/frame_" + 
                               std::to_string(header.timestamp_ns / 1000000) + ".jpg";
            std::ofstream out(path, std::ios::binary);
            if (out) {
                out.write(reinterpret_cast<const char*>(buffer.data() + jpeg_offset), jpeg_size);
            }
        }
    }

    zmq_close(sock);
}

void heartbeat_thread(void* ctx, const std::string& addr, const Options& opt) {
    void* sock = zmq_socket(ctx, ZMQ_REQ);  // Use REQ to send heartbeat request
    int timeout = 1000;
    zmq_setsockopt(sock, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));
    zmq_setsockopt(sock, ZMQ_SNDTIMEO, &timeout, sizeof(timeout));

    int rc = zmq_connect(sock, addr.c_str());
    if (rc != 0) {
        std::cerr << "[CONTROL] Failed to connect to " << addr << "\n";
        zmq_close(sock);
        return;
    }

    std::cout << "[CONTROL] Connected to " << addr << "\n";

    std::vector<uint8_t> buffer(256);
    auto last_heartbeat = std::chrono::steady_clock::now();
    bool connected = false;

    while (!g_stop.load()) {
        // Build and send heartbeat request
        PiMessageHeader req_header;
        req_header.magic = PI_MAGIC;
        req_header.version = PI_PROTOCOL_VERSION;
        req_header.msg_type = static_cast<uint16_t>(MessageType::HEARTBEAT);
        req_header.payload_size = 0;
        req_header._padding = 0;
        req_header.timestamp_ns = 0;
        req_header.sequence = 0;
        req_header.reserved = 0;

        if (zmq_send(sock, &req_header, sizeof(req_header), 0) < 0) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        // Receive response
        int recv_len = zmq_recv(sock, buffer.data(), buffer.size(), 0);
        if (recv_len >= static_cast<int>(sizeof(PiMessageHeader) + sizeof(HeartbeatPayload))) {
            PiMessageHeader header;
            std::memcpy(&header, buffer.data(), sizeof(header));

            if (validateHeader(header) && 
                static_cast<MessageType>(header.msg_type) == MessageType::HEARTBEAT) {
                
                HeartbeatPayload hb;
                std::memcpy(&hb, buffer.data() + sizeof(PiMessageHeader), sizeof(hb));
                
                last_heartbeat = std::chrono::steady_clock::now();
                connected = true;

                if (opt.verbose) {
                    std::cout << "[HEARTBEAT] uptime=" << hb.uptime_ms << "ms"
                              << " cam=" << (int)hb.rear_cam_healthy
                              << " radarL=" << (int)hb.radar_l_healthy
                              << " radarR=" << (int)hb.radar_r_healthy
                              << " imu=" << (int)hb.imu_healthy
                              << " chrony_offset=" << hb.chrony_offset_us << "us\n";
                }
            }
        }

        // Check for heartbeat timeout
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_heartbeat).count();
        if (connected && elapsed > 5) {
            std::cerr << "[HEARTBEAT] WARNING: No heartbeat for " << elapsed << " seconds\n";
            connected = false;
        }

        // Wait 1 second between heartbeats
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    zmq_close(sock);
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    Options opt;
    if (!parse_args(argc, argv, opt)) {
        return 1;
    }

    // Validate required --pi-ip
    if (opt.pi_ip.empty()) {
        std::cerr << "ERROR: --pi-ip is required\n";
        print_usage(argv[0]);
        return 1;
    }

    std::cout << "\n";
    std::cout << "==============================================================\n";
    std::cout << "              JETSON PI4 RECEIVER                             \n";
    std::cout << "==============================================================\n";
    std::cout << "  Connecting to Pi at " << opt.pi_ip << "                      \n";
    std::cout << "  Protocol: PiProtocol v1.0                                   \n";
    std::cout << "  Press Ctrl+C to stop                                        \n";
    std::cout << "==============================================================\n\n";

    void* ctx = zmq_ctx_new();
    if (!ctx) {
        std::cerr << "Failed to create ZMQ context\n";
        return 1;
    }

    // Build addresses - connect to Pi
    std::string cam_addr = build_addr(opt.pi_ip, PORT_REAR_CAM);
    std::string radar_l_addr = build_addr(opt.pi_ip, PORT_RADAR_L);
    std::string radar_r_addr = build_addr(opt.pi_ip, PORT_RADAR_R);
    std::string imu_addr = build_addr(opt.pi_ip, PORT_IMU);
    std::string ctrl_addr = build_addr(opt.pi_ip, PORT_CONTROL);

    // Stats
    StreamStats cam_stats, radar_l_stats, radar_r_stats, imu_stats;

    // Start receiver threads
    std::thread cam_thread(receiver_thread, ctx, cam_addr, MessageType::REAR_CAM_FRAME,
                           std::ref(cam_stats), std::ref(opt));
    std::thread radar_l_thread(receiver_thread, ctx, radar_l_addr, MessageType::REAR_RADAR_L,
                               std::ref(radar_l_stats), std::ref(opt));
    std::thread radar_r_thread(receiver_thread, ctx, radar_r_addr, MessageType::REAR_RADAR_R,
                               std::ref(radar_r_stats), std::ref(opt));
    std::thread imu_thread(receiver_thread, ctx, imu_addr, MessageType::IMU_SAMPLE,
                           std::ref(imu_stats), std::ref(opt));
    std::thread hb_thread(heartbeat_thread, ctx, ctrl_addr, std::ref(opt));

    // Stats printing loop
    auto last_stats = std::chrono::steady_clock::now();
    uint64_t last_cam = 0, last_radar_l = 0, last_radar_r = 0, last_imu = 0;

    while (!g_stop.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - last_stats).count();
        
        if (elapsed >= opt.stats_interval) {
            double cam_fps = (cam_stats.count - last_cam) / elapsed;
            double radar_l_hz = (radar_l_stats.count - last_radar_l) / elapsed;
            double radar_r_hz = (radar_r_stats.count - last_radar_r) / elapsed;
            double imu_hz = (imu_stats.count - last_imu) / elapsed;

            std::cout << "----------------------------------------------------------------\n";
            std::cout << "  RearCam:  " << std::fixed << std::setprecision(1) << cam_fps 
                      << " fps | drops=" << cam_stats.drops << "\n";
            std::cout << "  RadarL:   " << radar_l_hz << " Hz | drops=" << radar_l_stats.drops << "\n";
            std::cout << "  RadarR:   " << radar_r_hz << " Hz | drops=" << radar_r_stats.drops << "\n";
            std::cout << "  IMU:      " << imu_hz << " Hz | drops=" << imu_stats.drops << "\n";
            std::cout << "----------------------------------------------------------------\n";

            last_cam = cam_stats.count;
            last_radar_l = radar_l_stats.count;
            last_radar_r = radar_r_stats.count;
            last_imu = imu_stats.count;
            last_stats = now;
        }
    }

    std::cout << "\n[Main] Shutting down...\n";

    cam_thread.join();
    radar_l_thread.join();
    radar_r_thread.join();
    imu_thread.join();
    hb_thread.join();

    zmq_ctx_term(ctx);

    std::cout << "[Main] Receiver stopped.\n";
    return 0;
}
