// File: src/stage_a/BSDReceiver.cpp
#include "adas/stage_a/BSDReceiver.hpp"
#include <chrono>
#include <cstring>
#include <iostream>
#include <zmq.h>

namespace adas {

BSDReceiver::BSDReceiver(const std::string &pi_ip) : pi_ip_(pi_ip) {
  context_ = zmq_ctx_new();
  if (!context_) {
    std::cerr << "[BSDReceiver] Failed to create ZMQ context\n";
  }
}

BSDReceiver::~BSDReceiver() {
  stop();
  if (context_) {
    zmq_ctx_term(context_);
    context_ = nullptr;
  }
}

bool BSDReceiver::start() {
  if (running_.load())
    return true;
  if (!context_)
    return false;

  // Create Left Socket
  left_socket_ = zmq_socket(context_, ZMQ_PULL);
  if (!left_socket_)
    return false;
  int rcvtimeo = 100; // 100ms timeout
  zmq_setsockopt(left_socket_, ZMQ_RCVTIMEO, &rcvtimeo, sizeof(rcvtimeo));

  std::string left_uri = "tcp://" + pi_ip_ + ":5556";
  if (zmq_connect(left_socket_, left_uri.c_str()) != 0) {
    std::cerr << "[BSDReceiver] Failed to connect to left radar socket at "
              << left_uri << "\n";
    return false;
  }

  // Create Right Socket
  right_socket_ = zmq_socket(context_, ZMQ_PULL);
  if (!right_socket_)
    return false;
  zmq_setsockopt(right_socket_, ZMQ_RCVTIMEO, &rcvtimeo, sizeof(rcvtimeo));

  std::string right_uri = "tcp://" + pi_ip_ + ":5557";
  if (zmq_connect(right_socket_, right_uri.c_str()) != 0) {
    std::cerr << "[BSDReceiver] Failed to connect to right radar socket at "
              << right_uri << "\n";
    return false;
  }

  // Start Thread
  running_.store(true);
  left_last_rx_ms_ = Clock::now_ms();
  right_last_rx_ms_ = Clock::now_ms();

  thread_ = std::thread(&BSDReceiver::receiveLoop, this);
  std::cout << "[BSDReceiver] Started successfully. Left=" << left_uri
            << ", Right=" << right_uri << "\n";

  return true;
}

void BSDReceiver::stop() {
  running_.store(false);

  if (thread_.joinable()) {
    thread_.join();
  }

  if (left_socket_) {
    zmq_close(left_socket_);
    left_socket_ = nullptr;
  }

  if (right_socket_) {
    zmq_close(right_socket_);
    right_socket_ = nullptr;
  }
}

void BSDReceiver::processFailsafe(uint64_t now_ms) {
  // Failsafe Left
  if (left_state_.load() && (now_ms - left_last_rx_ms_ > TIMEOUT_MS)) {
    left_state_.store(false, std::memory_order_relaxed);
    std::cerr
        << "[BSDReceiver] Watchdog Left Timeout (>300ms). Resetting state.\n";
  }

  // Failsafe Right
  if (right_state_.load() && (now_ms - right_last_rx_ms_ > TIMEOUT_MS)) {
    right_state_.store(false, std::memory_order_relaxed);
    std::cerr
        << "[BSDReceiver] Watchdog Right Timeout (>300ms). Resetting state.\n";
  }
}

void BSDReceiver::receiveLoop() {
  zmq_pollitem_t items[] = {{left_socket_, 0, ZMQ_POLLIN, 0},
                            {right_socket_, 0, ZMQ_POLLIN, 0}};

  zmq_msg_t msg;
  zmq_msg_init(&msg);

  while (running_.load()) {
    uint64_t now_ms = Clock::now_ms();
    processFailsafe(now_ms);

    // 100ms poll timeout
    int rc = zmq_poll(items, 2, 100);
    if (rc < 0) {
      std::cerr << "[BSDReceiver] Poll error: " << zmq_strerror(zmq_errno())
                << "\n";
      continue;
    }

    if (rc == 0) {
      continue; // Poll timeout
    }

    // Check Left
    if (items[0].revents & ZMQ_POLLIN) {
      if (zmq_msg_recv(&msg, left_socket_, 0) > 0) {
        size_t msg_len = zmq_msg_size(&msg);
        uint8_t *msg_data = static_cast<uint8_t *>(zmq_msg_data(&msg));

        if (msg_len >= sizeof(BSDHeader)) {
          BSDHeader *header = reinterpret_cast<BSDHeader *>(msg_data);

          if (header->magic == 0x50493034 && header->version == 0x0100 &&
              header->msg_type == 0x0002) {
            if (header->payload_size == 1 && msg_len >= sizeof(BSDHeader) + 1) {
              uint8_t state = msg_data[sizeof(BSDHeader)];
              if (state == 0 || state == 1) {
                left_state_.store(state == 1, std::memory_order_relaxed);
                left_last_rx_ms_ = Clock::now_ms();

                std::cout << "[BSD] Left: " << (int)state << "\n";

                // Sequence gap tracking
                if (left_last_seq_ > 0 &&
                    header->sequence != left_last_seq_ + 1) {
                  left_drops_.fetch_add(1, std::memory_order_relaxed);
                }
                left_last_seq_ = header->sequence;
              } else {
                std::cout << "[BSD] Left invalid state byte: " << (int)state << "\n";
              }
            } else {
              std::cout << "[BSD] Left invalid length. msg_len=" << msg_len << "\n";
            }
          } else {
            std::cout << "[BSD] Left mismatch. Magic=" << std::hex << header->magic 
                      << " v=" << header->version << " type=" << header->msg_type << std::dec << "\n";
          }
        }
      }
    }

    // Check Right
    if (items[1].revents & ZMQ_POLLIN) {
      if (zmq_msg_recv(&msg, right_socket_, 0) > 0) {
        size_t msg_len = zmq_msg_size(&msg);
        uint8_t *msg_data = static_cast<uint8_t *>(zmq_msg_data(&msg));

        if (msg_len >= sizeof(BSDHeader)) {
          BSDHeader *header = reinterpret_cast<BSDHeader *>(msg_data);

          if (header->magic == 0x50493034 && header->version == 0x0100 &&
              header->msg_type == 0x0003) {
            if (header->payload_size == 1 && msg_len >= sizeof(BSDHeader) + 1) {
              uint8_t state = msg_data[sizeof(BSDHeader)];
              if (state == 0 || state == 1) {
                right_state_.store(state == 1, std::memory_order_relaxed);
                right_last_rx_ms_ = Clock::now_ms();

                std::cout << "[BSD] Right: " << (int)state << "\n";

                // Sequence gap tracking
                if (right_last_seq_ > 0 &&
                    header->sequence != right_last_seq_ + 1) {
                  right_drops_.fetch_add(1, std::memory_order_relaxed);
                }
                right_last_seq_ = header->sequence;
              } else {
                std::cout << "[BSD] Right invalid state byte: " << (int)state << "\n";
              }
            } else {
              std::cout << "[BSD] Right invalid length. msg_len=" << msg_len << "\n";
            }
          } else {
            std::cout << "[BSD] Right mismatch. Magic=" << std::hex << header->magic 
                      << " v=" << header->version << " type=" << header->msg_type << std::dec << "\n";
          }
        }
      }
    }
  }

  zmq_msg_close(&msg);
  std::cout << "[BSDReceiver] Thread stopped gracefully.\n";
}

} // namespace adas
