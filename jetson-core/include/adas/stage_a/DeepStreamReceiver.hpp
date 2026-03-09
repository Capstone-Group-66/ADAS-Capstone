// File: include/adas/stage_a/DeepStreamReceiver.hpp
#pragma once

#include "adas/common/Types.hpp"
#include "adas/queues/SPSCQueue.hpp"
#include <atomic>
#include <thread>
#include <vector>
#include <zmq.h>

namespace adas {

class DeepStreamReceiver {
public:
    DeepStreamReceiver();
    ~DeepStreamReceiver();

    // Non-copyable
    DeepStreamReceiver(const DeepStreamReceiver &) = delete;
    DeepStreamReceiver &operator=(const DeepStreamReceiver &) = delete;

    /// Start IPC receiver thread
    /// @param ds_queue Queue for DeepStream detections
    /// @return true if bound successfully
    bool start(SPSCQueue<DetBatch, 8> *ds_queue);

    /// Stop receiving and disconnect
    void stop();

    /// Check if running
    bool isRunning() const { return running_.load(std::memory_order_relaxed); }

private:
    void dsThread();

    void *context_ = nullptr;
    void *ds_socket_ = nullptr;

    SPSCQueue<DetBatch, 8> *ds_queue_ = nullptr;
    
    std::thread ds_thread_;
    std::atomic<bool> running_{false};
};

} // namespace adas
