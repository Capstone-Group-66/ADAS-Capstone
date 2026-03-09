// File: include/adas/stage_e/BEVDashboard.hpp
// Dedicated Bird's-Eye View dashboard for BSD and Fused Targets
#pragma once

#include "adas/stage_e/SensorFusion.hpp"
#include "adas/stage_a/BSDReceiver.hpp"

#include <opencv2/core.hpp>

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace adas {

class BEVDashboard {
  public:
    /// Constructor
    /// @param bsd_receiver Pointer to the running BSD receiver for presence state
    /// @param c_x Principal point X (from camera intrinsics)
    /// @param f_x Horizontal focal length (from camera intrinsics)
    BEVDashboard(BSDReceiver* bsd_receiver, float c_x, float f_x);
    ~BEVDashboard();

    // Non-copyable
    BEVDashboard(const BEVDashboard &) = delete;
    BEVDashboard &operator=(const BEVDashboard &) = delete;

    /// Starts the dedicated dashboard rendering thread
    void start();

    /// Stops the dashboard and closes the window
    void stop();

    /// Safely updates the data payload used by the rendering thread
    void update(const std::vector<FusedObject>& fused_objects);

  private:
    void renderLoop();

    BSDReceiver* bsd_receiver_;
    float c_x_;
    float f_x_;

    std::vector<FusedObject> latest_fused_;
    std::mutex data_mutex_;

    std::thread thread_;
    std::atomic<bool> running_{false};
};

} // namespace adas
