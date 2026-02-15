// File: include/adas/main_brain/SimpleBleServer.hpp
// Minimal BLE GATT Server using BlueZ D-Bus API
#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace adas {

/**
 * SimpleBleServer - Minimal BLE Peripheral for ADAS Vertical Slice
 *
 * This is a SIMPLIFIED implementation for the vertical slice demo.
 * It does NOT implement the full Hexagonal architecture from the proposal.
 *
 * What it DOES:
 *   - Advertises the ADAS Service UUID
 *   - Accepts phone connections
 *   - Sends notifications on the AlertStream characteristic
 *
 * What it does NOT do (deferred to full implementation):
 *   - Hexagonal ports/adapters pattern
 *   - Full ITransport interface
 *   - Pairing with 4-digit code (FR77)
 *   - Status characteristic (1Hz heartbeat)
 *   - Command characteristic
 *   - Reconnect ring buffer
 *   - Thermal/disk monitoring
 */
class SimpleBleServer {
  public:
    using OnConnectedCallback = std::function<void()>;
    using OnDisconnectedCallback = std::function<void()>;
    using OnGpsDataCallback = std::function<void(float speed_mps, uint64_t ts_ms)>;

    SimpleBleServer();
    ~SimpleBleServer();

    // Initialize BlueZ D-Bus connection and register GATT services
    bool initialize();

    // Start advertising the ADAS service
    bool startAdvertising();

    // Stop advertising
    void stopAdvertising();

    // Send a notification on the AlertStream characteristic
    // Returns true if sent successfully
    bool notifyAlertStream(const std::vector<uint8_t> &data);

    // Get current MTU (default 23, updated on connection)
    uint16_t getCurrentMtu() const { return mtu_; }

    // Check if a device is connected
    bool isConnected() const { return connected_.load(); }

    // Set callbacks
    void setOnConnected(OnConnectedCallback cb) { onConnected_ = std::move(cb); }
    void setOnDisconnected(OnDisconnectedCallback cb) { onDisconnected_ = std::move(cb); }
    void setOnGpsData(OnGpsDataCallback cb) { onGpsData_ = std::move(cb); }

    // Shutdown cleanly
    void shutdown();

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;

    std::atomic<bool> connected_{false};
    std::atomic<uint16_t> mtu_{23};

    OnConnectedCallback onConnected_;
    OnDisconnectedCallback onDisconnected_;
    OnGpsDataCallback onGpsData_;
};

} // namespace adas
