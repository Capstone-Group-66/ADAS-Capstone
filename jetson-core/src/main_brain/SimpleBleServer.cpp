// File: src/main_brain/SimpleBleServer.cpp
// Minimal BLE GATT Server implementation using BlueZ D-Bus API
//
// NOTE: This is a SIMPLIFIED stub for the vertical slice.
// Full BlueZ D-Bus implementation requires libdbus or sdbus-c++.
// For the demo, we use a mock/stub that prints to console.
// Replace with actual BlueZ calls when running on Jetson.

#include "adas/main_brain/SimpleBleServer.hpp"
#include "adas/main_brain/BleUuids.hpp"

#include <iostream>
#include <thread>
#include <chrono>

namespace adas {

/**
 * Implementation class (PIMPL pattern)
 *
 * In the full implementation, this would use:
 *   - libdbus or sdbus-c++ for D-Bus communication
 *   - BlueZ GATT API to register services/characteristics
 *   - HCI socket for low-level control if needed
 *
 * For this vertical slice stub, we simulate the BLE behavior.
 */
class SimpleBleServer::Impl {
public:
    bool initialized = false;
    bool advertising = false;

    // Simulated connection state (in real impl, comes from BlueZ callbacks)
    bool deviceConnected = false;
};

SimpleBleServer::SimpleBleServer() : impl_(std::make_unique<Impl>()) {}

SimpleBleServer::~SimpleBleServer() {
    shutdown();
}

bool SimpleBleServer::initialize() {
    std::cout << "[BLE] Initializing SimpleBleServer...\n";
    std::cout << "[BLE] Service UUID: " << BleUuids::ADAS_SERVICE << "\n";
    std::cout << "[BLE] AlertStream Characteristic: " << BleUuids::ADAS_ALERT_STREAM << "\n";

    // TODO: In real implementation:
    // 1. Connect to system D-Bus
    // 2. Get BlueZ adapter object (org.bluez.Adapter1)
    // 3. Register GATT application (org.bluez.GattManager1.RegisterApplication)
    // 4. Create service with characteristics

    impl_->initialized = true;
    std::cout << "[BLE] Initialization complete (stub mode)\n";
    return true;
}

bool SimpleBleServer::startAdvertising() {
    if (!impl_->initialized) {
        std::cerr << "[BLE] Error: Not initialized\n";
        return false;
    }

    std::cout << "[BLE] Starting advertisement...\n";
    std::cout << "[BLE] Advertising ADAS Service: " << BleUuids::ADAS_SERVICE << "\n";

    // TODO: In real implementation:
    // 1. Get LEAdvertisingManager1 interface
    // 2. Register advertisement with service UUIDs
    // 3. Set local name to "ADAS-Jetson"

    impl_->advertising = true;
    std::cout << "[BLE] Now discoverable as 'ADAS-Jetson'\n";

    // Simulate a connection after 2 seconds (for testing without real hardware)
    // Remove this in production!
    std::thread([this]() {
        std::this_thread::sleep_for(std::chrono::seconds(2));
        if (impl_->advertising && !connected_.load()) {
            std::cout << "[BLE] Simulated device connection\n";
            connected_.store(true);
            mtu_.store(185);  // Typical negotiated MTU
            if (onConnected_) onConnected_();
        }
    }).detach();

    return true;
}

void SimpleBleServer::stopAdvertising() {
    if (impl_->advertising) {
        std::cout << "[BLE] Stopping advertisement\n";
        impl_->advertising = false;
    }
}

bool SimpleBleServer::notifyAlertStream(const std::vector<uint8_t>& data) {
    if (!connected_.load()) {
        // Not connected, silently drop
        return false;
    }

    // TODO: In real implementation:
    // 1. Get the AlertStream characteristic object
    // 2. Call org.bluez.GattCharacteristic1.Notify or update Value property
    // 3. Respect MTU fragmentation

    std::cout << "[BLE] Notifying AlertStream: " << data.size() << " bytes\n";

    // Debug: print first few bytes
    std::cout << "[BLE]   Data: ";
    for (size_t i = 0; i < std::min(data.size(), size_t(16)); ++i) {
        printf("%02x ", data[i]);
    }
    if (data.size() > 16) std::cout << "...";
    std::cout << "\n";

    return true;
}

void SimpleBleServer::shutdown() {
    std::cout << "[BLE] Shutting down...\n";
    stopAdvertising();

    if (connected_.load()) {
        connected_.store(false);
        if (onDisconnected_) onDisconnected_();
    }

    impl_->initialized = false;
    std::cout << "[BLE] Shutdown complete\n";
}

}  // namespace adas
