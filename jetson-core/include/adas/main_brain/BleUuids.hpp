// File: include/adas/main_brain/BleUuids.hpp
// ADAS BLE Service and Characteristic UUIDs
#pragma once

#include <string>

namespace adas {

/**
 * Official ADAS BLE UUIDs
 *
 * These use the Bluetooth Base UUID format (0000XXXX-0000-1000-8000-00805f9b34fb)
 * with custom short IDs:
 *   - ADA5 = ADAS Service
 *   - A1E7 = AlertStream Characteristic
 *   - 57A7 = Status Characteristic
 *   - C0AD = Command Characteristic
 *   - FA17 = Pair Characteristic
 */
namespace BleUuids {

// Primary ADAS Service UUID
constexpr const char* ADAS_SERVICE = "0000ada5-0000-1000-8000-00805f9b34fb";

// Characteristics (nested under ADAS Service)
constexpr const char* ADAS_ALERT_STREAM = "0000a1e7-0000-1000-8000-00805f9b34fb";  // Notify
constexpr const char* ADAS_STATUS       = "000057a7-0000-1000-8000-00805f9b34fb";  // Notify/Read
constexpr const char* ADAS_COMMAND      = "0000c0ad-0000-1000-8000-00805f9b34fb";  // Write
constexpr const char* ADAS_PAIR         = "0000fa17-0000-1000-8000-00805f9b34fb";  // Write/Notify

// Client Characteristic Configuration Descriptor (standard BLE)
constexpr const char* CCCD = "00002902-0000-1000-8000-00805f9b34fb";

}  // namespace BleUuids
}  // namespace adas
