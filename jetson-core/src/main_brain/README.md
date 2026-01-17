# Main Brain - BLE Vertical Slice

This is a **minimal vertical slice** implementation of the ADAS Main Brain for testing BLE connectivity between the Jetson board and the Android mobile app.

## Status

| Component | Status |
|-----------|--------|
| Alert Schema (FR70) | ✅ Complete |
| CBOR Encoding | ✅ Complete (`nlohmann/json`) |
| BLE Header Framing | ✅ Complete |
| Fragmentation | ✅ Complete |
| **BLE Service (BlueZ)** | ⚠️ **STUB** |

## BLE Service - STUB WARNING

The `SimpleBleServer` class is currently a **stub implementation**. It simulates BLE behavior by printing to the console but does **not** use real Bluetooth hardware.

**To make this work on actual Jetson hardware**, you must implement the `TODO`s in `SimpleBleServer.cpp` using the Linux BlueZ D-Bus API:

1. Connect to the system D-Bus
2. Register a GATT application via `org.bluez.GattManager1`
3. Create the ADAS Service and AlertStream Characteristic
4. Enable advertising via `org.bluez.LEAdvertisingManager1`
5. Handle connection callbacks and MTU negotiation

## Official UUIDs

```
Service UUID:     0000ada5-0000-1000-8000-00805f9b34fb
AlertStream UUID: 0000a1e7-0000-1000-8000-00805f9b34fb
Status UUID:      000057a7-0000-1000-8000-00805f9b34fb
Command UUID:     0000c0ad-0000-1000-8000-00805f9b34fb
Pair UUID:        0000fa17-0000-1000-8000-00805f9b34fb
```

## Build

```bash
cd jetson-core/build
cmake ..
make main_brain
./main_brain
```

## Files

- `include/adas/main_brain/BleUuids.hpp` - UUID constants
- `include/adas/main_brain/Alert.hpp` - Alert data structures
- `include/adas/main_brain/AlertGenerator.hpp` - Test alert generation
- `include/adas/main_brain/BleFragmenter.hpp` - Packet fragmentation
- `include/adas/main_brain/SimpleBleServer.hpp` - BLE server interface
- `src/main_brain/*.cpp` - Implementations
