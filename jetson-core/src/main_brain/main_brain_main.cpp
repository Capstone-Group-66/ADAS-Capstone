// File: src/main_brain/main_brain_main.cpp
// ADAS Main Brain Entry Point - Vertical Slice Demo
//
// This is a MINIMAL implementation for testing BLE connectivity.
// It sends a test alert every few seconds to verify the phone can receive it.

#include "adas/main_brain/Alert.hpp"
#include "adas/main_brain/AlertGenerator.hpp"
#include "adas/main_brain/BleFragmenter.hpp"
#include "adas/main_brain/BleUuids.hpp"
#include "adas/main_brain/SimpleBleServer.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

namespace {

std::atomic<bool> g_shutdown{false};

void signalHandler(int signum) {
  std::cout << "\n[MainBrain] Received signal " << signum
            << ", shutting down...\n";
  g_shutdown.store(true);
}

void printBanner() {
  std::cout << R"(
    ===========================================================================

         AAAAA  DDDD    AAAAA  SSSSS      M   M  AAAAA  III  N   N
        AA   AA DD  DD AA   AA SS        MM MM AA   AA  III  NN  N
        AAAAAAA DD   DD AAAAAAA SSSSS    M M M AAAAAAA  III  N N N
        AA   AA DD  DD AA   AA     SS    M   M AA   AA  III  N  NN
        AA   AA DDDD   AA   AA SSSSS     M   M AA   AA  III  N   N

                    Main Brain - Vertical Slice Demo

    ===========================================================================
    )" << std::endl;

  std::cout << "BLE UUIDs:\n";
  std::cout << "  Service:     " << adas::BleUuids::ADAS_SERVICE << "\n";
  std::cout << "  AlertStream: " << adas::BleUuids::ADAS_ALERT_STREAM << "\n";
  std::cout << "\n";
}

} // namespace

int main(int argc, char *argv[]) {
  printBanner();

  // Setup signal handlers
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  // Create BLE server
  adas::SimpleBleServer bleServer;

  bleServer.setOnConnected([]() {
    std::cout << "[MainBrain] Phone connected! Ready to send alerts.\n";
  });

  bleServer.setOnDisconnected(
      []() { std::cout << "[MainBrain] Phone disconnected.\n"; });

  // Initialize BLE
  if (!bleServer.initialize()) {
    std::cerr << "[MainBrain] Failed to initialize BLE server\n";
    return 1;
  }

  // Start advertising
  if (!bleServer.startAdvertising()) {
    std::cerr << "[MainBrain] Failed to start advertising\n";
    return 1;
  }

  std::cout << "[MainBrain] Waiting for phone connection...\n";
  std::cout << "[MainBrain] Press Ctrl+C to exit\n\n";

  // Main loop: send test alerts periodically
  uint32_t alertSeq = 0;
  uint16_t tickId = 0;
  auto lastAlertTime = std::chrono::steady_clock::now();
  constexpr auto ALERT_INTERVAL = std::chrono::seconds(3);

  while (!g_shutdown.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    auto now = std::chrono::steady_clock::now();
    if (now - lastAlertTime >= ALERT_INTERVAL && bleServer.isConnected()) {
      // Cycle through alert types for demo
      adas::AlertType types[] = {adas::AlertType::LDW, adas::AlertType::FCW,
                                 adas::AlertType::RCW, adas::AlertType::BSD};
      auto type = types[alertSeq % 4];

      // Generate and encode alert as TickPayload
      auto alert = adas::generateTestAlert(type, alertSeq);
      std::vector<adas::Alert> alerts = {alert};
      auto payload = adas::encodeTickPayloadToCbor(tickId, alerts);

      // Fragment the payload based on current MTU
      auto frames =
          adas::fragmentPayload(tickId, payload, bleServer.getCurrentMtu());

      std::cout << "[MainBrain] Sending " << adas::alertTypeToString(type)
                << " alert (tick=" << tickId << ", fragments=" << frames.size()
                << ", payload=" << payload.size() << " bytes)\n";

      // Send all fragments
      for (const auto &frame : frames) {
        bleServer.notifyAlertStream(frame);
      }

      alertSeq++;
      tickId++;
      lastAlertTime = now;
    }
  }

  // Cleanup
  bleServer.shutdown();
  std::cout << "[MainBrain] Shutdown complete.\n";

  return 0;
}
