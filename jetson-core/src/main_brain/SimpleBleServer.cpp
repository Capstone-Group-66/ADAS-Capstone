// File: src/main_brain/SimpleBleServer.cpp
// BLE GATT Server implementation
// Uses a Python subprocess (ble_peripheral.py) for actual BLE operations
// C++ communicates with Python via stdin pipe

#include "adas/main_brain/SimpleBleServer.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <mutex>
#include <thread>

#include "adas/main_brain/BleUuids.hpp"

#ifdef __linux__
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace adas {

/**
 * Implementation class (PIMPL pattern)
 *
 * On Linux: Spawns ble_peripheral.py and writes alerts to its stdin
 * On Windows: Stub mode (prints to console)
 */
class SimpleBleServer::Impl {
public:
  bool initialized = false;
  bool advertising = false;

#ifdef __linux__
  pid_t python_pid = -1;
  FILE *python_stdin = nullptr;
  int python_stdout_fd = -1;
  std::mutex write_mutex;
  std::thread gps_reader_thread;
  std::atomic<bool> reader_running{false};
#endif

  ~Impl() {
#ifdef __linux__
    reader_running.store(false);
    if (python_stdin) {
      fclose(python_stdin);
      python_stdin = nullptr;
    }
    if (python_stdout_fd >= 0) {
      close(python_stdout_fd);
      python_stdout_fd = -1;
    }
    if (gps_reader_thread.joinable()) {
      gps_reader_thread.join();
    }
    if (python_pid > 0) {
      kill(python_pid, SIGTERM);
      waitpid(python_pid, nullptr, 0);
      python_pid = -1;
    }
#endif
  }

  bool launchPythonBle() {
#ifdef __linux__
    // Two pipes: stdin (C++ -> Python) and stdout (Python -> C++)
    int stdin_pipe[2];
    int stdout_pipe[2];
    if (pipe(stdin_pipe) == -1 || pipe(stdout_pipe) == -1) {
      std::cerr << "[BLE] Failed to create pipes\n";
      return false;
    }

    python_pid = fork();
    if (python_pid == -1) {
      std::cerr << "[BLE] Failed to fork\n";
      return false;
    }

    if (python_pid == 0) {
      // Child process: run Python script
      close(stdin_pipe[1]);                // Close write end of stdin pipe
      close(stdout_pipe[0]);               // Close read end of stdout pipe
      dup2(stdin_pipe[0], STDIN_FILENO);   // Redirect stdin
      dup2(stdout_pipe[1], STDOUT_FILENO); // Redirect stdout
      close(stdin_pipe[0]);
      close(stdout_pipe[1]);

      // Execute Python BLE peripheral
      execlp("python3", "python3", "scripts/ble_peripheral.py", nullptr);
      std::cerr << "[BLE] Failed to exec python3\n";
      _exit(1);
    }

    // Parent process
    close(stdin_pipe[0]);  // Close read end of stdin pipe
    close(stdout_pipe[1]); // Close write end of stdout pipe
    python_stdin = fdopen(stdin_pipe[1], "w");
    python_stdout_fd = stdout_pipe[0];

    if (!python_stdin) {
      std::cerr << "[BLE] Failed to open pipe for writing\n";
      kill(python_pid, SIGTERM);
      return false;
    }

    // Set line buffering
    setlinebuf(python_stdin);

    std::cout << "[BLE] Python BLE peripheral launched (PID: " << python_pid
              << ")\n";
    return true;
#else
    std::cout << "[BLE] Running in stub mode (Windows)\n";
    return true;
#endif
  }

  bool sendToPython(const std::string &json_alert) {
#ifdef __linux__
    if (!python_stdin)
      return false;

    std::lock_guard<std::mutex> lock(write_mutex);
    if (fprintf(python_stdin, "%s\n", json_alert.c_str()) < 0) {
      std::cerr << "[BLE] Failed to write to Python\n";
      return false;
    }
    fflush(python_stdin);
    return true;
#else
    // Windows stub: just print
    std::cout << "[BLE] Alert JSON: " << json_alert << "\n";
    return true;
#endif
  }

  /// Parse a GPS JSON line from Python stdout.
  /// Expected format: {"event":"gps","speed_mps":12.3,"ts_ms":1234}
  static bool parseGpsJson(const char *line, float &speed, uint64_t &ts) {
    // Lightweight parse — avoid adding a JSON library for two fields
    const char *sp = strstr(line, "\"speed_mps\"");
    const char *tp = strstr(line, "\"ts_ms\"");
    if (!sp || !tp)
      return false;
    sp = strchr(sp, ':');
    tp = strchr(tp, ':');
    if (!sp || !tp)
      return false;
    speed = static_cast<float>(atof(sp + 1));
    ts = static_cast<uint64_t>(strtoull(tp + 1, nullptr, 10));
    return true;
  }

  void startGpsReader(SimpleBleServer::OnGpsDataCallback cb) {
#ifdef __linux__
    if (python_stdout_fd < 0 || !cb)
      return;
    reader_running.store(true);
    gps_reader_thread = std::thread([this, cb = std::move(cb)]() {
      FILE *f = fdopen(python_stdout_fd, "r");
      if (!f) {
        std::cerr << "[BLE] Failed to open stdout pipe for reading\n";
        return;
      }
      char buf[512];
      while (reader_running.load()) {
        if (!fgets(buf, sizeof(buf), f))
          break;
        float speed = 0;
        uint64_t ts = 0;
        if (parseGpsJson(buf, speed, ts)) {
          std::cout << "[GPS] speed_mps=" << speed << " ts_ms=" << ts << "\n";
          cb(speed, ts);
        }
      }
      // Don't fclose — fd is owned by Impl and closed in destructor
    });
#endif
  }
};

SimpleBleServer::SimpleBleServer() : impl_(std::make_unique<Impl>()) {}

SimpleBleServer::~SimpleBleServer() { shutdown(); }

bool SimpleBleServer::initialize() {
  std::cout << "[BLE] Initializing SimpleBleServer...\n";
  std::cout << "[BLE] Service UUID: " << BleUuids::ADAS_SERVICE << "\n";
  std::cout << "[BLE] AlertStream Characteristic: "
            << BleUuids::ADAS_ALERT_STREAM << "\n";

  impl_->initialized = true;
  std::cout << "[BLE] Initialization complete\n";
  return true;
}

bool SimpleBleServer::startAdvertising() {
  if (!impl_->initialized) {
    std::cerr << "[BLE] Error: Not initialized\n";
    return false;
  }

  std::cout << "[BLE] Starting BLE peripheral...\n";

  if (!impl_->launchPythonBle()) {
    std::cerr << "[BLE] Failed to launch BLE peripheral\n";
    return false;
  }

  // Start GPS reader thread if callback is registered
  if (onGpsData_) {
    impl_->startGpsReader(onGpsData_);
  }

  impl_->advertising = true;
  connected_.store(true); // Assume connected for now
  mtu_.store(185);

  std::cout << "[BLE] Now discoverable as 'ADAS-Jetson'\n";

  if (onConnected_)
    onConnected_();

  return true;
}

void SimpleBleServer::stopAdvertising() {
  if (impl_->advertising) {
    std::cout << "[BLE] Stopping advertisement\n";
    impl_->advertising = false;
  }
}

bool SimpleBleServer::notifyAlertStream(const std::vector<uint8_t> &data) {
  if (!connected_.load()) {
    return false;
  }

  // Convert binary data to hex string for Python
  std::string hex;
  hex.reserve(data.size() * 2);
  for (uint8_t b : data) {
    char buf[3];
    snprintf(buf, sizeof(buf), "%02x", b);
    hex += buf;
  }

  // Send as JSON command to Python
  std::string json = "{\"cmd\":\"notify\",\"data\":\"" + hex + "\"}";

  std::cout << "[BLE] Notifying AlertStream: " << data.size() << " bytes\n";

  return impl_->sendToPython(json);
}

void SimpleBleServer::shutdown() {
  std::cout << "[BLE] Shutting down...\n";
  stopAdvertising();

  if (connected_.load()) {
    connected_.store(false);
    if (onDisconnected_)
      onDisconnected_();
  }

  impl_->initialized = false;
  std::cout << "[BLE] Shutdown complete\n";
}

} // namespace adas
