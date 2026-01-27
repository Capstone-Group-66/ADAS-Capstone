// File: src/main_brain/SimpleBleServer.cpp
// BLE GATT Server implementation
// Uses a Python subprocess (ble_peripheral.py) for actual BLE operations
// C++ communicates with Python via stdin pipe

#include "adas/main_brain/SimpleBleServer.hpp"

#include <array>
#include <chrono>
#include <cstdio>
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
    std::mutex write_mutex;
#endif

    ~Impl() {
#ifdef __linux__
        if (python_stdin) {
            fclose(python_stdin);
            python_stdin = nullptr;
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
        // Create pipe for communication
        int pipefd[2];
        if (pipe(pipefd) == -1) {
            std::cerr << "[BLE] Failed to create pipe\n";
            return false;
        }

        python_pid = fork();
        if (python_pid == -1) {
            std::cerr << "[BLE] Failed to fork\n";
            return false;
        }

        if (python_pid == 0) {
            // Child process: run Python script
            close(pipefd[1]);              // Close write end
            dup2(pipefd[0], STDIN_FILENO); // Redirect stdin
            close(pipefd[0]);

            // Execute Python BLE peripheral
            execlp("python3", "python3", "scripts/ble_peripheral.py", nullptr);
            std::cerr << "[BLE] Failed to exec python3\n";
            _exit(1);
        }

        // Parent process
        close(pipefd[0]); // Close read end
        python_stdin = fdopen(pipefd[1], "w");

        if (!python_stdin) {
            std::cerr << "[BLE] Failed to open pipe for writing\n";
            kill(python_pid, SIGTERM);
            return false;
        }

        // Set line buffering
        setlinebuf(python_stdin);

        std::cout << "[BLE] Python BLE peripheral launched (PID: " << python_pid << ")\n";
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
};

SimpleBleServer::SimpleBleServer() : impl_(std::make_unique<Impl>()) {}

SimpleBleServer::~SimpleBleServer() { shutdown(); }

bool SimpleBleServer::initialize() {
    std::cout << "[BLE] Initializing SimpleBleServer...\n";
    std::cout << "[BLE] Service UUID: " << BleUuids::ADAS_SERVICE << "\n";
    std::cout << "[BLE] AlertStream Characteristic: " << BleUuids::ADAS_ALERT_STREAM << "\n";

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
