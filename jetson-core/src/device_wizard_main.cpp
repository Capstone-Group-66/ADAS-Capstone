// File: src/device_wizard_main.cpp
// Standalone DeviceWizard executable
#include "adas/stage_a/DeviceWizard.hpp"

#include <iostream>

int main(int argc, char* argv[]) {
    std::string output_path = "config/hardware_map.json";
    bool show_preview = true;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--output" && i + 1 < argc) {
            output_path = argv[++i];
        } else if (arg == "--no-preview") {
            show_preview = false;
        } else if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "Options:\n"
                      << "  --output <path>   Output path for hardware_map.json\n"
                      << "  --no-preview      Skip camera preview windows\n"
                      << "  --help            Show this help\n";
            return 0;
        }
    }

    adas::DeviceWizard::runRegistration(output_path, show_preview);
    return 0;
}
