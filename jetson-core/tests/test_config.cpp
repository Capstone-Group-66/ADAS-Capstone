// File: tests/test_config.cpp
#include "adas/common/Config.hpp"

#include <iostream>

int main() {
  try {
    adas::Config cfg = adas::ConfigLoader::loadConfig("config/componentConfig.yaml");
    const bool ok = cfg.time.fusion_hz > 0 && cfg.cameras.width > 0 &&
                    cfg.cameras.height > 0;
    if (!ok) {
      std::cerr << "[FAIL] Config fields are invalid\n";
      return 1;
    }
    std::cout << "[PASS] Config loaded\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "[FAIL] Exception while loading config: " << e.what() << "\n";
    return 1;
  }
}

