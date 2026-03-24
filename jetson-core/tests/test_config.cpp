// File: tests/test_config.cpp
#include "adas/common/Config.hpp"

#include <iostream>

int main() {
  try {
    adas::Config cfg = adas::ConfigLoader::loadConfig("config/componentConfig.yaml");
    const bool ok = cfg.time.fusion_hz > 0 && cfg.cameras.width > 0 &&
                    cfg.cameras.height > 0 &&
                    cfg.stage_e_fusion.camera_hold_ms > 0 &&
                    cfg.stage_e_fusion.radar_hold_ms > 0 &&
                    cfg.stage_e_fusion.track_cleanup_ms > 0 &&
                    cfg.stage_e_fusion.provisional_min_hits >= 1 &&
                    cfg.stage_e_fusion.provisional_track_hold_ms > 0 &&
                    cfg.stage_e_fusion.promotion_min_hits >= 1 &&
                    cfg.stage_e_fusion.derived_speed_min_hits >= 2 &&
                    cfg.stage_e_fusion.derived_speed_max_dt_ms >=
                        cfg.stage_e_fusion.derived_speed_min_dt_ms &&
                    cfg.stage_e_fusion.gps_correction_gain >= 0.0f &&
                    cfg.stage_e_fusion.gps_correction_gain <= 1.0f &&
                    cfg.stage_e_fusion.ekf_r_radar_vz_derived >
                        cfg.stage_e_fusion.ekf_r_radar_vz;
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
