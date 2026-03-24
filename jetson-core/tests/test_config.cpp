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
                    cfg.stage_e_fusion.derived_speed_min_hits >= 1 &&
                    cfg.stage_e_fusion.derived_speed_min_dt_ms >= 1 &&
                    cfg.stage_e_fusion.derived_speed_max_dt_ms >=
                        cfg.stage_e_fusion.derived_speed_min_dt_ms &&
                    cfg.stage_e_fusion.derived_speed_hold_ms >= 1 &&
                    cfg.stage_e_fusion.derived_speed_max_plausible_mps > 0.0f &&
                    cfg.stage_e_fusion.derived_speed_jump_base_m >= 0.0f &&
                    cfg.stage_e_fusion.derived_speed_jump_slope_mps >= 0.0f &&
                    cfg.stage_e_fusion.radar_speed_disagreement_gate_mps >= 0.0f &&
                    cfg.stage_e_fusion.gps_correction_gain >= 0.0f &&
                    cfg.stage_e_fusion.gps_correction_gain <= 1.0f;
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
