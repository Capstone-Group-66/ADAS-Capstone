// File: tests/test_bev_dashboard.cpp
#include "adas/stage_e/BEVDashboardMath.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace {

bool near(float a, float b, float eps = 1e-3f) {
  return std::fabs(a - b) <= eps;
}

} // namespace

int main() {
  int passed = 0;
  int failed = 0;

  // Geometry: center pixel should map to zero angle.
  {
    const float angle = adas::bev::angleFromPixel(606.709f, 606.709f, 828.752f);
    if (near(angle, 0.0f, 1e-6f)) {
      std::cout << "[PASS] center pixel angle\n";
      ++passed;
    } else {
      std::cout << "[FAIL] center pixel angle\n";
      ++failed;
    }
  }

  // Geometry: 12.5-degree span check for camera corridors.
  {
    const float in = adas::bev::degToRad(12.4f);
    const float out = adas::bev::degToRad(12.6f);
    const bool ok = adas::bev::inAngleSpan(in, 12.5f) &&
                    !adas::bev::inAngleSpan(out, 12.5f);
    if (ok) {
      std::cout << "[PASS] camera corridor span\n";
      ++passed;
    } else {
      std::cout << "[FAIL] camera corridor span\n";
      ++failed;
    }
  }

  // Geometry: lateral projection from range and angle.
  {
    const float x = adas::bev::lateralFromRangeAndAngle(10.0f,
                                                         adas::bev::degToRad(10.0f));
    if (near(x, 1.763f, 0.01f)) {
      std::cout << "[PASS] range-angle projection\n";
      ++passed;
    } else {
      std::cout << "[FAIL] range-angle projection\n";
      ++failed;
    }
  }

  // Lifecycle: ttl ratio should decay to zero after ttl.
  {
    const uint64_t now_ns = 2'000'000'000ULL;
    const uint64_t last_ns = 1'700'000'000ULL; // 300 ms ago
    const float r1 = adas::bev::ttlRemaining01(now_ns, last_ns, 500);
    const float r2 = adas::bev::ttlRemaining01(now_ns, last_ns, 200);
    const bool ok = (r1 > 0.39f && r1 < 0.41f) && near(r2, 0.0f, 1e-6f);
    if (ok) {
      std::cout << "[PASS] ttl ratio decay\n";
      ++passed;
    } else {
      std::cout << "[FAIL] ttl ratio decay\n";
      ++failed;
    }
  }

  // Lifecycle: keep track if camera OR range fresh OR hold active.
  {
    const uint64_t now_ns = 4'000'000'000ULL;
    const bool cam_only = adas::bev::shouldKeepTrack(
        now_ns, now_ns - 100'000'000ULL, 0, 0, 500);
    const bool range_only = adas::bev::shouldKeepTrack(
        now_ns, 0, now_ns - 200'000'000ULL, 0, 500);
    const bool hold_only = adas::bev::shouldKeepTrack(
        now_ns, 0, 0, now_ns + 1'000'000ULL, 500);
    const bool no_signal = adas::bev::shouldKeepTrack(
        now_ns, 0, 0, 0, 500);
    const bool dead = adas::bev::shouldKeepTrack(
        now_ns, now_ns - 600'000'000ULL, now_ns - 700'000'000ULL, 0, 500);

    const bool ok = cam_only && range_only && hold_only && !no_signal && !dead;
    if (ok) {
      std::cout << "[PASS] track keep/purge state machine\n";
      ++passed;
    } else {
      std::cout << "[FAIL] track keep/purge state machine\n";
      ++failed;
    }
  }

  // Flow mapping: rejected candidates branch at the correct gate stage.
  {
    const bool ok =
        adas::bev::dropReasonToFlowStage(adas::FCWDropReason::CamAge) ==
            adas::bev::FlowStage::CameraFresh &&
        adas::bev::dropReasonToFlowStage(adas::FCWDropReason::NoRadar) ==
            adas::bev::FlowStage::HasRadar &&
        adas::bev::dropReasonToFlowStage(adas::FCWDropReason::OutOfPath) ==
            adas::bev::FlowStage::InPath;
    if (ok) {
      std::cout << "[PASS] flow stage drop-reason mapping\n";
      ++passed;
    } else {
      std::cout << "[FAIL] flow stage drop-reason mapping\n";
      ++failed;
    }
  }

  // Flow mapping: runner-up branches at risk or state depending on why it lost.
  {
    adas::FCWDebugCandidate best;
    best.valid = true;
    best.object_id = 42;
    best.active_level = 2;
    best.desired_level = 2;
    best.risk_score = 0.70f;
    best.base_risk_score = 0.62f;

    adas::FCWDebugCandidate lower_risk = best;
    lower_risk.object_id = 17;
    lower_risk.risk_score = 0.51f;

    adas::FCWDebugCandidate lower_level = best;
    lower_level.object_id = 8;
    lower_level.active_level = 1;
    lower_level.desired_level = 1;

    const bool ok =
        adas::bev::runnerUpToFlowStage(best, lower_risk) ==
            adas::bev::FlowStage::RiskMix &&
        adas::bev::runnerUpToFlowStage(best, lower_level) ==
            adas::bev::FlowStage::StateMachine;
    if (ok) {
      std::cout << "[PASS] flow stage runner-up mapping\n";
      ++passed;
    } else {
      std::cout << "[FAIL] flow stage runner-up mapping\n";
      ++failed;
    }
  }

  std::cout << "\nBEV dashboard tests: " << passed << " passed, " << failed
            << " failed\n";
  return failed > 0 ? 1 : 0;
}
