// File: include/adas/stage_e/BEVDashboardFlow.hpp
// Pure helper mapping for FCW thought-flow visualization.
#pragma once

#include "adas/stage_e/FCWMonitor.hpp"

#include <cstdint>
#include <string>

namespace adas::bev {

enum class FlowStage : uint8_t {
  Fusion = 0,
  CameraFresh = 1,
  HasRadar = 2,
  SpeedFresh = 3,
  ClassRelevant = 4,
  RangeOk = 5,
  QualityOk = 6,
  ClosingOk = 7,
  InPath = 8,
  RiskMix = 9,
  StateMachine = 10,
  Output = 11,
};

struct FlowLaneModel {
  uint64_t object_id = UINT64_MAX;
  FlowStage terminal_stage = FlowStage::Fusion;
  bool is_rejection = true;
  std::string terminal_label;
};

inline const char *flowStageName(FlowStage stage) {
  switch (stage) {
  case FlowStage::Fusion:
    return "Fusion";
  case FlowStage::CameraFresh:
    return "Cam fresh";
  case FlowStage::HasRadar:
    return "Has radar";
  case FlowStage::SpeedFresh:
    return "Speed fresh";
  case FlowStage::ClassRelevant:
    return "Class";
  case FlowStage::RangeOk:
    return "Range";
  case FlowStage::QualityOk:
    return "Quality";
  case FlowStage::ClosingOk:
    return "Closing";
  case FlowStage::InPath:
    return "In path";
  case FlowStage::RiskMix:
    return "Risk";
  case FlowStage::StateMachine:
    return "State";
  case FlowStage::Output:
  default:
    return "Output";
  }
}

inline FlowStage dropReasonToFlowStage(FCWDropReason reason) {
  switch (reason) {
  case FCWDropReason::CamAge:
    return FlowStage::CameraFresh;
  case FCWDropReason::NoRadar:
    return FlowStage::HasRadar;
  case FCWDropReason::SpeedStale:
    return FlowStage::SpeedFresh;
  case FCWDropReason::ClassFilter:
    return FlowStage::ClassRelevant;
  case FCWDropReason::RangeGate:
    return FlowStage::RangeOk;
  case FCWDropReason::LowQuality:
    return FlowStage::QualityOk;
  case FCWDropReason::LowClosingSpeed:
    return FlowStage::ClosingOk;
  case FCWDropReason::OutOfPath:
    return FlowStage::InPath;
  case FCWDropReason::None:
  default:
    return FlowStage::Fusion;
  }
}

inline FlowLaneModel makeRejectedFlowLane(const FCWDebugRejected &item) {
  FlowLaneModel lane;
  lane.object_id = item.object_id;
  lane.is_rejection = true;
  lane.terminal_stage = dropReasonToFlowStage(item.drop_reason);
  lane.terminal_label = fcwDropReasonName(item.drop_reason);
  return lane;
}

inline FlowStage runnerUpToFlowStage(const FCWDebugCandidate &best,
                                     const FCWDebugCandidate &runner) {
  if (runner.active_level < best.active_level ||
      runner.desired_level < best.desired_level ||
      runner.ttc_immediate_warn != best.ttc_immediate_warn ||
      runner.ttc_immediate_critical != best.ttc_immediate_critical ||
      runner.ttc_warn_floor != best.ttc_warn_floor ||
      runner.ttc_last_ditch != best.ttc_last_ditch) {
    return FlowStage::StateMachine;
  }
  if (std::abs(runner.risk_score - best.risk_score) > 1e-4f ||
      std::abs(runner.base_risk_score - best.base_risk_score) > 1e-4f) {
    return FlowStage::RiskMix;
  }
  return FlowStage::Output;
}

inline FlowLaneModel makeRunnerUpFlowLane(const FCWDebugCandidate &best,
                                          const FCWDebugCandidate &runner) {
  FlowLaneModel lane;
  lane.object_id = runner.object_id;
  lane.is_rejection = false;
  lane.terminal_stage = runnerUpToFlowStage(best, runner);
  lane.terminal_label =
      runner.comparison_reason.empty() ? "runner-up" : runner.comparison_reason;
  return lane;
}

} // namespace adas::bev
