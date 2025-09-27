#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/VisionManager.h>
#include <BaselineWalkingController/states/VisionMonitorState.h>

using namespace BWC;

static constexpr const char * OUT_STEP_UP = "STEP_UP";
static constexpr const char * OUT_AVOID   = "AVOID";

void VisionMonitorState::start(mc_control::fsm::Controller & ctl_)
{
  output("RUNNING"); // parallel state semantics
}

bool VisionMonitorState::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BaselineWalkingController&>(ctl_);
  if(!ctl.visionManager_) return true;

  // Optional gating: enough DS lead time
  auto pol = ctl.visionManager_->policy();
  bool inDS = ctl.footManager_->getCurrentContactFeet().size() == 2;

  // Time until next swing
  auto dsLead = [&](){
    if(ctl.footManager_->footstepQueue().empty()) return pol.minDSLead + 1.0;
    const auto & fs = ctl.footManager_->footstepQueue().front();
    return std::max(0.0, fs.swingStartTime - ctl.t());
  }();

  auto r = ctl.visionManager_->reading();
  auto d = ctl.visionManager_->decision();

  if(r.valid && r.stableCount >= 3 && inDS && dsLead >= pol.minDSLead)
  {
    if(d == ObstacleDecision::STEP_UP) { output(OUT_STEP_UP); }
    else if(d == ObstacleDecision::AVOID) { output(OUT_AVOID); }
  }
  return false; // parallel/monitor – never finishes
}

EXPORT_SINGLE_STATE("BWC::VisionMonitor", VisionMonitorState)
