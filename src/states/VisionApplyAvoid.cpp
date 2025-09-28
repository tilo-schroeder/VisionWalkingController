#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/VisionManager.h>
#include <BaselineWalkingController/states/VisionApplyAvoid.h>

using namespace BWC;

void VisionApplyAvoid::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BaselineWalkingController&>(ctl_);
  auto pol = ctl.visionManager_->policy();

  // Strategy switch
  if(pol.avoidMode == AvoidMode::Curve)
  {
    // Original C-path
    ctl.footManager_->clearFootstepQueue();
    Eigen::Vector3d target(pol.aroundForward,0,0);
    std::vector<Eigen::Vector3d> wps = {
      {0.0, pol.sideStep, 0.0},
      {pol.aroundForward, pol.sideStep, 0.0},
      {pol.aroundForward, 0.0, 0.0}
    };
    ctl.footManager_->walkToRelativePose(target, 0, wps);
    output("DONE");
    return;
  }

  // SideStep-until-clear mode
  stepsDone_   = 0;
  clearCount_  = 0;
  lastPlanTime_ = ctl.t();

  // Ensure DS and start from a clean slate
  ctl.footManager_->clearFootstepQueue();
}
bool VisionApplyAvoid::run(mc_control::fsm::Controller & ctl_) { 
  auto & ctl = static_cast<BaselineWalkingController&>(ctl_);
  auto pol = ctl.visionManager_->policy();
  if(pol.avoidMode == AvoidMode::Curve) { return true; } // already DONE in start()

  // SideStep-until-clear control loop
  const bool inDS = ctl.footManager_->getCurrentContactFeet().size() == 2;
  auto r = ctl.visionManager_->reading();
  auto d = ctl.visionManager_->decision();

  // Count consecutive "clear" frames (anything not AVOID)
  if(r.valid && d != ObstacleDecision::AVOID) {
    clearCount_++;
  } else {
    clearCount_ = 0;
  }

  // Finish condition: stable clear for N ticks
  if(clearCount_ >= pol.clearStableCount) {
    output("DONE");
    return true;
  }

  // Safety cap
  if(stepsDone_ >= pol.maxSideSteps) {
    mc_rtc::log::warning("[VisionApplyAvoid] Reached maxSideSteps={} without clearing obstacle; finishing.",
                         pol.maxSideSteps);
    output("DONE");
    return true;
  }

  // When queue is empty and we are in DS (with some lead), schedule the next sidestep
  auto dsLead = [&](){
    if(ctl.footManager_->footstepQueue().empty()) return pol.minDSLead + 1.0;
    const auto & fs = ctl.footManager_->footstepQueue().front();
    return std::max(0.0, fs.swingStartTime - ctl.t());
  }();

  if(inDS && ctl.footManager_->footstepQueue().empty() && dsLead >= pol.minDSLead) {
    // Plan a single lateral move
    const double dy = pol.sidestepSign >= 0 ? std::abs(pol.sideStep) : -std::abs(pol.sideStep);
    Eigen::Vector3d delta(0.0, dy, 0.0);
    ctl.footManager_->walkToRelativePose(delta, 0 /*yaw*/, {} /*wps*/);
    stepsDone_++;
    lastPlanTime_ = ctl.t();
  }

  return false; // keep running

}
void VisionApplyAvoid::teardown(mc_control::fsm::Controller &) {}

EXPORT_SINGLE_STATE("BWC::VisionApplyAvoid", VisionApplyAvoid)