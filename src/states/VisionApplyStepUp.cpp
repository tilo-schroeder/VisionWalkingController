#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/VisionManager.h>
#include <BaselineWalkingController/FootTypes.h>
#include <BaselineWalkingController/states/VisionApplyStepUp.h>
#include <mc_rtc/logging.h>

using namespace BWC;

void VisionApplyStepUp::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BaselineWalkingController&>(ctl_);
  auto pol = ctl.visionManager_->policy();
  auto r   = ctl.visionManager_->reading();

  // Decide the swing foot: keep any queued choice, otherwise use the front foot
  Foot nextSwing = Foot::Left;
  if(!ctl.footManager_->footstepQueue().empty())
  {
    nextSwing = ctl.footManager_->footstepQueue().front().foot;
  }
  else
  {
    const auto & l = ctl.footManager_->targetFootPose(Foot::Left).translation();
    const auto & rr = ctl.footManager_->targetFootPose(Foot::Right).translation();
    nextSwing = (l.x() > rr.x()) ? Foot::Left : Foot::Right; // lead with the front foot
  }

  // Ensure DS before planning a single step
  ctl.footManager_->clearFootstepQueue();

  // Reference heading from the mid pose (forward = robot heading)
  auto mid = sva::interpolate(
      ctl.footManager_->targetFootPose(Foot::Left),
      ctl.footManager_->targetFootPose(Foot::Right), 0.5);

  // Build the single target: advance forward, and raise by obstacle height
  auto swingStart = ctl.footManager_->targetFootPose(nextSwing);
  sva::PTransformd target = swingStart;
  target.translation() += mid.rotation() * Eigen::Vector3d(pol.approachStride, 0.0, 0.0); // FORWARD
  target.translation().z() += r.heightFilt;                                               // and UP

  // Timing
  double start = std::max(ctl.t() + 0.8, ctl.t());

  // One step: swing nextSwing directly to the forward+up target
  mc_rtc::Configuration swingCfg;
  swingCfg.add("type", "IndHorizontalVertical");
  swingCfg.add("clearance", r.heightFilt + pol.stepClearanceMargin); // peak above higher of start/end

  auto fs = ctl.footManager_->makeFootstep(nextSwing, target, start);
  fs.swingTrajConfig = swingCfg;
  ctl.footManager_->appendFootstep(fs);

  output("DONE");
}

bool VisionApplyStepUp::run(mc_control::fsm::Controller & ctl_) { return true; }
void VisionApplyStepUp::teardown(mc_control::fsm::Controller &) {}

EXPORT_SINGLE_STATE("BWC::VisionApplyStepUp", VisionApplyStepUp)
