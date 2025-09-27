#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/VisionManager.h>
#include <BaselineWalkingController/states/VisionApplyAvoid.h>

using namespace BWC;

void VisionApplyAvoid::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BaselineWalkingController&>(ctl_);
  auto pol = ctl.visionManager_->policy();

  ctl.footManager_->clearFootstepQueue();
  Eigen::Vector3d target(pol.aroundForward,0,0);
  std::vector<Eigen::Vector3d> wps = {
    {0.0, pol.sideStep, 0.0},
    {pol.aroundForward, pol.sideStep, 0.0},
    {pol.aroundForward, 0.0, 0.0}
  };
  ctl.footManager_->walkToRelativePose(target, 0, wps);

  output("DONE");
}
bool VisionApplyAvoid::run(mc_control::fsm::Controller &) { return true; }
void VisionApplyAvoid::teardown(mc_control::fsm::Controller &) {}

EXPORT_SINGLE_STATE("BWC::VisionApplyAvoid", VisionApplyAvoid)