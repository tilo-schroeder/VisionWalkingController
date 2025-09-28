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
  auto r = ctl.visionManager_->reading();

  // pick next swing
  Foot nextSwing = Foot::Left;
  if(!ctl.footManager_->footstepQueue().empty())
  {
    nextSwing = ctl.footManager_->footstepQueue().front().foot;
  }

  // ensure DS
  ctl.footManager_->clearFootstepQueue();

  auto mid = sva::interpolate( ctl.footManager_->targetFootPose(Foot::Left),
                               ctl.footManager_->targetFootPose(Foot::Right), 0.5 );
  sva::PTransformd approachMid( sva::RotZ(0.0),
       mid.translation() + Eigen::Vector3d(pol.approachStride,0,0));

  double start = std::max(ctl.t() + 0.8,
                          ctl.footManager_->footstepQueue().empty()
                            ? ctl.t() : ctl.footManager_->footstepQueue().back().transitEndTime + 1e-3);

  auto fs1 = ctl.footManager_->makeFootstep(nextSwing, approachMid, start);
  ctl.footManager_->appendFootstep(fs1);

  sva::PTransformd onMid = approachMid; onMid.translation().z() += r.heightFilt;
  mc_rtc::Configuration swingCfg;
  swingCfg.add("type", "IndHorizontalVertical");

  // Peak height above the higher of start/end z: 
  const double dz = r.heightFilt + pol.stepClearanceMargin;
  swingCfg.add("verticalTopOffset", std::vector<double>{0.0, 0.0, dz});

  auto fs2 = ctl.footManager_->makeFootstep(opposite(nextSwing), onMid, fs1.transitEndTime);
  fs2.swingTrajConfig = swingCfg;
  ctl.footManager_->appendFootstep(fs2);

  output("DONE");
}

bool VisionApplyStepUp::run(mc_control::fsm::Controller & ctl_) { return true; }
void VisionApplyStepUp::teardown(mc_control::fsm::Controller &) {}

EXPORT_SINGLE_STATE("BWC::VisionApplyStepUp", VisionApplyStepUp)
