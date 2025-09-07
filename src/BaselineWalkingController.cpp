#include <sys/syscall.h>

#include <mc_tasks/CoMTask.h>
#include <mc_tasks/FirstOrderImpedanceTask.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/OrientationTask.h>

#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/CentroidalManager.h>
#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/centroidal/CentroidalManagerDdpZmp.h>
#include <BaselineWalkingController/centroidal/CentroidalManagerFootGuidedControl.h>
#include <BaselineWalkingController/centroidal/CentroidalManagerIntrinsicallyStableMpc.h>
#include <BaselineWalkingController/centroidal/CentroidalManagerPreviewControlZmp.h>

#include <fstream>
#include <vector>
#include <string>
#include <stdexcept>
#include <algorithm>

using namespace BWC;

BaselineWalkingController::BaselineWalkingController(mc_rbdyn::RobotModulePtr rm,
                                                     double dt,
                                                     const mc_rtc::Configuration & _config,
                                                     bool allowEmptyManager)
: mc_control::fsm::Controller(rm, dt, _config)
{
  // Get the robot-specific configuration
  auto rconfig = config()("robots")(robot().module().name);
  if(rconfig.empty())
  {
    mc_rtc::log::error_and_throw("[BaselineWalkingController] {} section is empty, please provide a configuration",
                                 robot().module().name);
  }
  // Load the robot's configuration into the controller's configuration
  config().load(rconfig);
  // Load extra-overwrites
  auto overwriteConfigList = config()("OverwriteConfigList", mc_rtc::Configuration());
  auto overwriteConfigKeys = config()("OverwriteConfigKeys", std::vector<std::string>{});
  for(const auto & overwriteConfigKey : overwriteConfigKeys)
  {
    if(!overwriteConfigList.has(overwriteConfigKey))
    {
      mc_rtc::log::error_and_throw(
          "[BaselineWalkingController] {} in OverwriteConfigKeys but not in OverwriteConfigList", overwriteConfigKey);
    }
    config().load(overwriteConfigList(overwriteConfigKey));
  }

  config()("controllerName", name_);

  if(config().has("Cameras"))
  {
    leftCamName_  = std::string(config()("Cameras")("left",  std::string("")));
    rightCamName_ = std::string(config()("Cameras")("right", std::string("")));
  }
  
  // Setup tasks
  if(config().has("CoMTask"))
  {
    comTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::CoMTask>(solver(), config()("CoMTask"));
    comTask_->name("CoMTask");
  }
  else
  {
    mc_rtc::log::warning("[BaselineWalkingController] CoMTask configuration is missing.");
  }
  if(config().has("BaseOrientationTask"))
  {
    baseOriTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::OrientationTask>(solver(), config()("BaseOrientationTask"));
    baseOriTask_->name("BaseOriTask");
  }
  else
  {
    mc_rtc::log::warning("[BaselineWalkingController] BaseOrientationTask configuration is missing.");
  }
  if(config().has("FootTaskList"))
  {
    for(const auto & footTaskConfig : config()("FootTaskList"))
    {
      Foot foot = strToFoot(footTaskConfig("foot"));
      footTasks_.emplace(
          foot, mc_tasks::MetaTaskLoader::load<mc_tasks::force::FirstOrderImpedanceTask>(solver(), footTaskConfig));
      footTasks_.at(foot)->name("FootTask_" + std::to_string(foot));
    }
  }
  else
  {
    mc_rtc::log::warning("[BaselineWalkingController] FootTaskList configuration is missing.");
  }

  // Setup managers
  if(config().has("FootManager"))
  {
    footManager_ = std::make_shared<FootManager>(this, config()("FootManager"));
  }
  else
  {
    mc_rtc::log::warning("[BaselineWalkingController] FootManager configuration is missing.");
  }
  if(config().has("CentroidalManager"))
  {
    std::string centroidalManagerMethod = config()("CentroidalManager")("method", std::string(""));
    if(centroidalManagerMethod == "PreviewControlZmp")
    {
      centroidalManager_ = std::make_shared<CentroidalManagerPreviewControlZmp>(this, config()("CentroidalManager"));
    }
    else if(centroidalManagerMethod == "DdpZmp")
    {
      centroidalManager_ = std::make_shared<CentroidalManagerDdpZmp>(this, config()("CentroidalManager"));
    }
    else if(centroidalManagerMethod == "FootGuidedControl")
    {
      centroidalManager_ = std::make_shared<CentroidalManagerFootGuidedControl>(this, config()("CentroidalManager"));
    }
    else if(centroidalManagerMethod == "IntrinsicallyStableMpc")
    {
      centroidalManager_ =
          std::make_shared<CentroidalManagerIntrinsicallyStableMpc>(this, config()("CentroidalManager"));
    }
    else
    {
      if(!allowEmptyManager)
      {
        mc_rtc::log::error_and_throw("[BaselineWalkingController] Invalid centroidalManagerMethod: {}.",
                                     centroidalManagerMethod);
      }
    }
  }
  else
  {
    mc_rtc::log::warning("[BaselineWalkingController] CentroidalManager configuration is missing.");
  }

  // Setup anchor
  setDefaultAnchor();

  predictor_ = std::make_shared<OnnxModel>("model-v5.onnx", 4);

  mc_rtc::log::success("[BaselineWalkingController] Constructed.");
}

void BaselineWalkingController::reset(const mc_control::ControllerResetData & resetData)
{
  mc_control::fsm::Controller::reset(resetData);

  enableManagerUpdate_ = false;

  // auto-discover the correct camera names for the robot
  if((leftCamName_.empty() || rightCamName_.empty()) && datastore().has("MuJoCo::ListCameras"))
  {
    const auto cams = datastore().call<std::vector<std::string>>("MuJoCo::ListCameras");

    // Helper: prefer cameras that (1) start with the robot name and (2) contain tokens
    auto pick = [&](std::initializer_list<std::string> tokens) -> std::string {
      auto matches = [&](const std::string &name, bool strictRobot) {
        bool robotOk = !strictRobot || name.rfind(robot().name(), 0) == 0; // prefix == robot name
        if(!robotOk) return false;
        for(const auto &t : tokens) { if(name.find(t) == std::string::npos) return false; }
        return true;
      };
      // 1) strict (prefix = robot name), 2) relaxed (any camera)
      for(const auto &c : cams) if(matches(c, true )) return c;
      for(const auto &c : cams) if(matches(c, false)) return c;
      return {};
    };

    if(leftCamName_.empty())
    {
      leftCamName_ = pick({"eye","left"});
      if(leftCamName_.empty()) leftCamName_ = pick({"left"});
    }
    if(rightCamName_.empty())
    {
      rightCamName_ = pick({"eye","right"});
      if(rightCamName_.empty()) rightCamName_ = pick({"right"});
    }

    if(leftCamName_.empty() || rightCamName_.empty())
    {
      mc_rtc::log::warning("[BaselineWalkingController] Could not auto-detect stereo cameras. "
                          "Available: {}", fmt::join(cams, ", "));
    }
    else
    {
      mc_rtc::log::success("[BaselineWalkingController] Using cameras L='{}'  R='{}'",
                          leftCamName_, rightCamName_);
    }
  }

  std::vector<std::string> cams;
  if(datastore().has("MuJoCo::ListCameras"))
  {
    cams = datastore().call<std::vector<std::string>>("MuJoCo::ListCameras");
    mc_rtc::log::info("MuJoCo exposed cameras: \n");
    for(auto &s: cams) {
      mc_rtc::log::info(s);
    }
  }

  mc_rtc::log::info("Robot name={} module={}", robot().name(), robot().module().name);

  // Print message to set priority
  long tid = static_cast<long>(syscall(SYS_gettid));
  mc_rtc::log::info("[BaselineWalkingController] TID is {}. Run the following command to set high priority:\n  sudo "
                    "renice -n -20 -p {}",
                    tid, tid);
  mc_rtc::log::info("[BaselineWalkingController] You can check the current priority by the following command:\n  ps -p "
                    "`pgrep choreonoid` -o pid,tid,args,ni,pri,wchan m");

  mc_rtc::log::success("[BaselineWalkingController] Reset.");
}

bool BaselineWalkingController::run()
{
  t_ += dt();

  auto clearQueue = [&](){
    // leaves the current swing if any, but you're in DS when calling this
    footManager_->clearFootstepQueue();
  };

  auto lastEnd = [&](){
    return footManager_->footstepQueue().empty()
        ? t()
        : footManager_->footstepQueue().back().transitEndTime;
  };

  auto inDoubleSupport = [&](){
    return footManager_->getCurrentContactFeet().size() == 2;
  };

  // Only sample if (a) DS, (b) NN not run too recently, and (c) new camera stamp
  if(enableManagerUpdate_ && inDoubleSupport())
  {
    int W=0,H=0; double stamp=0.0;
    // throttle model calls
    if(t_ - perc_.lastInferTime >= policy_.minInferPeriod)
    {
      std::vector<uint8_t> L, R;
      if(datastore().has("MuJoCo::GetCameraRGB"))
      {
        datastore().call<bool, const std::string&, std::vector<uint8_t>&, int&, int&, double&>(
          "MuJoCo::GetCameraRGB", leftCamName_, L, W, H, stamp);
        datastore().call<bool, const std::string&, std::vector<uint8_t>&, int&, int&, double&>(
          "MuJoCo::GetCameraRGB", rightCamName_, R, W, H, stamp);

        if(stamp > perc_.lastStamp && W>0 && H>0 && L.size()==R.size() && !L.empty())
        {
          
          SaveStereoSideBySide(L, R, W, H, "stereo_preview.ppm");
          
          // Interleave channels
          std::vector<uint8_t> stereo; stereo.reserve(H*W*2*3);
          
          // FIXME: Possible culprit for inference problems

          for(int c=0;c<3;++c)
            for(int w=0;w<W;++w)
              for(int h=0;h<H;++h){
                stereo.push_back(L[(h*W+w)*3+(2-c)]);
                
          }
          for(int c=0;c<3;++c)
            for(int w=0;w<W;++w)
              for(int h=0;h<H;++h){
                stereo.push_back(R[(h*W+w)*3+(2-c)]);
          }
          std::vector<float> stereo_f(stereo.begin(), stereo.end());
          for (int i = 0; i < stereo_f.size(); ++i) stereo_f[i] /= 255;

          float h = predictor_->process_frame(stereo_f);
          mc_rtc::log::info("HEIGHT: {}", h);

          // simple IIR filter + stability counter
          if(!perc_.valid){ perc_.heightFilt = h; perc_.valid = true; perc_.stableCount = 1; }
          else { perc_.heightFilt = 0.7f*perc_.heightFilt + 0.3f*h; perc_.stableCount++; }

          perc_.lastStamp = stamp;
          perc_.lastInferTime = t_;
        }
      }
    }
  }

  auto decide = [&]() -> ObstacleDecision {
    if(!perc_.valid || perc_.stableCount < 3) return ObstacleDecision::NONE; // wait for a few filtered samples
    const double hi = policy_.maxStepHeight + policy_.hysteresis;
    const double lo = policy_.maxStepHeight - policy_.hysteresis;
    if(perc_.heightFilt > hi) return ObstacleDecision::AVOID;
    if(perc_.heightFilt < lo) return ObstacleDecision::STEP_UP;
    return decision_; // within band: keep previous to avoid flapping
  };

  // Only (re)decide if we still have enough time before next swing start
  auto dsLeadTime = [&]()->double {
    // Estimate: time until next swing = if queue empty, we will create it; else front.transitStartTime - t_
    if(footManager_->footstepQueue().empty()) return policy_.minDSLead + 1.0; // generous if idle
    const auto &fs = footManager_->footstepQueue().front();
    return std::max(0.0, fs.swingStartTime - t_);
  }();

  if(inDoubleSupport() && dsLeadTime >= policy_.minDSLead)
  {
    auto newDec = decide();
    if(newDec != decision_)
    {
      decision_ = newDec;
      decisionStamp_ = t_;
      mc_rtc::log::info("[BWC] Obstacle decision: {}  (h_filt={:.3f} m)",
                        (decision_==ObstacleDecision::STEP_UP?"STEP_UP":
                        decision_==ObstacleDecision::AVOID  ?"AVOID":"NONE"),
                        perc_.heightFilt);
    }
  }

  auto stopVelModeIfNeeded = [&](){
    if(footManager_->velModeEnabled()) footManager_->endVelMode();
  };

  // Build one landing step with elevated Z
  auto enqueueStepUp = [&](BWC::Foot swingFoot){
    stopVelModeIfNeeded();

    // mid-pose = between current feet, projected to ground
    auto mid = sva::interpolate( footManager_->targetFootPose(BWC::Foot::Left),
                                footManager_->targetFootPose(BWC::Foot::Right), 0.5 );
    // approach: move forward a bit keeping Z
    sva::PTransformd approachMid = sva::PTransformd(
        sva::RotZ(0.0),
        mid.translation() + Eigen::Vector3d(policy_.approachStride, 0.0, 0.0));
    // double start = t() + 0.8; // 0.8s from now; Possibly tweak
    double start = std::max(t() + 0.8, lastEnd() + 1e-3);

    // 1) approach on level
    auto fs1 = footManager_->makeFootstep(swingFoot, approachMid, start);
    footManager_->appendFootstep(fs1);

    // 2) step ON the block: same XY as approach but lifted by block height
    sva::PTransformd onMid = approachMid;
    onMid.translation().z() += perc_.heightFilt;

    mc_rtc::Configuration swingCfg;
    swingCfg.add("type", "IndHorizontalVertical");        // or "CubicSplineSimple"
    swingCfg.add("clearance", perc_.heightFilt + policy_.stepClearanceMargin); // depends on swing class keys

    auto fs2 = footManager_->makeFootstep(opposite(swingFoot), onMid, fs1.transitEndTime);
    fs2.swingTrajConfig = swingCfg;
    footManager_->appendFootstep(fs2);
  };

  auto enqueueAvoid = [&](){
    stopVelModeIfNeeded();
    Eigen::Vector3d target( policy_.aroundForward, 0.0, 0.0 );     // final forward progress
    std::vector<Eigen::Vector3d> wps = {
      { 0.0,  policy_.sideStep, 0.0 },
      { policy_.aroundForward,  policy_.sideStep, 0.0 },
      { policy_.aroundForward,  0.0, 0.0 }
    };
    footManager_->walkToRelativePose(target, /*lastFootstepNum=*/0, wps);
  };

  // Trigger only once per DS segment
  static ObstacleDecision lastApplied = ObstacleDecision::NONE;
  if(inDoubleSupport() && decision_ != ObstacleDecision::NONE && decision_ != lastApplied)
  {
    // Choose swing foot: by default, step with the foot that would swing next
    BWC::Foot nextSwing = BWC::Foot::Left;
    if(!footManager_->footstepQueue().empty())
      nextSwing = footManager_->footstepQueue().front().foot; // front is the *swing* in queue

    if(decision_ == ObstacleDecision::STEP_UP) {
      clearQueue();
      enqueueStepUp(nextSwing);
      // enqueueAvoid();
    }
      
    if(decision_ == ObstacleDecision::AVOID) {
      clearQueue();
      enqueueAvoid();
    }   

    lastApplied = decision_;
    predictor_->change_side();
  }

  if(enableManagerUpdate_) {
    footManager_->update();
    centroidalManager_->update();
  }

  return mc_control::fsm::Controller::run();
}

void BaselineWalkingController::stop()
{
  // Clean up tasks
  solver().removeTask(comTask_);
  solver().removeTask(baseOriTask_);
  for(const auto & foot : Feet::Both)
  {
    solver().removeTask(footTasks_.at(foot));
  }

  // Clean up managers
  footManager_->stop();
  centroidalManager_->stop();

  // Clean up anchor
  setDefaultAnchor();

  mc_control::fsm::Controller::stop();
}

void BaselineWalkingController::setDefaultAnchor()
{
  std::string anchorName = "KinematicAnchorFrame::" + robot().name();
  if(datastore().has(anchorName))
  {
    datastore().remove(anchorName);
  }
  datastore().make_call(anchorName, [this](const mc_rbdyn::Robot & robot) {
    return sva::interpolate(robot.surfacePose(footManager_->surfaceName(Foot::Left)),
                            robot.surfacePose(footManager_->surfaceName(Foot::Right)), 0.5);
  });
}

// One wide image: [LEFT | RIGHT]
void BaselineWalkingController::SaveStereoSideBySide(const std::vector<uint8_t>& L,
                                        const std::vector<uint8_t>& R,
                                        int W, int H,
                                        const std::string& path)
{
    if ((int)L.size() != W*H*3 || (int)R.size() != W*H*3) throw std::runtime_error("SaveStereoSideBySide: size mismatch");
    std::vector<uint8_t> out((size_t)H * (2*W) * 3);
    for (int y=0; y<H; ++y) {
        for (int x=0; x<W; ++x) {
            size_t s = (size_t)(y*W + x)*3;
            size_t dL = (size_t)(y*(2*W) + x)*3;
            size_t dR = (size_t)(y*(2*W) + (W + x))*3;
            out[dL+0]=L[s+0]; out[dL+1]=L[s+1]; out[dL+2]=L[s+2];
            out[dR+0]=R[s+0]; out[dR+1]=R[s+1]; out[dR+2]=R[s+2];
        }
    }
    SaveRGB_PPM(out, 2*W, H, path);
}

void BaselineWalkingController::SaveRGB_PPM(const std::vector<uint8_t>& rgb,
                               int W, int H,
                               const std::string& path)
{
    if ((int)rgb.size() != W*H*3) throw std::runtime_error("SaveRGB_PPM: size mismatch");
    std::ofstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("SaveRGB_PPM: cannot open " + path);
    f << "P6\n" << W << " " << H << "\n255\n";
    f.write(reinterpret_cast<const char*>(rgb.data()), (std::streamsize)rgb.size());
}