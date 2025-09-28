#include <sys/syscall.h>

#include <mc_tasks/CoMTask.h>
#include <mc_tasks/FirstOrderImpedanceTask.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/OrientationTask.h>

#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/CentroidalManager.h>
#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/VisionManager.h>
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

  if(config().has("VisionManager"))
  {
    visionManager_ = std::make_shared<VisionManager>(this, config()("VisionManager"));
  }
  else
  {
    mc_rtc::log::warning("[BaselineWalkingController] VisionManager configuration is missing.");
  }

  // Setup anchor
  setDefaultAnchor();

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
      mc_rtc::log::info("{}", s);
    }
  }

  mc_rtc::log::info("Robot name={} module={}", robot().name(), robot().module().name);

  if(visionManager_) { visionManager_->reset(); }

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

  if(enableManagerUpdate_) {
    if(visionManager_) visionManager_->update();
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
  if(visionManager_) visionManager_->stop();

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