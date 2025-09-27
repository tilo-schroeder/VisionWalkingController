#include <mc_rtc/logging.h>
#include <chrono>

#include <BaselineWalkingController/VisionManager.h>
#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/FootManager.h>

// Bring in your inline implementation (class definition) of OnnxModel
// If your predictor lives in namespace BWC in that file, wrap include accordingly.
namespace BWC {
#  include <BaselineWalkingController/predictor.cpp>
}


using namespace BWC;

VisionManager::VisionManager(BaselineWalkingController * ctl, const mc_rtc::Configuration & cfg)
: ctl_(ctl)
{
  if(cfg.has("Cameras"))
  {
    auto cams = cfg("Cameras");
    if(cams.has("left"))  leftCamName_  = std::string(cams("left"));
    if(cams.has("right")) rightCamName_ = std::string(cams("right"));
  }
  if(cfg.has("OnnxPath")) onnxPath_ = std::string(cfg("OnnxPath"));

  if(cfg.has("Policy"))
  {
    auto pc = cfg("Policy");
    if(pc.has("maxStepHeight"))       policy_.maxStepHeight       = (double) pc("maxStepHeight");
    if(pc.has("hysteresis"))          policy_.hysteresis          = (double) pc("hysteresis");
    if(pc.has("minInferPeriod"))      policy_.minInferPeriod      = (double) pc("minInferPeriod");
    if(pc.has("minDSLead"))           policy_.minDSLead           = (double) pc("minDSLead");
    if(pc.has("stepClearanceMargin")) policy_.stepClearanceMargin = (double) pc("stepClearanceMargin");
    if(pc.has("approachStride"))      policy_.approachStride      = (double) pc("approachStride");
    if(pc.has("sideStep"))            policy_.sideStep            = (double) pc("sideStep");
    if(pc.has("aroundForward"))       policy_.aroundForward       = (double) pc("aroundForward");
    if(pc.has("onlyInDoubleSupport")) policy_.onlyInDoubleSupport = (bool)   pc("onlyInDoubleSupport");
  }

  // After config is read, you can safely instantiate the model
  // model_ = std::make_unique<OnnxModel>(onnxPath_, /*num_threads*/4);
  model_ = std::make_shared<OnnxModel>(onnxPath_, 4);
}

VisionManager::~VisionManager(){ stop(); }

void VisionManager::reset()
{
  running_.store(true);
  enabled_.store(true);
  // start worker
  worker_ = std::thread(&VisionManager::threadLoop_, this);
}

void VisionManager::stop()
{
  running_.store(false);
  if(worker_.joinable()) worker_.join();
}

void VisionManager::update()
{
  // Nothing heavy here; just sanity if you ever need to react each tick
}

void VisionManager::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
//   using namespace mc_rtc::gui;

//   // First block: controls/labels (no Image here)
//   gui.addElement({"Vision"},
//     mc_rtc::gui::Checkbox("Enable",
//       [this]() { return enabled(); },
//       [this](bool v){ enable(v); }),

//     mc_rtc::gui::ArrayLabel("Reading [stamp, h_filt, stable]",
//       [this](){
//         return std::array<double,3>{
//           lastStamp_.load(),
//           static_cast<double>(heightFilt_.load()),
//           static_cast<double>(stableCount_.load())
//         };
//       }),

//     mc_rtc::gui::Label("Decision",
//       [this](){
//         auto d = decision_.load();
//         return d==ObstacleDecision::STEP_UP ? "STEP_UP" :
//                d==ObstacleDecision::AVOID   ? "AVOID"   : "NONE";
//       }),

//     mc_rtc::gui::Button("Force NONE",    [this](){ forceDecision(ObstacleDecision::NONE); }),
//     mc_rtc::gui::Button("Force STEP_UP", [this](){ forceDecision(ObstacleDecision::STEP_UP); }),
//     mc_rtc::gui::Button("Force AVOID",   [this](){ forceDecision(ObstacleDecision::AVOID); }),

//     mc_rtc::gui::NumberInput("maxStepHeight [m]",
//       [this](){ return policy_.maxStepHeight; },
//       [this](double v){ policy_.maxStepHeight = v; }),

//     mc_rtc::gui::NumberInput("hysteresis [m]",
//       [this](){ return policy_.hysteresis; },
//       [this](double v){ policy_.hysteresis = v; }),

//     mc_rtc::gui::NumberInput("minInferPeriod [s]",
//       [this](){ return policy_.minInferPeriod; },
//       [this](double v){ policy_.minInferPeriod = v; }),

//     mc_rtc::gui::Checkbox("Only in DS",
//       [this](){ return policy_.onlyInDoubleSupport; },
//       [this](bool v){ policy_.onlyInDoubleSupport = v; })
//   );

//   // Optional second block: Image widget (in its own addElement, so no dangling comma)
// #if __has_include(<mc_rtc/gui/Image.h>)
//   gui.addElement({"Vision"},
//     mc_rtc::gui::Image("Stereo preview",
//       [this]() -> const std::vector<uint8_t> & { return previewRGB_; },
//       [this](){ return previewW_; },
//       [this](){ return previewH_; },
//       [](){ return false; }) // RGB (not RGBA)
//   );
// #endif

//   // Plot uses the 5-arg signature in your mc_rtc
//   gui.addPlot("Vision/Height",
//     mc_rtc::gui::plot::X("t", [this](){ return ctl_->t(); }),
//     mc_rtc::gui::plot::Y("h_filt",
//       [this](){ return (double)heightFilt_.load(); },
//       mc_rtc::gui::Color::Blue,
//       mc_rtc::gui::plot::Style::Line,
//       mc_rtc::gui::plot::Side::Left)
//   );
}

void VisionManager::addToLogger(mc_rtc::Logger & logger)
{
  logger.addLogEntry("vision_stamp",       [this](){ return lastStamp_.load(); });
  logger.addLogEntry("vision_height_filt", [this](){ return heightFilt_.load(); });
  logger.addLogEntry("vision_stable",      [this](){ return stableCount_.load(); });
  logger.addLogEntry("vision_decision",    [this](){ return (int)decision_.load(); });
}

VisionReading VisionManager::reading() const
{
  return VisionReading{ lastStamp_.load(), heightFilt_.load(), valid_.load(), stableCount_.load() };
}
ObstacleDecision VisionManager::decision() const { return decision_.load(); }
void VisionManager::forceDecision(ObstacleDecision d) { decision_.store(d); }

bool VisionManager::grabStereo_(std::vector<uint8_t> &L, std::vector<uint8_t> &R, int &W, int &H, double &stamp)
{
  if(!ctl_->datastore().has("MuJoCo::GetCameraRGB")) return false;
  // You can also support CameraSensor path here if available.
  bool okL = ctl_->datastore().call<bool, const std::string&, std::vector<uint8_t>&, int&, int&, double&>(
               "MuJoCo::GetCameraRGB", leftCamName_, L, W, H, stamp);
  bool okR = ctl_->datastore().call<bool, const std::string&, std::vector<uint8_t>&, int&, int&, double&>(
               "MuJoCo::GetCameraRGB", rightCamName_, R, W, H, stamp);
  return okL && okR && W>0 && H>0 && L.size()==R.size() && !L.empty();
}

void VisionManager::threadLoop_()
{
  auto wall = [](){ using namespace std::chrono;
    return duration_cast<duration<double>>(steady_clock::now().time_since_epoch()).count(); };

  while(running_.load())
  {
    if(!enabled_.load()) { std::this_thread::sleep_for(std::chrono::milliseconds(50)); continue; }

    // Gating by DS (optional)
    if(policy_.onlyInDoubleSupport) {
      if(ctl_->footManager_->getCurrentContactFeet().size() != 2) { std::this_thread::sleep_for(std::chrono::milliseconds(10)); continue; }
    }

    // Throttle
    double now = wall();
    if(lastInferWall_ > 0.0 && now - lastInferWall_ < policy_.minInferPeriod) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2)); continue;
    }

    // Grab cameras
    std::vector<uint8_t> L, R; int W=0,H=0; double stamp=0.0;
    if(!grabStereo_(L,R,W,H,stamp)) { std::this_thread::sleep_for(std::chrono::milliseconds(10)); continue; }

    // Keep side-by-side preview for GUI (optional)
    {
      std::lock_guard<std::mutex> lk(previewMtx_);
      previewW_ = 2*W; previewH_ = H;
      previewRGB_.resize((size_t)previewW_*previewH_*3);
      for(int y=0;y<H;y++)for(int x=0;x<W;x++){
        size_t s=(size_t)(y*W+x)*3;
        size_t dl=(size_t)(y*(2*W)+x)*3, dr=(size_t)(y*(2*W)+(W+x))*3;
        previewRGB_[dl+0]=L[s+0]; previewRGB_[dl+1]=L[s+1]; previewRGB_[dl+2]=L[s+2];
        previewRGB_[dr+0]=R[s+0]; previewRGB_[dr+1]=R[s+1]; previewRGB_[dr+2]=R[s+2];
      }
    }

    // Preprocess & inference (keep it simple: planarize HWC->float[0..1])
    std::vector<float> input; input.reserve((size_t)W*H*2*3);
    // (example: concatenate L then R by channel-major, matching your model)
    for(int c=0;c<3;c++) for(int y=0;y<H;y++) for(int x=0;x<W;x++)
      input.push_back(L[(y*W+x)*3+(2-c)]/255.f);
    for(int c=0;c<3;c++) for(int y=0;y<H;y++) for(int x=0;x<W;x++)
      input.push_back(R[(y*W+x)*3+(2-c)]/255.f);

    float h = model_->process_frame(input);

    mc_rtc::log::info("HEIGHT: %s", h);

    // Simple IIR + stability
    if(!valid_.load()) { filt_ = h; valid_.store(true); stableCount_.store(1); }
    else { filt_ = 0.7f*filt_ + 0.3f*h; stableCount_.fetch_add(1); }

    // Decide with hysteresis
    double hi = policy_.maxStepHeight + policy_.hysteresis;
    double lo = policy_.maxStepHeight - policy_.hysteresis;
    auto prev = decision_.load();
    auto newD = prev;
    if(filt_ > hi) newD = ObstacleDecision::AVOID;
    else if(filt_ < lo) newD = ObstacleDecision::STEP_UP;
    // else keep previous

    // Publish snapshot
    heightFilt_.store(filt_);
    lastStamp_.store(stamp);
    decision_.store(newD);

    lastInferWall_ = now;
  }
}
