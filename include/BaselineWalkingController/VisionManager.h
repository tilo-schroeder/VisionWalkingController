#pragma once
#include <mc_rtc/log/Logger.h>
#include <mc_rtc/Configuration.h>
#include <mc_rtc/gui.h>
#include <atomic>
#include <memory>
#include <thread>
#include <vector>
#include <string>
#include <mutex>

namespace BWC {

struct BaselineWalkingController;
class OnnxModel; 

enum class ObstacleDecision { NONE, STEP_UP, AVOID };

struct VisionPolicy
{
  double maxStepHeight{0.20};
  double hysteresis{0.02};
  double minInferPeriod{0.10};
  double minDSLead{0.15};
  double stepClearanceMargin{0.05};
  double approachStride{0.25};
  double sideStep{0.50};
  double aroundForward{1.00};
  bool   onlyInDoubleSupport{true};
  // Let mc_rtc load/save this type
  void load(const mc_rtc::Configuration & c)
  {
    c("maxStepHeight", maxStepHeight);
    c("hysteresis", hysteresis);
    c("minInferPeriod", minInferPeriod);
    c("minDSLead", minDSLead);
    c("stepClearanceMargin", stepClearanceMargin);
    c("approachStride", approachStride);
    c("sideStep", sideStep);
    c("aroundForward", aroundForward);
    c("onlyInDoubleSupport", onlyInDoubleSupport);
  }
  void save(mc_rtc::Configuration & c) const
  {
    c.add("maxStepHeight", maxStepHeight);
    c.add("hysteresis", hysteresis);
    c.add("minInferPeriod", minInferPeriod);
    c.add("minDSLead", minDSLead);
    c.add("stepClearanceMargin", stepClearanceMargin);
    c.add("approachStride", approachStride);
    c.add("sideStep", sideStep);
    c.add("aroundForward", aroundForward);
    c.add("onlyInDoubleSupport", onlyInDoubleSupport);
  }
};

struct VisionReading
{
  double stamp{0.0};
  float  heightFilt{0.f};
  bool   valid{false};
  int    stableCount{0};
};

class VisionManager
{
public:
  VisionManager(BaselineWalkingController * ctl, const mc_rtc::Configuration & cfg);
  ~VisionManager();

  void reset();
  void update();  // cheap, RT-safe (just snapshots)
  void stop();

  void addToGUI(mc_rtc::gui::StateBuilder & gui);
  void addToLogger(mc_rtc::Logger & logger);

  // Accessors used by FSM states
  VisionReading reading() const;
  ObstacleDecision decision() const;
  VisionPolicy & policy() { return policy_; }
  const VisionPolicy & policy() const { return policy_; }

  // UI hooks
  void forceDecision(ObstacleDecision d);
  void enable(bool on) { enabled_.store(on); }
  bool enabled() const { return enabled_.load(); }

  // Optional: expose the stereo preview for GUI
  const std::vector<uint8_t> & preview() const { return previewRGB_; }
  int previewW() const { return previewW_; }
  int previewH() const { return previewH_; }

private:
  void threadLoop_();               // non-RT loop
  bool grabStereo_(std::vector<uint8_t> & L, std::vector<uint8_t> & R, int & W, int & H, double & stamp);

private:
  BaselineWalkingController * ctl_{nullptr};
  VisionPolicy policy_;
  std::string leftCamName_, rightCamName_;
  std::string onnxPath_{"model-v5.onnx"};

  // non-RT resources
  std::shared_ptr<OnnxModel> model_;

  std::thread worker_;
  std::atomic<bool> running_{false};
  std::atomic<bool> enabled_{false};

  // shared snapshot (updated by thread, read by RT)
  std::atomic<double> lastStamp_{0.0};
  std::atomic<float>  heightFilt_{0.f};
  std::atomic<int>    stableCount_{0};
  std::atomic<bool>   valid_{false};
  std::atomic<ObstacleDecision> decision_{ObstacleDecision::NONE};

  // GUI preview
  mutable std::mutex previewMtx_;
  std::vector<uint8_t> previewRGB_;
  int previewW_{0}, previewH_{0};

  // internal
  double lastInferWall_{-1.0};
  float  filt_{0.f};
};

} // namespace BWC
