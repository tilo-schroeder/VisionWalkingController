#pragma once

#include <mc_control/api.h>
#include <mc_control/fsm/Controller.h>
#include <BaselineWalkingController/FootTypes.h>

namespace mc_tasks
{
struct CoMTask;
struct OrientationTask;

namespace force
{
struct FirstOrderImpedanceTask;
}
} // namespace mc_tasks

namespace BWC
{
class FootManager;
class CentroidalManager;
class VisionManager;

/** \brief Humanoid walking controller with various baseline methods. */
struct MC_CONTROL_DLLAPI BaselineWalkingController : public mc_control::fsm::Controller
{
public:
  /** \brief Constructor.
      \param rm robot module
      \param dt control timestep
      \param _config controller configuration
      \param allowEmptyManager whether to allow the managers to be empty (assuming initialized in the parent class)
   */
  BaselineWalkingController(mc_rbdyn::RobotModulePtr rm,
                            double dt,
                            const mc_rtc::Configuration & _config,
                            bool allowEmptyManager = false);

  /** \brief Reset a controller.

      This method is called when starting the controller.
   */
  void reset(const mc_control::ControllerResetData & resetData) override;

  /** \brief Run a controller.

      This method is called every control period.
   */
  bool run() override;

  /** \brief Stop a controller.

      This method is called when stopping the controller.
   */
  void stop() override;

  /** \brief Get controller name. */
  inline const std::string & name() const
  {
    return name_;
  }

  /** \brief Get current time. */
  inline double t() const noexcept
  {
    return t_;
  }

  /** \brief Get timestep. */
  inline double dt() const
  {
    return solver().dt();
  }

  /** \brief Set default anchor. */
  void setDefaultAnchor();

  void SaveRGB_PPM(const std::vector<uint8_t>& rgb,
                               int W, int H,
                               const std::string& path);
  
  void SaveStereoSideBySide(const std::vector<uint8_t>& L,
                                        const std::vector<uint8_t>& R,
                                        int W, int H,
                                        const std::string& path);

public:
  //! CoM task
  std::shared_ptr<mc_tasks::CoMTask> comTask_;

  //! Base link orientation task
  std::shared_ptr<mc_tasks::OrientationTask> baseOriTask_;

  //! Foot tasks
  std::unordered_map<Foot, std::shared_ptr<mc_tasks::force::FirstOrderImpedanceTask>> footTasks_;

  //! Foot manager
  std::shared_ptr<FootManager> footManager_;

  //! Centroidal manager
  std::shared_ptr<CentroidalManager> centroidalManager_;

  //! Vision manager
  std::shared_ptr<VisionManager> visionManager_;

  //! Whether to enable manager update
  bool enableManagerUpdate_ = false;

  struct PerceptionState
  {
    double lastStamp{0.0};
    double lastInferTime{ -1.0 };
    float  heightFilt{0.f};
    bool   valid{false};
    int    stableCount{0};
  };

  enum class ObstacleDecision { NONE, STEP_UP, AVOID };

  struct ObstaclePolicy
  {
    double maxStepHeight{0.2};     // [m] step ON if h <= this
    double hysteresis{0.02};        // [m] for stability
    double minInferPeriod{0.10};    // [s] run NN at most 10 Hz
    double minDSLead{0.15};         // [s] need this much time left in DS before next swing
    double stepClearanceMargin{0.05}; // [m] extra swing height above block
    double approachStride{0.25};    // [m] forward stride length
    double sideStep{0.5};          // [m] lateral for go-around
    double aroundForward{1.0};     // [m] forward distance while avoiding
  };

  PerceptionState perc_;
  ObstaclePolicy policy_;
  ObstacleDecision decision_{ObstacleDecision::NONE};
  double decisionStamp_{0.0};

protected:
  //! Controller name
  std::string name_ = "BWC";

  //! Current time [sec]
  double t_ = 0;

  std::string leftCamName_;
  std::string rightCamName_;
};
} // namespace BWC
