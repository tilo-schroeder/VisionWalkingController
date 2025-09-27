#pragma once

#include <BaselineWalkingController/State.h>

namespace BWC
{
struct VisionApplyAvoid : State
{
public:
  /** \brief Start. */
  void start(mc_control::fsm::Controller & ctl) override;

  /** \brief Run. */
  bool run(mc_control::fsm::Controller & ctl) override;

  /** \brief Teardown. */
  void teardown(mc_control::fsm::Controller & ctl) override;

};
} // namespace BWC