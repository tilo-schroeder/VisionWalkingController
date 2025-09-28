# VisionWalkingController
Humanoid walking controller with various baseline methods

[Original BaselineWalkingController](https://github.com/isri-aist/BaselineWalkingController)
[![LICENSE](https://img.shields.io/github/license/isri-aist/BaselineWalkingController)](https://github.com/isri-aist/BaselineWalkingController/blob/master/LICENSE)

# Additions
Our additions to the walking controller can be configured by adding the following block to etc/BaselineWalkingController.in.yaml

```yaml
VisionManager:
  Cameras:
    left:  ${left_cam_name_if_not_autodetected}
    right: ${right_cam_name_if_not_autodetected}
  OnnxPath: model-v6.onnx
  Policy:
    avoidMode: SideStep        # SideStep or Curve
    sidestepSign: 1            # +1 left (Y+), -1 right (Y-)
    maxSideSteps: 10
    clearStableCount: 3
    maxStepHeight: 0.15
    hysteresis: 0.02
    minInferPeriod: 0.10
    minDSLead: 0.15
    stepClearanceMargin: 0.30
    approachStride: 0.25
    sideStep: 0.50
    aroundForward: 1.00
    onlyInDoubleSupport: true
```


# Usage
The controller can be brought up with the instructions for the original BaselineWalkingController, but this was build for the [LookOMotion](https://github.com/tilo-schroeder/LookOMotion) project and it is easiest to get running in the devcontainer environment for that project.

