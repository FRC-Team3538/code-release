package com.robojackets.subsystems.vision;

import java.util.Optional;

public class DisabledVisionModule extends VisionModule {

  @Override
  public Optional<VisionEstimate> getNewPoseEstimate() {
    return Optional.empty();
  }
}
