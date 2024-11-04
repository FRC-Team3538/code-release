package com.robojackets.subsystems.vision;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N6;
import java.util.Optional;
import lombok.Builder;
import lombok.Data;

public abstract class VisionModule {
  @Data
  @Builder
  public static class VisionEstimate {
    private final double timestamp;
    private final Pose3d estimate;
    private final Vector<N6> stdDevs;
  }

  public Optional<VisionEstimate> getNewPoseEstimate() {
    return Optional.empty();
  }

  public Optional<VisionEstimate> getNewPoseEstimate(Pose3d reference) {
    return getNewPoseEstimate();
  }
}
