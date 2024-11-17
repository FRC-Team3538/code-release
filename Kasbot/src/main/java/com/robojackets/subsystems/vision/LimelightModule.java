package com.robojackets.subsystems.vision;

import com.robojackets.lib.RJLog;
import com.robojackets.lib.limelight.LimelightHelpers;
import com.robojackets.lib.limelight.LimelightHelpers.LimelightResults;
import com.robojackets.lib.limelight.LimelightHelpers.Results;
import edu.wpi.first.math.VecBuilder;
import java.util.Optional;

public class LimelightModule extends VisionModule {
  private final String limelight;

  private LimelightResults lastResults = new LimelightResults();

  {
    lastResults.targetingResults = new Results();
  }

  public LimelightModule(String limelight) {
    this.limelight = limelight;
  }

  @Override
  public Optional<VisionEstimate> getNewPoseEstimate() {
    var currentResults = maybeGetNewerVisionEstimate(lastResults, limelight);

    if (currentResults.isEmpty()) {
      return Optional.empty();
    }

    var total_latency =
        currentResults.get().targetingResults.latency_capture
            + currentResults.get().targetingResults.latency_pipeline
            + currentResults.get().targetingResults.latency_jsonParse;
    var estimate_timestamp =
        currentResults.get().targetingResults.timestamp_RIOFPGA_capture - total_latency * 0.001;

    lastResults = currentResults.get();

    RJLog.log(String.format("vision/%s/latency", limelight), total_latency);
    RJLog.log(
        String.format("vision/%s/capture", limelight),
        lastResults.targetingResults.timestamp_RIOFPGA_capture);
    RJLog.log(
        String.format("vision/%s/latency/capture", limelight),
        lastResults.targetingResults.latency_capture);
    RJLog.log(
        String.format("vision/%s/latency/pipeline", limelight),
        lastResults.targetingResults.latency_pipeline);
    RJLog.log(
        String.format("vision/%s/latency/parse", limelight),
        lastResults.targetingResults.latency_jsonParse);
    RJLog.log(
        String.format("vision/%s/estimate", limelight),
        lastResults.targetingResults.getBotPose3d_wpiBlue());
    RJLog.log(String.format("vision/%s/timestamp", limelight), estimate_timestamp);
    RJLog.log(
        String.format("vision/%s/fiducial_count", limelight),
        lastResults.targetingResults.targets_Fiducials.length);

    var measurement = currentResults.get().targetingResults.getBotPose3d_wpiBlue();

    if (lastResults.targetingResults.targets_Fiducials.length >= 2) {
      return Optional.of(
          VisionEstimate.builder()
              .timestamp(estimate_timestamp)
              .estimate(measurement)
              .stdDevs(VecBuilder.fill(0.1, 0.1, 0.1, 0.1, 0.1, 0.1))
              .build());
    } else {
      return Optional.of(
          VisionEstimate.builder()
              .timestamp(estimate_timestamp)
              .estimate(measurement)
              .stdDevs(VecBuilder.fill(1, 1, 1, 1, 1, 1))
              .build());
    }
  }

  private Optional<LimelightResults> maybeGetNewerVisionEstimate(
      LimelightResults results, String limelight) {
    if (!LimelightHelpers.tableExists(limelight)) {
      return Optional.empty();
    }

    var currentResults = LimelightHelpers.getLatestResults(limelight);

    if (currentResults.targetingResults == null) {
      return Optional.empty();
    }

    if (!currentResults.targetingResults.valid) {
      return Optional.empty();
    }

    if (currentResults.targetingResults.timestamp_LIMELIGHT_publish
        <= results.targetingResults.timestamp_LIMELIGHT_publish) {
      return Optional.empty();
    }

    if (currentResults.targetingResults.botpose_wpiblue[0] == 0
        && currentResults.targetingResults.botpose_wpiblue[1] == 0
        && currentResults.targetingResults.botpose_wpiblue[2] == 0
        && currentResults.targetingResults.botpose_wpiblue[3] == 0
        && currentResults.targetingResults.botpose_wpiblue[4] == 0
        && currentResults.targetingResults.botpose_wpiblue[5] == 0) {
      return Optional.empty();
    }

    // max 4 targets, typically 1-2
    for (var fiducial : currentResults.targetingResults.targets_Fiducials) {
      if (fiducial.getRobotPose_TargetSpace().getTranslation().getNorm() > 7) {
        return Optional.empty();
      }
    }

    return Optional.of(currentResults);
  }
}
