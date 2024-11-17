package com.robojackets.subsystems.vision;

import com.robojackets.lib.RJLog;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionModule extends VisionModule {
  private final List<PhotonCamera> cameras;
  private final List<PhotonPoseEstimator> poseEstimators;
  private final ArrayList<Optional<EstimatedRobotPose>> estimates = new ArrayList<>();

  public PhotonVisionModule(List<PhotonCamera> cameras, List<PhotonPoseEstimator> poseEstimators) {
    assert (cameras.size() > 0);
    assert (poseEstimators.size() == cameras.size());

    this.cameras = cameras;
    this.poseEstimators = poseEstimators;
  }

  @Override
  public Optional<VisionEstimate> getNewPoseEstimate(Pose3d reference) {
    estimates.clear();

    for (int idx = 0; idx < cameras.size(); ++idx) {
      var estimator = poseEstimators.get(idx);
      var camera = cameras.get(idx);

      estimator.setReferencePose(reference);

      var estimate = estimator.update();
      estimates.add(estimate);

      recordOutput(camera, estimator, estimate);
    }

    EstimatedRobotPose best = null;

    for (var estimate : estimates) {
      if (best == null && estimate.isPresent()) {
        best = estimate.get();
      } else if (estimate.isPresent()) {
        var current = estimate.get();
        if (FirstIsBetter(current, best)) {
          best = current;
        }
      }
    }

    if (best == null) {
      return Optional.empty();
    }

    // if (best.targetsUsed.size() == 1) {
    //   // drop really noisy stuff
    //   return Optional.empty();
    // }

    return Optional.of(new VisionEstimate(best.timestampSeconds, best.estimatedPose, stdevs(best)));
  }

  private void recordOutput(
      PhotonCamera camera, PhotonPoseEstimator estimator, Optional<EstimatedRobotPose> estimate) {
    List<AprilTag> detectedTags = new ArrayList<>();

    for (PhotonTrackedTarget target : camera.getLatestResult().targets) {
      estimator
          .getFieldTags()
          .getTagPose(target.getFiducialId())
          .ifPresent(
              targetPose -> {
                var aprilTag = new AprilTag(target.getFiducialId(), targetPose);
                detectedTags.add(aprilTag);
              });
    }

    Pose3d[] arrayTags = detectedTags.stream().map(tag -> tag.pose).toArray(len -> new Pose3d[len]);
    RJLog.log(String.format("Vision/%s/Tags", camera.getName()), arrayTags);

    estimate.ifPresent(
        inner -> RJLog.log(String.format("Vision/%s/Pose", camera.getName()), inner.estimatedPose));
  }

  private boolean FirstIsBetter(EstimatedRobotPose first, EstimatedRobotPose second) {
    var first_stdev = stdevs(first);
    var second_stdev = stdevs(second);

    var first_position_stdev =
        Math.sqrt(
            first_stdev.get(0) * first_stdev.get(0) + first_stdev.get(1) * first_stdev.get(1));
    var second_position_stdev =
        Math.sqrt(
            second_stdev.get(0) * second_stdev.get(0) + second_stdev.get(1) * second_stdev.get(1));

    if (first.targetsUsed.size() == second.targetsUsed.size()) {
      return first_position_stdev < second_position_stdev;
    }

    return first.targetsUsed.size() > second.targetsUsed.size();
  }

  protected Vector<N6> stdevs(EstimatedRobotPose estimate) {
    if (estimate.targetsUsed.size() == 1) {
      var dist = AverageDistance(estimate.targetsUsed);
      var x = 5.95e-4 + 1.07e-4 * dist + 9.4e-4 * dist * dist;
      var y = -1.72e-4 + 2.62e-4 * dist + 4.88e-5 * dist * dist;
      var z = -1.9e-4 - 9.43e-5 * dist + 3.92e-4 * dist * dist;
      var rx = 0.0562 - 0.123 * dist + 0.0886 * dist * dist;
      var ry = -1 + 1.44 * dist + 4.11e-3 * dist * dist;
      var rz = 0.223 - 0.366 * dist + 0.211 * dist * dist;

      return VecBuilder.fill(
          x,
          y,
          z,
          Units.degreesToRadians(rx),
          Units.degreesToRadians(ry),
          Units.degreesToRadians(rz));
    } else {
      var dist = AverageDistance(estimate.targetsUsed);
      var x = 3.53e-4 * Math.exp(0.589 * dist);
      var y = 5.88e-04 * Math.exp(0.646 * dist);
      var z = 4.37e-4 * Math.exp(0.705 * dist);
      var rx = 0.041 * Math.exp(0.174 * dist);
      var ry = 0.0173 * Math.exp(0.48 * dist);
      var rz = 0.0393 * Math.exp(0.375 * dist);

      return VecBuilder.fill(
          x,
          y,
          z,
          Units.degreesToRadians(rx),
          Units.degreesToRadians(ry),
          Units.degreesToRadians(rz));
    }
  }

  private double AverageDistance(List<PhotonTrackedTarget> targets) {
    var distance = 0;
    for (var target : targets) {
      distance += target.getBestCameraToTarget().getTranslation().getNorm();
    }

    return distance / targets.size();
  }
}
