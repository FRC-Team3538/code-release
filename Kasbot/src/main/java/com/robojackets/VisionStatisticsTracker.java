package com.robojackets;

import com.robojackets.constants.VisionConstants;
import com.robojackets.lib.amquake.statistics.CornerStatistics;
import com.robojackets.lib.amquake.statistics.TargetStatistics;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.HashMap;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionStatisticsTracker {
  // #################################
  private String photonCameraName;
  private final double kBufferLengthSeconds = 20;
  // #################################

  private PhotonCamera photonCamera;
  private PhotonPoseEstimator photonEstimator;
  private AprilTagFieldLayout tagLayout;
  private HashMap<Integer, TargetStatistics> photonTargetStatsMap;
  private TargetStatistics photonMultitagStats;
  private NetworkTable photonCameraTable;
  private DoubleArrayPublisher photonEstPoseArrayPublisher;

  public VisionStatisticsTracker(String cameraName) {
    photonCameraName = cameraName;
    tagLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();

    photonCamera = new PhotonCamera(photonCameraName);
    photonEstimator =
        new PhotonPoseEstimator(
            tagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            photonCamera,
            VisionConstants.frontCameraTransform);
    photonTargetStatsMap = new HashMap<Integer, TargetStatistics>();
    photonMultitagStats = new TargetStatistics("Photon Multitag Stats", kBufferLengthSeconds);

    photonCameraTable =
        NetworkTableInstance.getDefault().getTable("photonvision").getSubTable(photonCameraName);
    photonEstPoseArrayPublisher = photonCameraTable.getDoubleArrayTopic("EstPoseArray").publish();
  }

  public void updatePhotonStats() {
    var result = photonCamera.getLatestResult();
    double timestamp = result.getTimestampSeconds();

    // single-tag stats
    for (var target : result.getTargets()) {
      String name = "Photon Target(" + target.getFiducialId() + ") Stats";
      if (!photonTargetStatsMap.containsKey(target.getFiducialId())) {
        photonTargetStatsMap.put(
            target.getFiducialId(), new TargetStatistics(name, kBufferLengthSeconds));
      }
      var stats = photonTargetStatsMap.get(target.getFiducialId());
      stats.update(target, timestamp);
    }

    // estimated pose stats (needs tag layout!)
    var estimation = photonEstimator.update(result);
    if (estimation.isEmpty()) return;
    var estimatedPose = estimation.get().estimatedPose;
    photonMultitagStats.update(
        estimatedPose, CornerStatistics.allTargetCorners(estimation.get().targetsUsed), timestamp);

    double[] estPoseArray = {
      estimatedPose.getX(),
      estimatedPose.getY(),
      estimatedPose.getZ(),
      Math.toDegrees(estimatedPose.getRotation().getX()),
      Math.toDegrees(estimatedPose.getRotation().getY()),
      Math.toDegrees(estimatedPose.getRotation().getZ())
    };
    photonEstPoseArrayPublisher.set(estPoseArray);
  }
}
