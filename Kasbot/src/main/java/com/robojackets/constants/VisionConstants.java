package com.robojackets.constants;

import com.robojackets.subsystems.vision.DisabledVisionModule;
import com.robojackets.subsystems.vision.PhotonVisionModule;
import com.robojackets.subsystems.vision.VisionModule;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.io.IOException;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.SimCameraProperties;

public class VisionConstants {
  public static final Transform3d frontCameraTransform =
      new Transform3d(
          new Translation3d(Units.inchesToMeters(27.25 / 2.0 - 1.5), 0, 0.55),
          new Rotation3d(0, Units.degreesToRadians(-28.125), 0));
  public static final Transform3d rearCameraTransform =
      new Transform3d(
          new Translation3d(-Units.inchesToMeters(27.25 / 2.0 - 7.0 + 1.75), 0, 0.24),
          new Rotation3d(0, Units.degreesToRadians(-28.125), Units.degreesToRadians(180)));

  public static final String frontCameraId = "Front Apriltag";
  public static final String rearCameraId = "Rear Apriltag";
  public static final PhotonCamera frontCamera = new PhotonCamera(frontCameraId);
  public static final PhotonCamera rearCamera = new PhotonCamera(rearCameraId);

  public static final PhotonPoseEstimator frontPoseEstimator =
      new PhotonPoseEstimator(
          FieldConstants.pairedAprilTags,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          frontCamera,
          frontCameraTransform);
  public static final PhotonPoseEstimator rearPoseEstimator =
      new PhotonPoseEstimator(
          FieldConstants.pairedAprilTags,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          rearCamera,
          rearCameraTransform);

  {
    frontPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
    rearPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
  }

  // Currently on practice bot
  public static final VisionModule camera =
      new PhotonVisionModule(
          List.of(frontCamera, rearCamera), List.of(frontPoseEstimator, rearPoseEstimator));

  public static final VisionModule noCamera = new DisabledVisionModule();

  private static SimCameraProperties loadCameraProperties(String path, int width, int height) {
    try {
      return new SimCameraProperties(path, width, height);
    } catch (IOException ex) {
      System.out.printf("Could not read front camera properties: %s", ex.getMessage());
      ex.printStackTrace();
    }

    return null;
  }
  // Sim only
  // public static Supplier<SimCameraProperties> frontCameraProperties =
  //     () ->
  //         new SimCameraProperties(); //
  // loadCameraProperties("vision/photon_calibration_front_ov9281.json", 1280, 800);
  // public static Supplier<SimCameraProperties> rearCameraProperties =
  //     () ->
  //         new SimCameraProperties(); //
  // loadCameraProperties("vision/photon_calibration_rear_ov9281.json", 1280, 800);

  // public static final VisionModule camera =
  //     new VisionSim(
  //         List.of(frontCamera, rearCamera),
  //         List.of(frontPoseEstimator, rearPoseEstimator),
  //         List.of(frontCameraProperties.get(), rearCameraProperties.get()));

  // Currently on Comp bot
  // public static final VisionModule camera = new DisabledVisionModule();
}
