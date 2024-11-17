package com.robojackets.subsystems.vision;

import com.ctre.phoenix6.Utils;
import com.robojackets.constants.FieldConstants;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSim extends PhotonVisionModule {
  /* For Sim Purposes Only */
  private VisionSystemSim visionSim;

  public VisionSim(
      List<PhotonCamera> cameras,
      List<PhotonPoseEstimator> estimators,
      List<SimCameraProperties> cameraProperties) {
    super(cameras, estimators);
    if (!Utils.isSimulation()) {
      throw new IllegalStateException("Cannot start VisionSim while not in simulation!");
    }

    visionSim = new VisionSystemSim("main");
    visionSim.addAprilTags(FieldConstants.aprilTags);

    for (int idx = 0; idx < cameras.size(); ++idx) {
      var camera = cameras.get(idx);
      var estimator = estimators.get(idx);
      var property = cameraProperties.get(idx);

      var cameraSim = new PhotonCameraSim(camera, property);
      cameraSim.enableRawStream(true);
      cameraSim.enableProcessedStream(true);
      cameraSim.enableDrawWireframe(true);

      visionSim.addCamera(cameraSim, estimator.getRobotToCameraTransform());
    }

    SmartDashboard.putData("VisionSim/DebugField", visionSim.getDebugField());
  }

  @Override
  public Optional<VisionEstimate> getNewPoseEstimate() {
    return Optional.empty();
  }

  @Override
  public Optional<VisionEstimate> getNewPoseEstimate(Pose3d reference) {
    visionSim.update(reference);

    return super.getNewPoseEstimate(reference);
  }
}
