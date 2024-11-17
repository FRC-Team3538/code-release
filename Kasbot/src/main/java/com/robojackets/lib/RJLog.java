package com.robojackets.lib;

import com.choreo.lib.ChoreoTrajectoryState;
import com.robojackets.ctre.SwerveDrivetrain.SwerveDriveState;
import com.robojackets.ctre.SwerveRequest.SwerveControlRequestParameters;
import com.robojackets.subsystems.vision.VisionModule.VisionEstimate;
import dev.doglog.DogLog;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;

public class RJLog extends DogLog {

  public static void log(String key, Measure<?> measure) {
    log(String.format("%s/%s", key, measure.unit().name()), measure.magnitude());
  }

  public static void log(String key, Vector<?> vector) {
    log(key, vector.getData());
  }

  public static void log(String key, ChoreoTrajectoryState state) {
    var pose = new Pose2d(state.x, state.y, Rotation2d.fromRadians(state.heading));
    var speeds = new ChassisSpeeds(state.velocityX, state.velocityY, state.angularVelocity);
    var forces = new SwerveModuleState[4];

    for (int i = 0; i < 4; i++) {
      var forceX = state.moduleForcesX[i];
      var forceY = state.moduleForcesY[i];
      var angle = new Rotation2d(forceX, forceY);
      var magnitude = Math.sqrt(forceX * forceX + forceY * forceY);
      forces[i] = new SwerveModuleState(magnitude, angle);
    }

    log(String.format("%s/Timestamp", key), state.timestamp);
    log(String.format("%s/Pose", key), pose);
    log(String.format("%s/Speeds", key), speeds);
    log(String.format("%s/ModuleForces", key), forces);
  }

  public static void log(String key, VisionEstimate estimate) {
    log(String.format("%s/Timestamp", key), estimate.getTimestamp());
    log(String.format("%s/Estimate", key), estimate.getEstimate());
    log(String.format("%s/StdDevs", key), estimate.getStdDevs());
  }

  public static void log(String key, SwerveDriveState state) {
    log(String.format("%s/SuccessfulDaqs", key), state.SuccessfulDaqs);
    log(String.format("%s/FailedDaqs", key), state.FailedDaqs);
    log(String.format("%s/Pose", key), state.Pose);
    log(String.format("%s/Speeds", key), state.speeds);
    log(String.format("%s/ModuleStates", key), state.ModuleStates);
    log(String.format("%s/ModuleTargets", key), state.ModuleTargets);
    log(String.format("%s/OdometryPeriod", key), state.OdometryPeriod);
  }

  public static void log(String key, SwerveControlRequestParameters requestParameters) {
    log(String.format("%s/Speed", key), requestParameters.currentChassisSpeed);
    log(String.format("%s/Pose", key), requestParameters.currentPose);
    log(String.format("%s/Timestamp", key), requestParameters.timestamp);
    log(String.format("%s/ModulePositions", key), requestParameters.swervePositions);
    log(String.format("%s/OperatorForward", key), requestParameters.operatorForwardDirection);
    log(String.format("%s/UpdatePeriod", key), requestParameters.updatePeriod);
  }
}
