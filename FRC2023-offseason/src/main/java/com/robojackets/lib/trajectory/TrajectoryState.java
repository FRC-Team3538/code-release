package com.robojackets.lib.trajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import lombok.Builder;

@Builder
public class TrajectoryState implements Interpolatable<TrajectoryState> {
  private double timestamp;

  private double x;
  private double y;
  private double heading;

  private double velocityX;
  private double velocityY;
  private double angularVelocity;

  public double getTimestamp() {
    return timestamp;
  }

  public Pose2d getPose() {
    return new Pose2d(x, y, Rotation2d.fromRadians(heading));
  }

  public ChassisSpeeds getChassisSpeeds() {
    return new ChassisSpeeds(velocityX, velocityY, angularVelocity);
  }

  @Override
  public TrajectoryState interpolate(TrajectoryState endValue, double t) {
    double scale = (getTimestamp() - t) / (endValue.getTimestamp() - t);
    var interp_pose = getPose().interpolate(endValue.getPose(), scale);

    return TrajectoryState.builder()
        .x(interp_pose.getX())
        .y(interp_pose.getY())
        .heading(interp_pose.getRotation().getRadians())
        .velocityX(MathUtil.interpolate(this.velocityX, endValue.velocityX, scale))
        .velocityY(MathUtil.interpolate(this.velocityY, endValue.velocityY, scale))
        .angularVelocity(
            MathUtil.interpolate(this.angularVelocity, endValue.angularVelocity, scale))
        .build();
  }

  public double[] asArray() {
    return new double[] {
      timestamp, x, y, heading, velocityX, velocityY, angularVelocity,
    };
  }
}
