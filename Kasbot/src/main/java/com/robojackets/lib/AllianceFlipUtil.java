// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.robojackets.lib;

import com.robojackets.constants.FieldConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Utility functions for flipping from the blue to red alliance. */
public class AllianceFlipUtil {
  /** Flips an x coordinate to the correct side of the field based on the current alliance color. */
  public static double apply(double xCoordinate) {
    if (shouldFlip()) {
      return FieldConstants.fieldLength - xCoordinate;
    } else {
      return xCoordinate;
    }
  }

  /** Flips a translation to the correct side of the field based on the current alliance color. */
  public static Translation2d apply(Translation2d translation) {
    if (shouldFlip()) {
      return new Translation2d(apply(translation.getX()), translation.getY());
    } else {
      return translation;
    }
  }

  /** Flips a rotation based on the current alliance color. */
  public static Rotation2d rotate(Rotation2d rotation) {
    return apply(rotation, shouldFlip());
  }

  private static Rotation2d HALF_TURN = Rotation2d.fromDegrees(180);

  /** Flips a rotation based on the current alliance color. */
  public static Rotation2d rotate(Rotation2d rotation, boolean flip) {
    if (flip) {
      return HALF_TURN.plus(rotation);
    } else {
      return rotation;
    }
  }

  /** Flips a rotation based on the current alliance color. */
  public static Rotation2d mirror(Rotation2d rotation) {
    return mirror(rotation, shouldFlip());
  }

  /** Flips a rotation based on the current alliance color. */
  public static Rotation2d mirror(Rotation2d rotation, boolean flip) {
    if (flip) {
      return new Rotation2d(-rotation.getCos(), rotation.getSin());
    } else {
      return rotation;
    }
  }

  /** Flips a rotation based on the current alliance color. */
  public static Rotation2d apply(Rotation2d rotation) {
    return apply(rotation, shouldFlip());
  }

  /** Flips a rotation based on the current alliance color. */
  public static Rotation2d apply(Rotation2d rotation, boolean flip) {
    if (flip) {
      return new Rotation2d(-rotation.getCos(), rotation.getSin());
    } else {
      return rotation;
    }
  }

  /** Flips a pose to the correct side of the field based on the current alliance color. */
  public static Pose2d apply(Pose2d pose) {
    if (shouldFlip()) {
      return new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()));
    } else {
      return pose;
    }
  }

  public static Translation3d apply(Translation3d translation3d) {
    if (shouldFlip()) {
      return new Translation3d(
          apply(translation3d.getX()), translation3d.getY(), translation3d.getZ());
    } else {
      return translation3d;
    }
  }

  /** Flips a rotation based on the current alliance color. */
  public static Rotation3d apply(Rotation3d rotation) {
    if (shouldFlip()) {
      var flippedYaw = apply(Rotation2d.fromRadians(rotation.getZ()));
      return new Rotation3d(rotation.getX(), rotation.getY(), flippedYaw.getRadians());
    } else {
      return rotation;
    }
  }

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
  }
}
