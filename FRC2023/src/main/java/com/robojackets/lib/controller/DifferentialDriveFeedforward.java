// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.robojackets.lib.controller;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;

/** A helper class which computes the feedforward outputs for a differential drive drivetrain. */
public class DifferentialDriveFeedforward {
  private final LinearPlantInversionFeedforward<N2, N2, N2> m_feedforward;

  /**
   * Creates a new DifferentialDriveFeedforward with the specified parameters.
   *
   * @param plant The linear system this feedforward is ideal for.
   * @param dtSeconds the discretization timestep.
   */
  public DifferentialDriveFeedforward(LinearSystem<N2, N2, N2> plant, double dtSeconds) {
    m_feedforward = new LinearPlantInversionFeedforward<>(plant, dtSeconds);
  }

  /**
   * Creates a new DifferentialDriveFeedforward with the specified parameters.
   *
   * @param plant The linear system this feedforward is ideal for.
   */
  public DifferentialDriveFeedforward(LinearSystem<N2, N2, N2> plant) {
    m_feedforward = new LinearPlantInversionFeedforward<>(plant, 0.02);
  }

  /**
   * Creates a new DifferentialDriveFeedforward with the specified parameters.
   *
   * @param kVLinear The linear velocity gain in volts per (meters per second).
   * @param kALinear The linear acceleration gain in volts per (meters per second squared).
   * @param kVAngular The angular velocity gain in volts per (radians per second).
   * @param kAAngular The angular acceleration gain in volts per (radians per second squared).
   * @param trackwidth The distance between the differential drive's left and right wheels, in
   *     meters.
   */
  public DifferentialDriveFeedforward(
      double kVLinear, double kALinear, double kVAngular, double kAAngular, double trackwidth) {
    this(
        LinearSystemId.identifyDrivetrainSystem(
            kVLinear, kALinear, kVAngular, kAAngular, trackwidth));
  }

  /**
   * Creates a new DifferentialDriveFeedforward with the specified parameters.
   *
   * @param kVLinear The linear velocity gain in volts per (meters per second).
   * @param kALinear The linear acceleration gain in volts per (meters per second squared).
   * @param kVAngular The angular velocity gain in volts per (meters per second).
   * @param kAAngular The angular acceleration gain in volts per (meters per second squared).
   */
  public DifferentialDriveFeedforward(
      double kVLinear, double kALinear, double kVAngular, double kAAngular) {
    this(LinearSystemId.identifyDrivetrainSystem(kVLinear, kALinear, kVAngular, kAAngular));
  }

  /**
   * Calculates the differential drive feedforward inputs given velocity setpoints.
   *
   * @param currentLeftVelocity The current left velocity of the differential drive in
   *     meters/second.
   * @param nextLeftVelocity The next left velocity of the differential drive in meters/second.
   * @param currentRightVelocity The current right velocity of the differential drive in
   *     meters/second.
   * @param nextRightVelocity The next right velocity of the differential drive in meters/second.
   * @return A DifferentialDriveWheelVoltages object containing the computed feedforward voltages.
   */
  public DifferentialDriveWheelVoltages calculate(
      double currentLeftVelocity,
      double nextLeftVelocity,
      double currentRightVelocity,
      double nextRightVelocity) {
    var r = VecBuilder.fill(currentLeftVelocity, currentRightVelocity);
    var nextR = VecBuilder.fill(nextLeftVelocity, nextRightVelocity);
    var u = m_feedforward.calculate(r, nextR);
    return new DifferentialDriveWheelVoltages(u.get(0, 0), u.get(1, 0));
  }
}
