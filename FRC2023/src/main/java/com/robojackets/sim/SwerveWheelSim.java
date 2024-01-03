// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.robojackets.sim;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

/** Represents a simulated elevator mechanism. */
public class SwerveWheelSim extends LinearSystemSim<N2, N1, N1> {
  // Gearbox for the elevator.
  private final DCMotor m_gearbox;

  // Gearing between the motors and the output.
  private final double m_gearing;

  // The radius of the wheel.
  private final double m_wheelRadius;

  /**
   * Creates a simulated elevator mechanism.
   *
   * @param plant The linear system that represents the elevator.
   * @param gearbox The type of and number of motors in the elevator gearbox.
   * @param gearing The gearing of the elevator (numbers greater than 1 represent reductions).
   * @param drumRadiusMeters The radius of the wheel.
   */
  public SwerveWheelSim(
      LinearSystem<N2, N1, N1> plant, DCMotor gearbox, double gearing, double wheelRadiusMeters) {
    this(plant, gearbox, gearing, wheelRadiusMeters, null);
  }

  /**
   * Creates a simulated elevator mechanism.
   *
   * @param plant The linear system that represents the elevator.
   * @param gearbox The type of and number of motors in the elevator gearbox.
   * @param gearing The gearing of the elevator (numbers greater than 1 represent reductions).
   * @param wheelRadiusMeters The radius of the wheel.
   * @param measurementStdDevs The standard deviations of the measurements.
   */
  public SwerveWheelSim(
      LinearSystem<N2, N1, N1> plant,
      DCMotor gearbox,
      double gearing,
      double wheelRadiusMeters,
      Matrix<N1, N1> measurementStdDevs) {
    super(plant, measurementStdDevs);
    m_gearbox = gearbox;
    m_gearing = gearing;
    m_wheelRadius = wheelRadiusMeters;
  }

  /**
   * Returns the position of the elevator.
   *
   * @return The position of the elevator.
   */
  public double getPositionMeters() {
    return getOutput(0);
  }

  /**
   * Returns the velocity of the elevator.
   *
   * @return The velocity of the elevator.
   */
  public double getVelocityMetersPerSecond() {
    return m_x.get(1, 0);
  }

  /**
   * Returns the elevator current draw.
   *
   * @return The elevator current draw.
   */
  @Override
  public double getCurrentDrawAmps() {
    // I = V / R - omega / (Kv * R)
    // Reductions are greater than 1, so a reduction of 10:1 would mean the motor is
    // spinning 10x faster than the output
    // v = r w, so w = v/r
    double motorVelocityRadPerSec = getVelocityMetersPerSecond() / m_wheelRadius * m_gearing;
    var appliedVoltage = m_u.get(0, 0);
    return m_gearbox.getCurrent(motorVelocityRadPerSec, appliedVoltage)
        * Math.signum(appliedVoltage);
  }

  /**
   * Sets the input voltage for the elevator.
   *
   * @param volts The input voltage.
   */
  public void setInputVoltage(double volts) {
    setInput(volts);
  }

  /**
   * Updates the state of the elevator.
   *
   * @param currentXhat The current state estimate.
   * @param u The system inputs (voltage).
   * @param dtSeconds The time difference between controller updates.
   */
  @Override
  protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
    // Calculate updated x-hat from Runge-Kutta.
    var updatedXhat =
        NumericalIntegration.rkdp(
            (x, _u) -> m_plant.getA().times(x).plus(m_plant.getB().times(_u)),
            currentXhat,
            u,
            dtSeconds);
    return updatedXhat;
  }
}
