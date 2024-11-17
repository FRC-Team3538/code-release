package com.robojackets.oi;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.robojackets.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;

public class DriverControls {
  private final XboxController driver;

  public DriverControls(XboxController driver) {
    this.driver = driver;

    Preferences.initDouble("Controller/Driver/Left_Deadband", 0);
    Preferences.initDouble("Controller/Driver/Left_Limit", 1);
    Preferences.initDouble("Controller/Driver/Right_Deadband", 0);
    Preferences.initDouble("Controller/Driver/Right_Limit", 1);
  }

  public ChassisSpeeds fieldCentric(ControlMode mode) {
    var translation = fieldCentricInternal();
    var rotation = rotateInternal();

    return new ChassisSpeeds(
        translation.get(0) * Constants.maxLinearVelocity(mode, slow()).in(MetersPerSecond),
        translation.get(1) * Constants.maxLinearVelocity(mode, slow()).in(MetersPerSecond),
        rotation * Constants.maxAngularVelocity(mode, slow()).in(RadiansPerSecond));
  }

  public boolean shouldUseRobotCentric() {
    return driver.getPOV() != -1;
  }

  public boolean shouldManuallyRotate() {
    return rotateInternal() != 0;
  }

  public ChassisSpeeds robotCentric(ControlMode mode) {
    var translation = robotCentricInternal();
    var rotation = rotateInternal();

    return new ChassisSpeeds(
        translation.get(0) * Constants.maxLinearVelocity(mode, slow()).in(MetersPerSecond),
        translation.get(1) * Constants.maxLinearVelocity(mode, slow()).in(MetersPerSecond),
        rotation * Constants.maxAngularVelocity(mode, slow()).in(RadiansPerSecond));
  }

  public double rotateInternal() {
    var deadband = Preferences.getDouble("Controller/Driver/Right_Deadband", 0);
    var limit = Preferences.getDouble("Controller/Driver/Right_Limit", 1);

    var rotate = -driver.getRightX();

    return MathUtil.applyDeadband(rotate, deadband, limit);
  }

  private Vector<N2> robotCentricInternal() {
    var pov = driver.getPOV();

    if (pov == -1) {
      return VecBuilder.fill(0, 0);
    } else {
      var angle = Units.Degrees.of(pov).in(Units.Radians);
      return VecBuilder.fill(cos(angle) * 0.33, sin(-angle) * 0.33);
    }
  }

  private Vector<N2> fieldCentricInternal() {
    var deadband = Preferences.getDouble("Controller/Driver/Left_Deadband", 0);
    var limit = Preferences.getDouble("Controller/Driver/Left_Limit", 1);

    var translate = VecBuilder.fill(-driver.getLeftY(), -driver.getLeftX());

    var magnitude = translate.norm();
    if (magnitude == 0) {
      return translate;
    }

    return translate.unit().times(MathUtil.applyDeadband(magnitude, deadband, limit));
  }

  public boolean aim() {
    return driver.getLeftTriggerAxis() > 0.5;
  }

  public boolean slow() {
    return driver.getLeftBumper();
  }
}
