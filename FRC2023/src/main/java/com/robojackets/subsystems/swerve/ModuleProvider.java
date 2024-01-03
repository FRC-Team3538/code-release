package com.robojackets.subsystems.swerve;

import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import lombok.Builder;

@Builder
public class ModuleProvider {
  public enum ModulePosition {
    FRONT_RIGHT(Rotation2d.fromDegrees(0)),
    FRONT_LEFT(Rotation2d.fromDegrees(90)),
    REAR_LEFT(Rotation2d.fromDegrees(180)),
    REAR_RIGHT(Rotation2d.fromDegrees(270));

    private Rotation2d facing;

    private ModulePosition(Rotation2d facing) {
      this.facing = facing;
    }

    public Rotation2d transform(ModulePosition installedAt) {
      return this.facing.minus(installedAt.facing);
    }

    public Translation2d getPosition(double wheelBase, double trackWidth) {
      switch (this) {
        case FRONT_RIGHT:
          return new Translation2d(wheelBase / 2, -trackWidth / 2);
        case FRONT_LEFT:
          return new Translation2d(wheelBase / 2, trackWidth / 2);
        case REAR_LEFT:
          return new Translation2d(-wheelBase / 2, trackWidth / 2);
        case REAR_RIGHT:
          return new Translation2d(-wheelBase / 2, -trackWidth / 2);
        default:
          return new Translation2d();
      }
    }

    public String getName() {
      switch (this) {
        case FRONT_RIGHT:
          return "FRONT_RIGHT";
        case FRONT_LEFT:
          return "FRONT_LEFT";
        case REAR_LEFT:
          return "REAR_LEFT";
        case REAR_RIGHT:
          return "REAR_RIGHT";
        default:
          return null;
      }
    }
  }

  private SwerveModuleConfig moduleConfig;
  private double wheelBase;
  private double trackWidth;

  private FalconSwerveModuleIO createSwerveModuleIO(
      int driveID, int azimuthID, int encoderID, SwerveModuleConfig moduleConfig) {
    return new FalconSwerveModuleIO(
        new TalonFX(driveID, moduleConfig.getCanBus()),
        new TalonFX(azimuthID, moduleConfig.getCanBus()),
        new CANcoder(encoderID, moduleConfig.getCanBus()),
        moduleConfig.getDriveFeedforward(),
        moduleConfig.getAzimuthFeedforward());
  }

  public SwerveModule getModule1(ModulePosition position) {
    return new SwerveModule(
        moduleConfig.applyWithOffset(
            createSwerveModuleIO(0, 1, 20, moduleConfig),
            Rotation2d.fromRotations(0.164795)
                .plus(Rotation2d.fromRotations(0.5))
                .plus(ModulePosition.FRONT_LEFT.transform(position)),
            20),
        position.getPosition(wheelBase, trackWidth),
        position.getName());
  }

  public SwerveModule getModule2(ModulePosition position) {
    return new SwerveModule(
        moduleConfig.applyWithOffset(
            createSwerveModuleIO(2, 3, 21, moduleConfig),
            Rotation2d.fromRotations(-0.404541)
                .plus(Rotation2d.fromRotations(0.5))
                .plus(ModulePosition.FRONT_RIGHT.transform(position)),
            21),
        position.getPosition(wheelBase, trackWidth),
        position.getName());
  }

  public SwerveModule getModule3(ModulePosition position) {
    return new SwerveModule(
        moduleConfig.applyWithOffset(
            createSwerveModuleIO(4, 5, 22, moduleConfig),
            Rotation2d.fromRotations(0.215088)
                .plus(Rotation2d.fromRotations(0.5))
                .plus(ModulePosition.REAR_LEFT.transform(position)),
            22),
        position.getPosition(wheelBase, trackWidth),
        position.getName());
  }

  public SwerveModule getModule4(ModulePosition position) {
    return new SwerveModule(
        moduleConfig.applyWithOffset(
            createSwerveModuleIO(6, 7, 23, moduleConfig),
            Rotation2d.fromRotations(-0.283447)
                .plus(Rotation2d.fromRotations(0.5))
                .plus(ModulePosition.REAR_RIGHT.transform(position)),
            23),
        position.getPosition(wheelBase, trackWidth),
        position.getName());
  }

  public SwerveModule getModule5(ModulePosition position) {
    return new SwerveModule(
        moduleConfig.applyWithOffset(
            createSwerveModuleIO(0, 4, 20, moduleConfig),
            Rotation2d.fromRotations(0.231445).plus(ModulePosition.FRONT_LEFT.transform(position)),
            20),
        position.getPosition(wheelBase, trackWidth),
        position.getName());
  }

  public SwerveModule getModule6(ModulePosition position) {
    return new SwerveModule(
        moduleConfig.applyWithOffset(
            createSwerveModuleIO(1, 5, 21, moduleConfig),
            Rotation2d.fromRotations(-0.2253).plus(ModulePosition.FRONT_RIGHT.transform(position)),
            21),
        position.getPosition(wheelBase, trackWidth),
        position.getName());
  }

  public SwerveModule getModule7(ModulePosition position) {
    return new SwerveModule(
        moduleConfig.applyWithOffset(
            createSwerveModuleIO(2, 6, 22, moduleConfig),
            Rotation2d.fromRotations(0.071289).plus(ModulePosition.REAR_LEFT.transform(position)),
            22),
        position.getPosition(wheelBase, trackWidth),
        position.getName());
  }

  public SwerveModule getModule8(ModulePosition position) {
    return new SwerveModule(
        moduleConfig.applyWithOffset(
            createSwerveModuleIO(3, 7, 23, moduleConfig),
            Rotation2d.fromRotations(0.483887).plus(ModulePosition.REAR_RIGHT.transform(position)),
            23),
        position.getPosition(wheelBase, trackWidth),
        position.getName());
  }

  public SwerveModule getModule9(ModulePosition position) {
    return new SwerveModule(
        moduleConfig.applyWithOffset(
            createSwerveModuleIO(0, 4, 20, moduleConfig),
            Rotation2d.fromRotations(0.213623).plus(ModulePosition.FRONT_LEFT.transform(position)),
            20),
        position.getPosition(wheelBase, trackWidth),
        position.getName());
  }

  public SwerveModule getModule10(ModulePosition position) {
    return new SwerveModule(
        moduleConfig.applyWithOffset(
            createSwerveModuleIO(1, 5, 21, moduleConfig),
            Rotation2d.fromRotations(-0.077637)
                .plus(ModulePosition.FRONT_RIGHT.transform(position)),
            21),
        position.getPosition(wheelBase, trackWidth),
        position.getName());
  }

  public SwerveModule getModule11(ModulePosition position) {
    return new SwerveModule(
        moduleConfig.applyWithOffset(
            createSwerveModuleIO(2, 6, 22, moduleConfig),
            Rotation2d.fromRotations(0.136475).plus(ModulePosition.REAR_LEFT.transform(position)),
            22),
        position.getPosition(wheelBase, trackWidth),
        position.getName());
  }

  public SwerveModule getModule12(ModulePosition position) {
    return new SwerveModule(
        moduleConfig.applyWithOffset(
            createSwerveModuleIO(3, 7, 23, moduleConfig),
            Rotation2d.fromRotations(-0.309570).plus(ModulePosition.REAR_RIGHT.transform(position)),
            23),
        position.getPosition(wheelBase, trackWidth),
        position.getName());
  }

  public static SwerveModule getFakeModule() {
    return new SwerveModule(new SwerveModuleIO(), new Translation2d(), "Fake");
  }
}
