package com.robojackets;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenixpro.hardware.Pigeon2;
import com.robojackets.lib.UnitConversion;
import com.robojackets.oi.CSSelector;
import com.robojackets.oi.ControlScheme;
import com.robojackets.subsystems.Arm;
import com.robojackets.subsystems.gyro.GyroIO;
import com.robojackets.subsystems.gyro.Pigeon2GyroIO;
import com.robojackets.subsystems.gyro.PigeonGyroIO;
import com.robojackets.subsystems.swerve.ModuleContainer;
import com.robojackets.subsystems.swerve.ModuleProvider;
import com.robojackets.subsystems.swerve.ModuleProvider.ModulePosition;
import com.robojackets.subsystems.swerve.SwerveDrive;
import com.robojackets.subsystems.swerve.SwerveModuleConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class RobotContainer {

  public ControlScheme controls;

  // public final TankDrive tank;
  public final SwerveDrive swerve;

  public WPI_TalonFX intakemotor = new WPI_TalonFX(29);

  public Arm arm = new Arm();

  public RobotContainer() {
    // Factory default config
    // intakemotor.configFactoryDefault();

    switch (Constants.getDrivetrain()) {
      case SWERVE:
        SwerveModuleConfig config;
        ModuleProvider provider;
        switch (Constants.getRobot()) {
          case ROBOT_REAL:
            config =
                SwerveModuleConfig.builder()
                    .canBus("3538_Robot")
                    .driveFeedforward(new SimpleMotorFeedforward(0.22496, 2.5629, 0.43277))
                    .azimuthFeedforward(new SimpleMotorFeedforward(0.37043, 1.5376, 0.024777))
                    .driveVelocityPIDkP(0.1)
                    .azimuthPositionPIDkP(45.815)
                    .azimuthPositionPIDkD(2.1592)
                    .azimuthMotionMagicCruiseVelocity(60)
                    .azimuthMotionMagicAcceleration(1000)
                    .build(); // Constants for Drive Command stuff
            provider =
                ModuleProvider.builder()
                    .moduleConfig(config)
                    .wheelBase(19.5 * UnitConversion.METERS_PER_INCH)
                    .trackWidth(17.5 * UnitConversion.METERS_PER_INCH)
                    .build();
            if (Constants.getID().equals("ROBOT_A")) {
              swerve =
                  new SwerveDrive(
                      new ModuleContainer(
                          provider.getModule9(ModulePosition.FRONT_LEFT),
                          provider.getModule10(ModulePosition.FRONT_RIGHT),
                          provider.getModule11(ModulePosition.REAR_LEFT),
                          provider.getModule12(ModulePosition.REAR_RIGHT)),
                      new Pigeon2GyroIO(new Pigeon2(30, config.getCanBus())));
            } else {
              swerve =
                  new SwerveDrive(
                      new ModuleContainer(
                          provider.getModule5(ModulePosition.FRONT_LEFT),
                          provider.getModule6(ModulePosition.FRONT_RIGHT),
                          provider.getModule7(ModulePosition.REAR_LEFT),
                          provider.getModule8(ModulePosition.REAR_RIGHT)),
                      new Pigeon2GyroIO(new Pigeon2(30, config.getCanBus())));
            }

            break;
          case ROBOT_ROVER:
            config =
                SwerveModuleConfig.builder()
                    .canBus("swerve_rover_drive")
                    .driveFeedforward(new SimpleMotorFeedforward(0.22496, 2.5629, 0.43277))
                    .azimuthFeedforward(new SimpleMotorFeedforward(0.37043, 1.5376, 0.024777))
                    .driveVelocityPIDkP(0.1)
                    .azimuthPositionPIDkP(45.815)
                    .azimuthPositionPIDkD(2.1592)
                    .azimuthMotionMagicCruiseVelocity(60)
                    .azimuthMotionMagicAcceleration(1000)
                    .build();
            provider =
                ModuleProvider.builder()
                    .moduleConfig(config)
                    .wheelBase(20.5 * UnitConversion.METERS_PER_INCH)
                    .trackWidth(20.5 * UnitConversion.METERS_PER_INCH)
                    .build();
            swerve =
                new SwerveDrive(
                    new ModuleContainer(
                        provider.getModule1(ModulePosition.FRONT_LEFT),
                        provider.getModule2(ModulePosition.FRONT_RIGHT),
                        provider.getModule3(ModulePosition.REAR_LEFT),
                        provider.getModule4(ModulePosition.REAR_RIGHT)),
                    new PigeonGyroIO(new PigeonIMU(30)));
            break;
          case ROBOT_SIMBOT:
            config =
                SwerveModuleConfig.builder()
                    .canBus("3538_Robot")
                    .driveFeedforward(new SimpleMotorFeedforward(0, 2.5629, 0.43277))
                    .azimuthFeedforward(new SimpleMotorFeedforward(0, 1.5376, 0.024777))
                    .driveVelocityPIDkP(0.1)
                    .azimuthPositionPIDkP(45.815)
                    .azimuthPositionPIDkD(2.1592)
                    .azimuthMotionMagicCruiseVelocity(60)
                    .azimuthMotionMagicAcceleration(1000)
                    .build();
            provider =
                ModuleProvider.builder()
                    .moduleConfig(config)
                    .wheelBase(19.5 * UnitConversion.METERS_PER_INCH)
                    .trackWidth(17.5 * UnitConversion.METERS_PER_INCH)
                    .build();
            swerve =
                new SwerveDrive(
                    new ModuleContainer(
                        provider.getModule1(ModulePosition.FRONT_LEFT),
                        provider.getModule2(ModulePosition.FRONT_RIGHT),
                        provider.getModule3(ModulePosition.REAR_LEFT),
                        provider.getModule4(ModulePosition.REAR_RIGHT)),
                    new Pigeon2GyroIO(new Pigeon2(30, config.getCanBus())));
            break;
          default:
            swerve =
                new SwerveDrive(
                    new ModuleContainer(
                        ModuleProvider.getFakeModule(),
                        ModuleProvider.getFakeModule(),
                        ModuleProvider.getFakeModule(),
                        ModuleProvider.getFakeModule()),
                    new GyroIO());
        }

        // tank = new TankDrive(new TankDriveIO(new GearboxIO(), new GearboxIO(), new GyroIO()));
        break;
      case TANK:
        // tank = new TankDrive();
        swerve =
            new SwerveDrive(
                new ModuleContainer(
                    ModuleProvider.getFakeModule(),
                    ModuleProvider.getFakeModule(),
                    ModuleProvider.getFakeModule(),
                    ModuleProvider.getFakeModule()),
                new GyroIO());
        break;
      case NONE:
      default:
        // tank = new TankDrive(new TankDriveIO(new GearboxIO(), new GearboxIO(), new GyroIO()));
        swerve =
            new SwerveDrive(
                new ModuleContainer(
                    ModuleProvider.getFakeModule(),
                    ModuleProvider.getFakeModule(),
                    ModuleProvider.getFakeModule(),
                    ModuleProvider.getFakeModule()),
                new GyroIO());
    }

    controls = CSSelector.findMainControls();
    intakemotor.setNeutralMode(NeutralMode.Brake);
  }
}
