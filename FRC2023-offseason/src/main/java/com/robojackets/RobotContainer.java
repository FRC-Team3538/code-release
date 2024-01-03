package com.robojackets;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.robojackets.generated.TunerConstants;
import com.robojackets.oi.CSSelector;
import com.robojackets.oi.ControlScheme;
import com.robojackets.subsystems.Arm;
import com.robojackets.subsystems.swerve.CTRESwerveDrive;

public class RobotContainer {

  public ControlScheme controls;

  public final CTRESwerveDrive swerve = TunerConstants.DriveTrain;

  public TalonFX intakemotor = new TalonFX(29);

  public Arm arm = new Arm();

  public RobotContainer() {
    // Factory default config
    intakemotor.getConfigurator().apply(new TalonFXConfiguration());

    SignalLogger.enableAutoLogging(true);
    SignalLogger.start();

    controls = CSSelector.findMainControls();
  }
}
