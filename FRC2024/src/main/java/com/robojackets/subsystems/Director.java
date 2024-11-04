package com.robojackets.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

public class Director {
  private TalonFX directorMotor = new TalonFX(16, "*");

  private DutyCycleOut request = new DutyCycleOut(0);
  private TorqueCurrentFOC directorcurrent = new TorqueCurrentFOC(0);

  public Director() {
    var DirectorTalonFXConfigs = new TalonFXConfiguration();
    directorMotor.getConfigurator().apply(DirectorTalonFXConfigs);
  }

  public void setDirectorCurrent(double current) {
    directorMotor.setControl(directorcurrent.withOutput(current));
  }

  public void setDirectortoAmp(double dutycycle) {
    directorMotor.setControl(request.withOutput(dutycycle));
  }

  public void setDirectortoShooter(double dutycycle) {
    directorMotor.setControl(request.withOutput(-dutycycle));
  }

  public void setDirectortoStop() {
    directorMotor.setControl(request.withOutput(0.0));
  }
}
