package com.robojackets.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NoteHandler {
  // private TalonFX noteHandler1 = new TalonFX(24, "*");
  private TalonFX noteHandler2 = new TalonFX(25, "*");

  private DutyCycleOut request = new DutyCycleOut(0);
  private TorqueCurrentFOC handlercurrent = new TorqueCurrentFOC(0);

  public NoteHandler() {
    var NoteHandlerTalonFXConfigs = new TalonFXConfiguration();
    // noteHandler1.getConfigurator().apply(NoteHandlerTalonFXConfigs);
    noteHandler2.getConfigurator().apply(NoteHandlerTalonFXConfigs);
  }

  public void setHandlerAmpCurrent(double current) {
    // noteHandler1.setControl(handlercurrent.withOutput(-current).withMaxAbsDutyCycle(0.4));
    noteHandler2.setControl(handlercurrent.withOutput(current).withMaxAbsDutyCycle(0.4));
  }

  public void setHandlerScore(double dutycycle) {
    // noteHandler1.setControl(request.withOutput(dutycycle));
    noteHandler2.setControl(request.withOutput(0.4));
  }

  public void setHandlerTrap(double dutycycle) {
    // noteHandler1.setControl(request.withOutput(dutycycle));
    noteHandler2.setControl(request.withOutput(-dutycycle));
  }

  public void setHandlerStop() {
    // noteHandler1.setControl(request.withOutput(0.0));
    noteHandler2.setControl(request.withOutput(0.0));
  }

  // public double getHandler1Current() {
  //   return noteHandler1.getTorqueCurrent().getValue();
  // }

  public void setHandlerDutyCycle(double dutycycle) {
    // noteHandler1.setControl(request.withOutput(dutycycle));
    noteHandler2.setControl(request.withOutput(-dutycycle));
  }

  public double getDutyCycle() {
    return noteHandler2.getDutyCycle().getValue();
  }

  public double getTorqueCurrent() {
    return noteHandler2.getTorqueCurrent().getValue();
  }

  public double getStatorCurrent() {
    return noteHandler2.getStatorCurrent().getValue();
  }

  public double getSupplyCurrent() {
    return noteHandler2.getSupplyCurrent().getValue();
  }

  public void SDOutputs() {
    SmartDashboard.putNumber("notehandler torque current", getTorqueCurrent());
    SmartDashboard.putNumber("notehandler supply current", getSupplyCurrent());
    SmartDashboard.putNumber("notehandler stator current", getStatorCurrent());
  }
}
