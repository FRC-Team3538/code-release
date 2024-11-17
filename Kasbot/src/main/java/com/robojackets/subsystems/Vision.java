package com.robojackets.subsystems;

import com.robojackets.lib.RJLog;
import com.robojackets.subsystems.vision.VisionModule;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private final Drive drive;
  private final VisionModule visionModule;

  private boolean enabled = true;

  public Vision(Drive drive, VisionModule visionModule) {
    this.drive = drive;
    this.visionModule = visionModule;

    setDefaultCommand(enableCommand());
  }

  public Command enableCommand() {
    return run(this::enable).withName("Vision::Enabled");
  }

  public Command disableCommand() {
    return run(this::disable).withName("Vision::Disabled");
  }

  public void enable() {
    this.enabled = true;
  }

  public void disable() {
    this.enabled = false;
  }

  public void enable(boolean enabled) {
    this.enabled = enabled;
  }

  public boolean enabled() {
    return enabled;
  }

  @Override
  public void periodic() {
    RJLog.log("Vision/enabled", enabled);

    if (!enabled) return;

    var maybeEstimate = visionModule.getNewPoseEstimate(drive.getPose());

    if (maybeEstimate.isEmpty()) {
      RJLog.log("Vision/apply", false);
      return;
    }

    var estimate = maybeEstimate.get();
    RJLog.log("Vision", estimate);

    if (!drive.isWithinBuffer(estimate.getTimestamp())) {
      RJLog.log("Vision/apply", false);
      return;
    }

    if (DriverStation.isEnabled()) {
      if (Math.abs(estimate.getEstimate().getZ()) > 0.1) {
        // not on the ground; that ain't us
        RJLog.log("Vision/apply", false);
        return;
      }

      var stdevs = estimate.getStdDevs();
      if (Math.sqrt(stdevs.get(0) * stdevs.get(0) + stdevs.get(1) * stdevs.get(1)) > 2) {
        RJLog.log("Vision/apply", false);
        return;
      }

      RJLog.log("Vision/apply", true);

      drive.addVisionEstimate(estimate);
    } else {
      RJLog.log("Vision/apply", true);

      drive.addVisionEstimate(estimate);
    }
  }
}
