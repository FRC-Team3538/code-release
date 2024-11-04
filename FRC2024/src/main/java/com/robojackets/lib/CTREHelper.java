package com.robojackets.lib;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;
import lombok.experimental.UtilityClass;

@UtilityClass
public class CTREHelper {
  public static StatusCode checkStatus(StatusCode code, String action) {
    if (code != StatusCode.OK) {
      DriverStation.reportError(String.format("Error %s: %s", action, code), false);
    }
    return code;
  }
}
