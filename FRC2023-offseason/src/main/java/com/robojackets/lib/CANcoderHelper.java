package com.robojackets.lib;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import java.util.function.Consumer;
import lombok.experimental.UtilityClass;

@UtilityClass
public class CANcoderHelper {
  public static StatusCode applyConfig(CANcoder encoder, Consumer<CANcoderConfiguration> change) {
    return applyConfig(encoder, change, false);
  }

  public static StatusCode applyConfig(
      CANcoder encoder, Consumer<CANcoderConfiguration> change, boolean refresh) {
    var config = new CANcoderConfiguration();

    if (refresh) {
      var status = encoder.getConfigurator().refresh(config);

      if (status != StatusCode.OK) {
        return status;
      }
    }

    change.accept(config);

    return encoder.getConfigurator().apply(config);
  }

  public static StatusCode updateMagnetSensorConfigs(
      CANcoder encoder, Consumer<MagnetSensorConfigs> change) {
    return updateMagnetSensorConfigs(encoder, change, false);
  }

  public static StatusCode updateMagnetSensorConfigs(
      CANcoder encoder, Consumer<MagnetSensorConfigs> change, boolean refresh) {
    var config = new MagnetSensorConfigs();

    if (refresh) {
      var status = encoder.getConfigurator().refresh(config);

      if (status != StatusCode.OK) {
        return status;
      }
    }

    change.accept(config);
    return encoder.getConfigurator().apply(config);
  }
}
