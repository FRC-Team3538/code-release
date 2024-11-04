package com.robojackets.lib;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.function.Consumer;
import lombok.experimental.UtilityClass;

@UtilityClass
public class FalconHelper {
  public static StatusCode applyConfig(TalonFX falcon, Consumer<TalonFXConfiguration> change) {
    return applyConfig(falcon, change, false);
  }

  public static StatusCode applyConfig(
      TalonFX falcon, Consumer<TalonFXConfiguration> change, boolean refresh) {
    var config = new TalonFXConfiguration();
    if (refresh) {
      var status = falcon.getConfigurator().refresh(config);

      if (status != StatusCode.OK) {
        return status;
      }
    }

    change.accept(config);

    return falcon.getConfigurator().apply(config);
  }

  public static StatusCode updateSlot0Configs(TalonFX falcon, Consumer<Slot0Configs> change) {
    return updateSlot0Configs(falcon, change, false);
  }

  public static StatusCode updateSlot0Configs(
      TalonFX falcon, Consumer<Slot0Configs> change, boolean refresh) {
    var config = new Slot0Configs();
    if (refresh) {
      var status = falcon.getConfigurator().refresh(config);

      if (status != StatusCode.OK) {
        return status;
      }
    }

    change.accept(config);
    return falcon.getConfigurator().apply(config);
  }

  public static StatusCode updateSlot1Configs(TalonFX falcon, Consumer<Slot1Configs> change) {
    return updateSlot1Configs(falcon, change, false);
  }

  public static StatusCode updateSlot1Configs(
      TalonFX falcon, Consumer<Slot1Configs> change, boolean refresh) {
    var config = new Slot1Configs();
    if (refresh) {
      var status = falcon.getConfigurator().refresh(config);

      if (status != StatusCode.OK) {
        return status;
      }
    }

    change.accept(config);
    return falcon.getConfigurator().apply(config);
  }

  public static StatusCode updateSlot2Configs(TalonFX falcon, Consumer<Slot2Configs> change) {
    return updateSlot2Configs(falcon, change, false);
  }

  public static StatusCode updateSlot2Configs(
      TalonFX falcon, Consumer<Slot2Configs> change, boolean refresh) {
    var config = new Slot2Configs();
    if (refresh) {
      var status = falcon.getConfigurator().refresh(config);

      if (status != StatusCode.OK) {
        return status;
      }
    }

    change.accept(config);
    return falcon.getConfigurator().apply(config);
  }

  public static StatusCode updateMotorOutputConfigs(
      TalonFX falcon, Consumer<MotorOutputConfigs> change) {
    return updateMotorOutputConfigs(falcon, change, false);
  }

  public static StatusCode updateMotorOutputConfigs(
      TalonFX falcon, Consumer<MotorOutputConfigs> change, boolean refresh) {
    var config = new MotorOutputConfigs();
    if (refresh) {
      var status = falcon.getConfigurator().refresh(config);

      if (status != StatusCode.OK) {
        return status;
      }
    }

    change.accept(config);
    return falcon.getConfigurator().apply(config);
  }

  public static StatusCode updateCurrentLimitsConfigs(
      TalonFX falcon, Consumer<CurrentLimitsConfigs> change) {
    return updateCurrentLimitsConfigs(falcon, change, false);
  }

  public static StatusCode updateCurrentLimitsConfigs(
      TalonFX falcon, Consumer<CurrentLimitsConfigs> change, boolean refresh) {
    var config = new CurrentLimitsConfigs();
    if (refresh) {
      var status = falcon.getConfigurator().refresh(config);

      if (status != StatusCode.OK) {
        return status;
      }
    }

    change.accept(config);
    return falcon.getConfigurator().apply(config);
  }

  public static StatusCode updateVoltageConfigs(TalonFX falcon, Consumer<VoltageConfigs> change) {
    return updateVoltageConfigs(falcon, change, false);
  }

  public static StatusCode updateVoltageConfigs(
      TalonFX falcon, Consumer<VoltageConfigs> change, boolean refresh) {
    var config = new VoltageConfigs();
    if (refresh) {
      var status = falcon.getConfigurator().refresh(config);

      if (status != StatusCode.OK) {
        return status;
      }
    }

    change.accept(config);
    return falcon.getConfigurator().apply(config);
  }

  public static StatusCode updateTorqueCurrentConfigs(
      TalonFX falcon, Consumer<TorqueCurrentConfigs> change) {
    return updateTorqueCurrentConfigs(falcon, change, false);
  }

  public static StatusCode updateTorqueCurrentConfigs(
      TalonFX falcon, Consumer<TorqueCurrentConfigs> change, boolean refresh) {
    var config = new TorqueCurrentConfigs();
    if (refresh) {
      var status = falcon.getConfigurator().refresh(config);

      if (status != StatusCode.OK) {
        return status;
      }
    }

    change.accept(config);
    return falcon.getConfigurator().apply(config);
  }

  public static StatusCode updateFeedbackConfigs(TalonFX falcon, Consumer<FeedbackConfigs> change) {
    return updateFeedbackConfigs(falcon, change, false);
  }

  public static StatusCode updateFeedbackConfigs(
      TalonFX falcon, Consumer<FeedbackConfigs> change, boolean refresh) {
    var config = new FeedbackConfigs();
    if (refresh) {
      var status = falcon.getConfigurator().refresh(config);

      if (status != StatusCode.OK) {
        return status;
      }
    }

    change.accept(config);
    return falcon.getConfigurator().apply(config);
  }

  public static StatusCode updateOpenLoopRampsConfigs(
      TalonFX falcon, Consumer<OpenLoopRampsConfigs> change) {
    return updateOpenLoopRampsConfigs(falcon, change, false);
  }

  public static StatusCode updateOpenLoopRampsConfigs(
      TalonFX falcon, Consumer<OpenLoopRampsConfigs> change, boolean refresh) {
    var config = new OpenLoopRampsConfigs();
    if (refresh) {
      var status = falcon.getConfigurator().refresh(config);

      if (status != StatusCode.OK) {
        return status;
      }
    }

    change.accept(config);
    return falcon.getConfigurator().apply(config);
  }

  public static StatusCode updateClosedLoopRampsConfigs(
      TalonFX falcon, Consumer<ClosedLoopRampsConfigs> change) {
    return updateClosedLoopRampsConfigs(falcon, change, false);
  }

  public static StatusCode updateClosedLoopRampsConfigs(
      TalonFX falcon, Consumer<ClosedLoopRampsConfigs> change, boolean refresh) {
    var config = new ClosedLoopRampsConfigs();
    if (refresh) {
      var status = falcon.getConfigurator().refresh(config);

      if (status != StatusCode.OK) {
        return status;
      }
    }

    change.accept(config);
    return falcon.getConfigurator().apply(config);
  }

  public static StatusCode updateHardwareLimitSwitchConfigs(
      TalonFX falcon, Consumer<HardwareLimitSwitchConfigs> change) {
    return updateHardwareLimitSwitchConfigs(falcon, change, false);
  }

  public static StatusCode updateHardwareLimitSwitchConfigs(
      TalonFX falcon, Consumer<HardwareLimitSwitchConfigs> change, boolean refresh) {
    var config = new HardwareLimitSwitchConfigs();
    if (refresh) {
      var status = falcon.getConfigurator().refresh(config);

      if (status != StatusCode.OK) {
        return status;
      }
    }

    change.accept(config);
    return falcon.getConfigurator().apply(config);
  }

  public static StatusCode updateAudioConfigs(TalonFX falcon, Consumer<AudioConfigs> change) {
    return updateAudioConfigs(falcon, change, false);
  }

  public static StatusCode updateAudioConfigs(
      TalonFX falcon, Consumer<AudioConfigs> change, boolean refresh) {
    var config = new AudioConfigs();
    if (refresh) {
      var status = falcon.getConfigurator().refresh(config);

      if (status != StatusCode.OK) {
        return status;
      }
    }

    change.accept(config);
    return falcon.getConfigurator().apply(config);
  }

  public static StatusCode updateSoftwareLimitSwitchConfigs(
      TalonFX falcon, Consumer<SoftwareLimitSwitchConfigs> change) {
    return updateSoftwareLimitSwitchConfigs(falcon, change, false);
  }

  public static StatusCode updateSoftwareLimitSwitchConfigs(
      TalonFX falcon, Consumer<SoftwareLimitSwitchConfigs> change, boolean refresh) {
    var config = new SoftwareLimitSwitchConfigs();
    if (refresh) {
      var status = falcon.getConfigurator().refresh(config);

      if (status != StatusCode.OK) {
        return status;
      }
    }

    change.accept(config);
    return falcon.getConfigurator().apply(config);
  }

  public static StatusCode updateMotionMagicConfigs(
      TalonFX falcon, Consumer<MotionMagicConfigs> change) {
    return updateMotionMagicConfigs(falcon, change, false);
  }

  public static StatusCode updateMotionMagicConfigs(
      TalonFX falcon, Consumer<MotionMagicConfigs> change, boolean refresh) {
    var config = new MotionMagicConfigs();
    if (refresh) {
      var status = falcon.getConfigurator().refresh(config);

      if (status != StatusCode.OK) {
        return status;
      }
    }

    change.accept(config);
    return falcon.getConfigurator().apply(config);
  }

  public static StatusCode updateCustomParamsConfigs(
      TalonFX falcon, Consumer<CustomParamsConfigs> change) {
    return updateCustomParamsConfigs(falcon, change, false);
  }

  public static StatusCode updateCustomParamsConfigs(
      TalonFX falcon, Consumer<CustomParamsConfigs> change, boolean refresh) {
    var config = new CustomParamsConfigs();
    if (refresh) {
      var status = falcon.getConfigurator().refresh(config);

      if (status != StatusCode.OK) {
        return status;
      }
    }

    change.accept(config);
    return falcon.getConfigurator().apply(config);
  }

  public static StatusCode updateClosedLoopGeneralConfigs(
      TalonFX falcon, Consumer<ClosedLoopGeneralConfigs> change) {
    return updateClosedLoopGeneralConfigs(falcon, change, false);
  }

  public static StatusCode updateClosedLoopGeneralConfigs(
      TalonFX falcon, Consumer<ClosedLoopGeneralConfigs> change, boolean refresh) {
    var config = new ClosedLoopGeneralConfigs();
    if (refresh) {
      var status = falcon.getConfigurator().refresh(config);

      if (status != StatusCode.OK) {
        return status;
      }
    }

    change.accept(config);
    return falcon.getConfigurator().apply(config);
  }
}
