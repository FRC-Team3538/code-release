package com.robojackets.lib;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import java.util.function.Consumer;

public class AllianceMonitor {
  private static Optional<Alliance> lastAlliance = Optional.empty();

  public static void IfAllianceSetThen(Consumer<Alliance> operator) {
    var alliance = DriverStation.getAlliance();

    if (lastAlliance.isEmpty() && alliance.isPresent()) {
      lastAlliance = alliance;
      operator.accept(alliance.get());
    }
  }

  public static void IfAllianceUpdatedThen(Consumer<Alliance> operator) {
    var alliance = DriverStation.getAlliance();

    if (lastAlliance.isPresent() && alliance.isPresent()) {
      var before = lastAlliance.get();
      var after = alliance.get();

      if (!before.equals(after)) {
        lastAlliance = alliance;
        operator.accept(alliance.get());
      }
    }
  }
}
