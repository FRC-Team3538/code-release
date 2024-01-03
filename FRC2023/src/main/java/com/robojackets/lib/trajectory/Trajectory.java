package com.robojackets.lib.trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import lombok.AllArgsConstructor;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;

@AllArgsConstructor
public class Trajectory {
  private final List<TrajectoryState> states;

  public Trajectory() {
    states = List.of();
  }

  private TrajectoryState sampleInternal(double timestamp) {
    if (timestamp < states.get(0).getTimestamp()) {
      return states.get(0);
    }
    if (timestamp > getTotalTime()) {
      return states.get(states.size() - 1);
    }

    int low = 0;
    int high = states.size() - 1;

    while (low != high) {
      int mid = (low + high) / 2;
      if (states.get(mid).getTimestamp() < timestamp) {
        low = mid + 1;
      } else {
        high = mid;
      }
    }

    if (low == 0) {
      return states.get(low);
    }

    var behindState = states.get(low - 1);
    var aheadState = states.get(low);

    if ((aheadState.getTimestamp() - behindState.getTimestamp()) < 1e-6) {
      return aheadState;
    }

    return behindState.interpolate(aheadState, timestamp);
  }

  public TrajectoryState sample(double timestamp) {
    return sample(timestamp, false);
  }

  public TrajectoryState sample(double timestamp, boolean mirrorForRedAlliance) {
    var state = sampleInternal(timestamp);
    if (mirrorForRedAlliance) {
      return AllianceFlipUtil.apply(state);
    }
    return state;
  }

  public Pose2d getInitialPose() {
    return states.get(0).getPose();
  }

  public Pose2d getFinalPose() {
    return states.get(states.size() - 1).getPose();
  }

  public double getTotalTime() {
    return states.get(states.size() - 1).getTimestamp();
  }
}
