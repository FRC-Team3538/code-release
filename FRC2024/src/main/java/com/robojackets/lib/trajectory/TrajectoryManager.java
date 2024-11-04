package com.robojackets.lib.trajectory;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import org.apache.commons.io.FileUtils;

public class TrajectoryManager {
  private List<String> trajectories = new ArrayList<>();

  private static TrajectoryManager instance;

  public static TrajectoryManager getInstance() {
    if (instance == null) {
      instance = new TrajectoryManager();
    }

    return instance;
  }

  public void LoadTrajectories() {
    var traj_dir = new File(Filesystem.getDeployDirectory(), "choreo");
    if (traj_dir.exists()) {
      FileUtils.iterateFiles(traj_dir, new String[] {"traj"}, false)
          .forEachRemaining(
              file -> {
                var len = file.getName().length();
                trajectories.add(file.getName().substring(0, len - 5));
              });
    }
  }

  public List<String> getTrajectories() {
    return trajectories;
  }
}
