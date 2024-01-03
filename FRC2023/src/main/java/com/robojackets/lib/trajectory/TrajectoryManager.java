package com.robojackets.lib.trajectory;

import com.google.gson.Gson;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.apache.commons.io.FileUtils;

public class TrajectoryManager {
  private List<String> trajectories = new ArrayList<>();
  private List<String> pathplannerTrajectories = new ArrayList<>();

  private final Gson gson = new Gson();

  private static TrajectoryManager instance;

  public static TrajectoryManager getInstance() {
    if (instance == null) {
      instance = new TrajectoryManager();
    }

    return instance;
  }

  public Trajectory getTrajectory(String trajName) {
    var traj_dir = new File(Filesystem.getDeployDirectory(), "trajectories");
    var traj_file = new File(traj_dir, trajName);

    return loadFile(traj_file);
  }

  public void LoadTrajectories() {
    var traj_dir = new File(Filesystem.getDeployDirectory(), "trajectories");
    if (traj_dir.exists()) {
      FileUtils.iterateFiles(traj_dir, new String[] {"json"}, false)
          .forEachRemaining(file -> trajectories.add(file.getName()));
    }
  }

  public void LoadPathPlannerTrajectories() {
    var traj_dir = new File(Filesystem.getDeployDirectory(), "pathplanner");
    if (traj_dir.exists()) {
      FileUtils.iterateFiles(traj_dir, new String[] {"path"}, false)
          .forEachRemaining(
              file -> {
                pathplannerTrajectories.add(
                    file.getName().substring(0, file.getName().length() - 5));
              });
    }
  }

  private Trajectory loadFile(File path) {
    try {
      var reader = new BufferedReader(new FileReader(path));
      var states = gson.fromJson(reader, TrajectoryState[].class);

      return new Trajectory(Arrays.asList(states));
    } catch (Exception ex) {
      DriverStation.reportError(ex.getMessage(), ex.getStackTrace());
    }
    return null;
  }

  public List<String> getTrajectories() {
    return trajectories;
  }

  public List<String> getPPTrajectories() {
    return pathplannerTrajectories;
  }
}
