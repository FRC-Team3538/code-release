package com.robojackets.lib;

import static com.ctre.phoenix6.BaseStatusSignal.waitForAll;

import com.ctre.phoenix6.BaseStatusSignal;
import java.util.ArrayList;

public class SignalManager {
  private static SignalManager instance = new SignalManager();

  private ArrayList<BaseStatusSignal> statusesList = new ArrayList<>();
  private BaseStatusSignal[] statuses;

  public static SignalManager getInstance() {
    return instance;
  }

  public void register(BaseStatusSignal... statuses) {
    for (var status : statuses) {
      statusesList.add(status);
    }
  }

  public void finish() {
    statuses = new BaseStatusSignal[statusesList.size()];

    statusesList.toArray(statuses);

    statusesList.clear();
  }

  public void refresh() {
    waitForAll(0, statuses);
  }
}
