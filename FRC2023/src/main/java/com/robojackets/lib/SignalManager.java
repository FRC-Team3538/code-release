package com.robojackets.lib;

import static com.ctre.phoenixpro.BaseStatusSignalValue.waitForAll;

import com.ctre.phoenixpro.BaseStatusSignalValue;
import java.util.ArrayList;

public class SignalManager {
  private static SignalManager instance = new SignalManager();

  private ArrayList<BaseStatusSignalValue> statusesList = new ArrayList<>();
  private BaseStatusSignalValue[] statuses;

  public static SignalManager getInstance() {
    return instance;
  }

  public void register(BaseStatusSignalValue... statuses) {
    for (var status : statuses) {
      statusesList.add(status);
    }
  }

  public void finalize() {
    statuses = new BaseStatusSignalValue[statusesList.size()];

    statusesList.toArray(statuses);

    statusesList.clear();
  }

  public void refresh() {
    waitForAll(0, statuses);
  }
}
