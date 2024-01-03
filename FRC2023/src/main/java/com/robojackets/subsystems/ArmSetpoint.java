package com.robojackets.subsystems;

import lombok.Getter;

@Getter
public enum ArmSetpoint {
  GROUND(0.86, -9.83),
  STOW(1.0, 90.0),
  HIGH_STOW(1.0, 132.0),
  MID(0, 231.0),
  SHELF(2.0, 180.0),
  HIGH_MILFORD(20.0, 228.0),
  HIGH_MUSKEGON(16.0, 210.0),
  BIGBOI_HIGH(16.6, 223.0),
  LOW_SCORE(2.5, 41.0);

  private double length;
  private double angle;

  private ArmSetpoint(double length, double angle) {
    this.length = length;
    this.angle = angle;
  }
}
