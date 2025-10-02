// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final int PIVOT_MOTOR_ID = 13;
  public static final int PIVOT_CANCODER_ID = 30;
  public static final double PIVOT_GEAR_RATIO = 216;
  // MOI = 11.2kg * (0.37m^2)
  public static final double PIVOT_MOI = 11.2 * Math.pow(0.37, 2);

  public static final double MOTOR_MOI = 0.5 * 0.27 * Math.pow(0.05, 2);

  public static final int INTAKE_MOTOR_ID = 41;
  public static final int ALGAE_MANIPULATOR_ID = 42;
  public static final int CANRANGE_ID = 21;
  public static final int INTAKE_CURRENT_LIMIT = 35;
  public static final int LEFT_PIVOT_ID_1 = 31;
  public static final int LEFT_PIVOT_ID_2 = 32;
  public static final int RIGHT_PIVOT_ID_1 = 33;
  public static final int RIGHT_PIVOT_ID_2 = 34;
  public static final int PIVOT_STATOR_CURRENT_LIMIT = 60;
  public static final int PIVOT_SUPPLY_CURRENT_LIMIT = 50;
  public static final int EXTENSION_STATOR_CURRENT_LIMIT = 30;
  public static final int EXTENSION_SUPPLY_CURRENT_LIMIT = 20;
  public static final int CLIMBER_ID = 43;
  public static final int CLIMBER_STATOR_CURRENT_LIMIT = 30;
  public static final int CLIMBER_SUPPLY_CURRENT_LIMIT = 25;

  public static final int WRIST_ID = 51;
  public static final int WRIST_CANCODER_ID = 50;
  public static final double WRIST_GEAR_RATIO = 110.2;
  public static final int WRIST_STATOR_CURRENT_LIMIT = 30;
  public static final int WRIST_SUPPLY_CURRENT_LIMIT = 25;

  public static final int ALGAE_MANIPULATOR_MOTOR_ID = 61;

  public static final double SCORING_POSITION_OFFSET = 6.47; // in inches

  public static final int EXTENSION_ID = 40;
  // Gears are 12:44:60 tooth gears in a line
  // Determine Gear Ratio 12/44 * 44/60
  // Sprockets are 1:1 22 teeth
  // 22 teeth = 140mm
  // 140 * 0.2 = linear movement per rotation
  // 1000(aka 1 meter)/linear movement
  public static final double EXTENSION_GEAR_RATIO = 35.7909806729;
  public static final double CLIMBER_GEAR_RATIO = 200;
}
