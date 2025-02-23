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
  public static final double PIVOT_GEAR_RATIO = 190;

  public static final int INTAKE_MOTOR_ID = 41;
  public static final int ALGAE_MANIPULATOR_ID = 42;
  public static final int INTAKE_CURRENT_LIMIT = 20;
  public static final int LEFT_PIVOT_ID_1 = 31;
  public static final int LEFT_PIVOT_ID_2 = 32;
  public static final int RIGHT_PIVOT_ID_1 = 33;
  public static final int RIGHT_PIVOT_ID_2 = 34;

  public static final int intakeMotorID = 33;
  public static final int algaeManipulatorMotorID = 61;

  public static final int EXTENSION_ID = 40;
  public static final double EXTENSION_GEAR_RATIO = 10;
}
