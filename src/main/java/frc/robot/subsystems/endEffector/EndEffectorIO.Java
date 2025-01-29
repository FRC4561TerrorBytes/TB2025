// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {

  @AutoLog
  public static class EndEffectorIOInputs {

    // pivot components
    public double endEffectorStatorCurrent = 0.0;
    public double endEffectorVoltage = 0.0;
    public double endEffectorMotorOneTemp = 0.0;
    public boolean endEffectorMotorOneConnected = false;
    public double endEffectorSpeed = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(EndEffectorIOInputs inputs) {}

  public default void setEndEffectorSpeed(double speed) {}

  public default void stopEndEffector() {}
}
