// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  @AutoLog
  public static class PivotIOInputs {
    public double pivotAngle = 0.0;

    // pivot components
    public double pivotStatorCurrent = 0.0;
    public double pivotSupplyCurrent = 0.0;
    public double pivotVoltage = 0.0;
    public double pivotSetpoint = 0.0;
    public double pivotMotorOneTemp = 0.0;
    public boolean pivotMotorOneConnected = false;
    public double pivotSpeed = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(PivotIOInputs inputs) {}

  public default void setPivotPosition(double angle) {}

  public default void setPivotVoltage(double voltage) {}
}
