// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double pivotAngle = 0.0;
    public double extensionHeight = 0.0;

    // pivot components
    public double pivotStatorCurrent = 0.0;
    public double pivotSupplyCurrent = 0.0;
    public double pivotVoltage = 0.0;
    public double pivotSetpoint = 0.0;
    public double pivotMotorOneTemp = 0.0;
    public boolean pivotMotorOneConnected = false;
    public double pivotMotorTwoTemp = 0.0;
    public boolean pivotMotorTwoConnected = false;
    public double pivotMotorThreeTemp = 0.0;
    public boolean pivotMotorThreeConnected = false;
    public double pivotMotorFourTemp = 0.0;
    public boolean pivotMotorFourConnected = false;
    public double pivotSpeed = 0.0;

    // carriage components
    public double extensionStatorCurrent = 0.0;
    public double extensionSupplyCurrent = 0.0;
    public double extensionVoltage = 0.0;
    public double extensionSetpoint = 0.0;
    public double extensionMotorTemp = 0.0;
    public boolean extensionMotorConnected = false;
    public double extensionSpeed = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setTargetPosition(double extenstionPosition, double pivotPosition) {}

  public default void setExtensionPosition() {}

  public default void setPivotSetpoint() {}

  public default void setExtensionVoltage(double voltage) {}

  public default void setPivotVoltage(double voltage) {}
}
