// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import frc.robot.RobotContainer.ElevatorPosition;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double pivotAngle = 0.0;
    public double extensionHeight = 0.0;
    // pivot components
    public double pivotOneStatorCurrent = 0.0;
    public double pivotOneSupplyCurrent = 0.0;
    public double pivotOneVoltage = 0.0;
    public double pivotSetpoint = 0.0;
    public double pivotOneSpeed = 0.0;
    public double pivotMotorOneTemp = 0.0;
    public boolean pivotMotorOneConnected = false;
    public double pivotMotorTwoTemp = 0.0;
    public boolean pivotMotorTwoConnected = false;
    public double pivotTwoStatorCurrent = 0.0;
    public double pivotTwoSupplyCurrent = 0.0;
    public double pivotTwoVoltage = 0.0;
    public double pivotTwoSpeed = 0.0;
    public double pivotMotorThreeTemp = 0.0;
    public boolean pivotMotorThreeConnected = false;
    public double pivotThreeStatorCurrent = 0.0;
    public double pivotThreeSupplyCurrent = 0.0;
    public double pivotThreeVoltage = 0.0;
    public double pivotThreeSpeed = 0.0;
    public double pivotMotorFourTemp = 0.0;
    public boolean pivotMotorFourConnected = false;
    public double pivotFourStatorCurrent = 0.0;
    public double pivotFourSupplyCurrent = 0.0;
    public double pivotFourVoltage = 0.0;
    public double pivotFourSpeed = 0.0;

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

  public default void setExtensionPosition(double position) {}

  public default void setTargetPosition(ElevatorPosition position) {}

  public default void setPivotPosition(double position) {}

  public default void setExtensionVoltage(double voltage) {}

  public default void setPivotVoltage(double voltage) {}
}
