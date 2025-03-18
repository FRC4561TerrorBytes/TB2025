// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {

    public double climberAngle = 0.0;
    public double climberStatorCurrent = 0.0;
    public double climberSupplyCurrent = 0.0;
    public double climberVoltage = 0.0;
    public double climberSetpoint = 0.0;
    public double climberMotorTemp = 0.0;
    public boolean climberMotorConnected = false;
    public double climberSpeed = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setSetpoint(double position) {}

  public default void setClimberOutput(double voltage) {}
}
