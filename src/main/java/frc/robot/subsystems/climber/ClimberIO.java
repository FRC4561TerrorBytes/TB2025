// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  /** Creates a new ClimberIO. */
  @AutoLog
  public static class ClimberIOInputs {
    public double climberAppliedVolts = 0.0;
    public double climberCurrentAmps = 0.0;
    public double climberPosition = 0.0;
    public boolean climberLimitSwitch = false;
    public double climberTempC = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}
  ;

  public default void setClimberSpeed(double speed) {}
  ;

  public default void stopClimber() {}
  ;
}
