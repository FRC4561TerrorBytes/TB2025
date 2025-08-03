// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface WristIO {

  @AutoLog
  public static class WristIOInputs {

    public double wristAngle = 0.0;
    public double wristStatorCurrent = 0.0;
    public double wristSupplyCurrent = 0.0;
    public double wristVoltage = 0.0;
    public double wristSetpoint = 0.0;
    public double wristMotorTemp = 0.0;
    public boolean wristMotorConnected = false;
    public boolean wristEncoderConnected = false;
    public double wristSpeed = 0.0;
  }

  public default void updateInputs(WristIOInputs inputs) {}

  public default void setSetpoint(double position) {}

  public default void setOutput(double voltage) {}
}
