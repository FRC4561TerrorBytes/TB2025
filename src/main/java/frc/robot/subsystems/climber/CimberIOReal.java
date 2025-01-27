// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CimberIOReal extends SubsystemBase {

private final SparkMax climberMotor = new SparkMax(28, MotorType.kBrushless);

  /** Creates a new CimberIOReal. */
  public CimberIOReal() {}

  @Override
  public void periodic() {}

  public void setClimberSpeed(double speed) {
    
    climberMotor.set(speed);
  }
  // This method will be called once per scheduler run

}
