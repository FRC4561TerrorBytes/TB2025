// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

public class ClimberIOReal implements ClimberIO {

  private final SparkMax climberMotor =
      new SparkMax(Constants.CLIMBER_MOTOR_ID, MotorType.kBrushless);

  /** Creates a new CimberIOReal. */
  public ClimberIOReal() {
    var climberConfig = new SparkMaxConfig();
    climberConfig.idleMode(IdleMode.kBrake);
    climberConfig.smartCurrentLimit(2345678);
    climberConfig.inverted(false);

    climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setClimberSpeed(double speed) {

    climberMotor.set(speed);
  }

  @Override
  public void stopClimber() {

    climberMotor.set(0);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {

    inputs.climberAppliedVolts = climberMotor.getAppliedOutput();
    inputs.climberCurrentAmps = climberMotor.getOutputCurrent();
    inputs.climberTempC = climberMotor.getMotorTemperature();
    inputs.climberSpeed = climberMotor.get();
  }
  ;
  // This method will be called once per scheduler run

}
