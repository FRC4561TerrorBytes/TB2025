// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorIOReal extends SubsystemBase {
  private TalonFX pivotMotorOne = new TalonFX(Constants.PIVOT_MOTOR_ONE_ID);
  private TalonFX pivotMotorTwo = new TalonFX(Constants.PIVOT_MOTOR_TWO_ID);
  private TalonFX pivotMotorThree = new TalonFX(Constants.PIVOT_MOTOR_THREE_ID);
  private TalonFX pivotMotorFour = new TalonFX(Constants.PIVOT_MOTOR_FOUR_ID);

  private TalonFX extensionMotor = new TalonFX(Constants.EXTENSION_MOTOR_ID);
  
  private final StatusSignal<Angle> pivotAngle;
  private final StatusSignal<Current> pivotStatorCurrent;
  private final StatusSignal<Current> pivotSupplyCurrent;
  private final StatusSignal<AngularVelocity> pivotSpeed;
  private final StatusSignal<Voltage> pivotVoltage;
  private final StatusSignal<Temperature> pivotTemp;

  private final StatusSignal<Angle> extensionAngle;
  private final StatusSignal<Current> extensionStatorCurrent;
  private final StatusSignal<Current> extensionSupplyCurrent;
  private final StatusSignal<AngularVelocity>  extensionSpeed;
  private final StatusSignal<Voltage> extensionVoltage;
  private final StatusSignal<Temperature> extensionTemp;

  
  /** Creates a new ElevatorIOReal. */
  public ElevatorIOReal() {

    var pivotPIDConfig = new Slot0Configs();
    var extensionPIDConfig = new Slot0Configs();
    // ADD PID Configs


    var pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.Slot0 = pivotPIDConfig;//
    // turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
    pivotConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
    pivotConfig.MotionMagic.MotionMagicAcceleration =
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    pivotConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
    pivotConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    pivotConfig.ClosedLoopGeneral.ContinuousWrap = false;
    pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    tryUntilOk(5, () -> pivotMotorOne.getConfigurator().apply(pivotConfig, 0.25));

    var extensionConfig = new TalonFXConfiguration();
    extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    extensionConfig.Slot0 = pivotPIDConfig;//
    // turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
    extensionConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
    extensionConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
    extensionConfig.MotionMagic.MotionMagicAcceleration =
    extensionConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    extensionConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
    extensionConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    extensionConfig.ClosedLoopGeneral.ContinuousWrap = false;
    extensionConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    tryUntilOk(5, () -> extensionMotor.getConfigurator().apply(extensionConfig, 0.25));

    pivotAngle = pivotMotorOne.getPosition();
    pivotStatorCurrent = pivotMotorOne.getStatorCurrent();
    pivotSupplyCurrent = pivotMotorOne.getSupplyCurrent();
    pivotSpeed = pivotMotorOne.getVelocity();
    pivotVoltage = pivotMotorOne.getMotorVoltage();
    pivotTemp = pivotMotorOne.getDeviceTemp();

    extensionAngle = extensionMotor.getPosition();
    extensionStatorCurrent = extensionMotor.getStatorCurrent();
    extensionSupplyCurrent = extensionMotor.getSupplyCurrent();
    extensionSpeed = extensionMotor.getVelocity();
    extensionVoltage = extensionMotor.getMotorVoltage();
    extensionTemp = extensionMotor.getDeviceTemp();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
