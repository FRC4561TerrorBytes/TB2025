// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ElevatorIOReal implements ElevatorIO {
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
  private final StatusSignal<Temperature> pivotOneTemp;
  private final StatusSignal<Temperature> pivotTwoTemp;
  private final StatusSignal<Temperature> pivotThreeTemp;
  private final StatusSignal<Temperature> pivotFourTemp;

  private final StatusSignal<Angle> extensionAngle;
  private final StatusSignal<Current> extensionStatorCurrent;
  private final StatusSignal<Current> extensionSupplyCurrent;
  private final StatusSignal<AngularVelocity> extensionSpeed;
  private final StatusSignal<Voltage> extensionVoltage;
  private final StatusSignal<Temperature> extensionTemp;

  private double pivotSetpoint = 0.0;
  private double extensionSetpoint = 0.0;

  private double pivotFeedForward = 0.0;
  private double extensionFeedForward = 0.0;

  private final MotionMagicVoltage m_request_pivot = new MotionMagicVoltage(0);
  private final MotionMagicVoltage m_request_extension = new MotionMagicVoltage(0);

  /** Creates a new ElevatorIOReal. */
  public ElevatorIOReal() {

    var pivotPIDConfig = new Slot0Configs();
    var extensionPIDConfig = new Slot0Configs();
    // ADD PID Configs

    var pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.Slot0 = pivotPIDConfig; //
    // turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
    pivotConfig.Feedback.RotorToSensorRatio = Constants.PIVOT_GEAR_RATIO;
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / Constants.PIVOT_GEAR_RATIO;
    pivotConfig.MotionMagic.MotionMagicAcceleration =
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    pivotConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * Constants.PIVOT_GEAR_RATIO;
    pivotConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    pivotConfig.ClosedLoopGeneral.ContinuousWrap = false;
    pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    tryUntilOk(5, () -> pivotMotorOne.getConfigurator().apply(pivotConfig, 0.25));
    tryUntilOk(5, () -> pivotMotorTwo.getConfigurator().apply(pivotConfig, 0.25));
    tryUntilOk(5, () -> pivotMotorThree.getConfigurator().apply(pivotConfig, 0.25));
    tryUntilOk(5, () -> pivotMotorFour.getConfigurator().apply(pivotConfig, 0.25));

    var extensionConfig = new TalonFXConfiguration();
    extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    extensionConfig.Slot0 = extensionPIDConfig; //
    // turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
    extensionConfig.Feedback.RotorToSensorRatio = Constants.EXTENSION_GEAR_RATIO;
    extensionConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / Constants.EXTENSION_GEAR_RATIO;
    extensionConfig.MotionMagic.MotionMagicAcceleration =
        extensionConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    extensionConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * Constants.EXTENSION_GEAR_RATIO;
    extensionConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    extensionConfig.ClosedLoopGeneral.ContinuousWrap = false;
    extensionConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    tryUntilOk(5, () -> extensionMotor.getConfigurator().apply(extensionConfig, 0.25));

    pivotAngle = pivotMotorOne.getPosition();
    pivotStatorCurrent = pivotMotorOne.getStatorCurrent();
    pivotSupplyCurrent = pivotMotorOne.getSupplyCurrent();
    pivotSpeed = pivotMotorOne.getVelocity();
    pivotVoltage = pivotMotorOne.getMotorVoltage();
    pivotOneTemp = pivotMotorOne.getDeviceTemp();
    pivotTwoTemp = pivotMotorTwo.getDeviceTemp();
    pivotThreeTemp = pivotMotorThree.getDeviceTemp();
    pivotFourTemp = pivotMotorFour.getDeviceTemp();

    extensionAngle = extensionMotor.getPosition();
    extensionStatorCurrent = extensionMotor.getStatorCurrent();
    extensionSupplyCurrent = extensionMotor.getSupplyCurrent();
    extensionSpeed = extensionMotor.getVelocity();
    extensionVoltage = extensionMotor.getMotorVoltage();
    extensionTemp = extensionMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        pivotAngle,
        pivotStatorCurrent,
        pivotSupplyCurrent,
        pivotSpeed,
        pivotVoltage,
        pivotOneTemp,
        pivotTwoTemp,
        pivotThreeTemp,
        pivotFourTemp);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        extensionAngle,
        extensionStatorCurrent,
        extensionSupplyCurrent,
        extensionSpeed,
        extensionVoltage,
        extensionTemp);
    ParentDevice.optimizeBusUtilizationForAll(
        pivotMotorOne, pivotMotorTwo, pivotMotorThree, pivotMotorFour, extensionMotor);
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    var pivotOneStatus =
        BaseStatusSignal.refreshAll(
            pivotAngle,
            pivotStatorCurrent,
            pivotSupplyCurrent,
            pivotSpeed,
            pivotVoltage,
            pivotOneTemp);
    var pivotTwoStatus = BaseStatusSignal.refreshAll(pivotTwoTemp);
    var pivotThreeStatus = BaseStatusSignal.refreshAll(pivotThreeTemp);
    var pivotFourStatus = BaseStatusSignal.refreshAll(pivotFourTemp);

    var extensionStatus =
        BaseStatusSignal.refreshAll(
            extensionAngle,
            extensionStatorCurrent,
            extensionSupplyCurrent,
            extensionSpeed,
            extensionVoltage,
            extensionTemp);

    inputs.pivotAngle = pivotAngle.getValueAsDouble();

    inputs.pivotMotorOneConnected = pivotOneStatus.isOK();
    inputs.pivotMotorOneTemp = pivotOneTemp.getValueAsDouble();

    inputs.pivotMotorTwoConnected = pivotTwoStatus.isOK();
    inputs.pivotMotorTwoTemp = pivotTwoTemp.getValueAsDouble();

    inputs.pivotMotorThreeConnected = pivotThreeStatus.isOK();
    inputs.pivotMotorThreeTemp = pivotThreeTemp.getValueAsDouble();

    inputs.pivotMotorFourConnected = pivotFourStatus.isOK();
    inputs.pivotMotorFourTemp = pivotFourTemp.getValueAsDouble();

    inputs.extensionHeight = extensionAngle.getValueAsDouble();
    inputs.extensionMotorConnected = extensionStatus.isOK();

    inputs.pivotStatorCurrent = pivotStatorCurrent.getValueAsDouble();
    inputs.pivotSupplyCurrent = pivotSupplyCurrent.getValueAsDouble();
    inputs.pivotSpeed = pivotMotorOne.getVelocity().getValueAsDouble();
    inputs.pivotVoltage = pivotMotorOne.getMotorVoltage().getValueAsDouble();
    inputs.pivotSetpoint = this.pivotSetpoint;

    inputs.extensionStatorCurrent = extensionMotor.getStatorCurrent().getValueAsDouble();
    inputs.extensionSupplyCurrent = extensionMotor.getSupplyCurrent().getValueAsDouble();
    inputs.extensionVoltage = extensionMotor.getMotorVoltage().getValueAsDouble();
    inputs.extensionSetpoint = this.extensionSetpoint;
    inputs.extensionSpeed = extensionMotor.getVelocity().getValueAsDouble();

    m_request_extension.FeedForward = Math.sin(Units.degreesToRadians(inputs.pivotAngle)) * extensionFeedForward;
    m_request_pivot.FeedForward = Math.cos(Units.degreesToRadians(inputs.pivotAngle)) * pivotFeedForward + inputs.extensionHeight*1; // change 1 to be whatever scalar we find
  }

  @Override
  public void setExtensionPosition(double position) {
    extensionSetpoint = position;
    extensionMotor.setControl(m_request_extension.withPosition(extensionSetpoint).withFeedForward(0));
  }

  @Override
  public void setPivotSetpoint(double position) {
    pivotSetpoint = position;
    pivotMotorOne.setControl(m_request_pivot.withPosition(pivotSetpoint).withFeedForward(0));
  }

  @Override
  public void setExtensionVoltage(double voltage) {
    extensionMotor.setVoltage(voltage);
  }

  @Override
  public void setPivotVoltage(double voltage) {
    pivotMotorOne.setVoltage(voltage);
  }
}
