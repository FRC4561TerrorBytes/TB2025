// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

/** Add your docs here. */
public class ClimberIOReal implements ClimberIO {
  private TalonFX climberMotor = new TalonFX(Constants.CLIMBER_ID);

  private final StatusSignal<Angle> climberAngle;
  private final StatusSignal<Current> climberStatorCurrent;
  private final StatusSignal<Current> climberSupplyCurrent;
  private final StatusSignal<AngularVelocity> climberSpeed;
  private final StatusSignal<Voltage> climberVoltage;
  private final StatusSignal<Temperature> climberTemp;

  private double climberSetpoint = 0.0;
  private double climberFeedForward = 0.0;

  private final MotionMagicVoltage m_request_climber = new MotionMagicVoltage(0);

  public ClimberIOReal() {
    var climberPIDConfig = new Slot0Configs();
    climberPIDConfig.GravityType = GravityTypeValue.Arm_Cosine;
    // pivotPIDConfig.kS = 0.28;
    climberPIDConfig.kV = 0;
    climberPIDConfig.kA = 0;
    climberPIDConfig.kP = 25;
    climberPIDConfig.kI = 0;
    climberPIDConfig.kD = 0;

    var climberConfig = new TalonFXConfiguration();
    climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberConfig.Slot0 = climberPIDConfig;
    climberConfig.Feedback.SensorToMechanismRatio = Constants.CLIMBER_GEAR_RATIO;
    climberConfig.MotionMagic.MotionMagicCruiseVelocity = 3;
    climberConfig.MotionMagic.MotionMagicAcceleration =
        climberConfig.MotionMagic.MotionMagicCruiseVelocity / 0.050;
    climberConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * Constants.CLIMBER_GEAR_RATIO;
    climberConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    climberConfig.ClosedLoopGeneral.ContinuousWrap = false;
    climberConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    climberConfig.CurrentLimits.StatorCurrentLimit = Constants.CLIMBER_STATOR_CURRENT_LIMIT;
    climberConfig.CurrentLimits.SupplyCurrentLimit = Constants.CLIMBER_SUPPLY_CURRENT_LIMIT;
    climberConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    tryUntilOk(5, () -> climberMotor.getConfigurator().apply(climberConfig, 0.25));

    climberMotor.setPosition(0);

    climberAngle = climberMotor.getPosition();
    climberStatorCurrent = climberMotor.getStatorCurrent();
    climberSupplyCurrent = climberMotor.getSupplyCurrent();
    climberSpeed = climberMotor.getVelocity();
    climberVoltage = climberMotor.getMotorVoltage();
    climberTemp = climberMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        climberAngle,
        climberStatorCurrent,
        climberSupplyCurrent,
        climberSpeed,
        climberVoltage,
        climberTemp);

    ParentDevice.optimizeBusUtilizationForAll(climberMotor);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    var climberStatus =
        BaseStatusSignal.refreshAll(
            climberAngle,
            climberStatorCurrent,
            climberSupplyCurrent,
            climberSpeed,
            climberVoltage,
            climberTemp);

    inputs.climberAngle = climberAngle.getValueAsDouble();

    inputs.climberMotorConnected = climberStatus.isOK();
    inputs.climberStatorCurrent = climberStatorCurrent.getValueAsDouble();
    inputs.climberSupplyCurrent = climberSupplyCurrent.getValueAsDouble();
    inputs.climberSpeed = climberMotor.getVelocity().getValueAsDouble();
    inputs.climberVoltage = climberMotor.getMotorVoltage().getValueAsDouble();
    inputs.climberSetpoint = climberSetpoint;
  }

  @Override
  public void setSetpoint(double position) {
    climberSetpoint = position;
    climberMotor.setControl(m_request_climber.withPosition(climberSetpoint));
  }

  @Override
  public void setclimberVoltage(double voltage) {
    climberMotor.setVoltage(voltage);
  }
}
