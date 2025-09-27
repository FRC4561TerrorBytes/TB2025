// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;

/** Add your docs here. */
public class WristIOReal implements WristIO {
  private TalonFX WristMotor = new TalonFX(Constants.WRIST_ID);
  private CANcoder wristEncoder = new CANcoder(Constants.WRIST_CANCODER_ID);

  private final StatusSignal<Angle> WristAngle;
  private final StatusSignal<Current> WristStatorCurrent;
  private final StatusSignal<Current> WristSupplyCurrent;
  private final StatusSignal<AngularVelocity> WristSpeed;
  private final StatusSignal<Voltage> WristVoltage;
  private final StatusSignal<Temperature> WristTemp;

  private double WristSetpoint = 0.0;
  private double WristFeedForward = 0.0;

  private final MotionMagicVoltage m_request_Wrist = new MotionMagicVoltage(0);

  private final Alert wristAlert = new Alert("Wrist Disconnected.", AlertType.kWarning);
  private final Alert wristEncoderAlert =
      new Alert("Wrist Encoder Disconnected.", AlertType.kWarning);

  public WristIOReal() {
    var WristPIDConfig = new Slot0Configs();
    WristPIDConfig.GravityType = GravityTypeValue.Arm_Cosine;
    // pivotPIDConfig.kS = 0.28;
    WristPIDConfig.kV = 0;
    WristPIDConfig.kA = 0;
    WristPIDConfig.kP = 75; // 75
    WristPIDConfig.kI = 0;
    WristPIDConfig.kD = 0;

    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.withMagnetOffset(-0.098877 + 0.064453);
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> wristEncoder.getConfigurator().apply(cancoderConfig, 0.25));

    var wristConfig = new TalonFXConfiguration();
    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristConfig.Slot0 = WristPIDConfig;
    wristConfig.Feedback.RotorToSensorRatio = Constants.WRIST_GEAR_RATIO;
    wristConfig.Feedback.FeedbackRemoteSensorID = wristEncoder.getDeviceID();
    wristConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    wristConfig.MotionMagic.MotionMagicCruiseVelocity = 100 / Constants.WRIST_GEAR_RATIO;
    wristConfig.MotionMagic.MotionMagicAcceleration =
        wristConfig.MotionMagic.MotionMagicCruiseVelocity / 0.050;
    wristConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * Constants.WRIST_GEAR_RATIO;
    wristConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    wristConfig.ClosedLoopGeneral.ContinuousWrap = false;
    wristConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wristConfig.CurrentLimits.StatorCurrentLimit = Constants.WRIST_STATOR_CURRENT_LIMIT;
    wristConfig.CurrentLimits.SupplyCurrentLimit = Constants.WRIST_SUPPLY_CURRENT_LIMIT;
    wristConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    wristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    tryUntilOk(5, () -> WristMotor.getConfigurator().apply(wristConfig, 0.25));

    WristAngle = wristEncoder.getPosition();
    WristStatorCurrent = WristMotor.getStatorCurrent();
    WristSupplyCurrent = WristMotor.getSupplyCurrent();
    WristSpeed = WristMotor.getVelocity();
    WristVoltage = WristMotor.getMotorVoltage();
    WristTemp = WristMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        WristAngle,
        WristStatorCurrent,
        WristSupplyCurrent,
        WristSpeed,
        WristVoltage,
        WristTemp);

    ParentDevice.optimizeBusUtilizationForAll(WristMotor);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    var WristStatus =
        BaseStatusSignal.refreshAll(
            WristAngle,
            WristStatorCurrent,
            WristSupplyCurrent,
            WristSpeed,
            WristVoltage,
            WristTemp);

    inputs.wristAngle = WristAngle.getValueAsDouble();

    inputs.wristMotorConnected = WristStatus.isOK();
    inputs.wristEncoderConnected = wristEncoder.isConnected();
    inputs.wristStatorCurrent = WristStatorCurrent.getValueAsDouble();
    inputs.wristSupplyCurrent = WristSupplyCurrent.getValueAsDouble();
    inputs.wristSpeed = WristMotor.getVelocity().getValueAsDouble();
    inputs.wristVoltage = WristMotor.getMotorVoltage().getValueAsDouble();
    inputs.wristSetpoint = WristSetpoint;

    wristEncoderAlert.set(!inputs.wristEncoderConnected);
    wristAlert.set(!inputs.wristMotorConnected);
  }

  @Override
  public void setSetpoint(double position) {
    WristSetpoint = Units.degreesToRotations(position);
    WristMotor.setControl(m_request_Wrist.withPosition(WristSetpoint));
  }

  @Override
  public void setOutput(double speed) {
    WristMotor.set(speed);
  }
}
