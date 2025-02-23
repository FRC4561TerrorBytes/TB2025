package frc.robot.subsystems.elevator;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.RobotContainer.ElevatorPosition;

public class ElevatorIOReal implements ElevatorIO {

  private TalonFX pivotMotorL1 = new TalonFX(Constants.LEFT_PIVOT_ID_1);
  private TalonFX pivotMotorL2 = new TalonFX(Constants.LEFT_PIVOT_ID_2);
  private TalonFX pivotMotorR1 = new TalonFX(Constants.RIGHT_PIVOT_ID_1);
  private TalonFX pivotMotorR2 = new TalonFX(Constants.RIGHT_PIVOT_ID_2);

  private CANcoder pivotEncoder = new CANcoder(Constants.PIVOT_CANCODER_ID);

  private TalonFX extensionMotor = new TalonFX(Constants.EXTENSION_ID);

  private final StatusSignal<Angle> pivotAngle;
  private final StatusSignal<Current> pivotStatorCurrent;
  private final StatusSignal<Current> pivotSupplyCurrent;
  private final StatusSignal<AngularVelocity> pivotSpeed;
  private final StatusSignal<Voltage> pivotVoltage;
  private final StatusSignal<Temperature> pivotOneTemp;

  private final StatusSignal<Angle> extensionHeight;
  private final StatusSignal<Current> extensionStatorCurrent;
  private final StatusSignal<Current> extensionSupplyCurrent;
  private final StatusSignal<AngularVelocity> extensionSpeed;
  private final StatusSignal<Voltage> extensionVoltage;
  private final StatusSignal<Temperature> extensionTemp;

  private double pivotSetpoint = 0.0;
  private double pivotFeedForward = 0.0;
  private double extensionSetpoint = 0.0;
  private double extensionFeedForward = 0.0;

  private final MotionMagicVoltage m_request_pivot = new MotionMagicVoltage(0);
  private final MotionMagicVoltage m_request_extension = new MotionMagicVoltage(0);

  public ElevatorIOReal() {
    var pivotPIDConfig = new Slot0Configs();
    pivotPIDConfig.GravityType = GravityTypeValue.Arm_Cosine;
    // pivotPIDConfig.kS = 0.28;
    pivotPIDConfig.kV = 0;
    pivotPIDConfig.kA = 0;
    pivotPIDConfig.kP = 4;
    pivotPIDConfig.kI = 0;
    pivotPIDConfig.kD = 0;

    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.withMagnetOffset(-0.267090);
    tryUntilOk(5, () -> pivotEncoder.getConfigurator().apply(cancoderConfig, 0.25));

    var pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.Slot0 = pivotPIDConfig; //
    pivotConfig.Feedback.SensorToMechanismRatio = Constants.PIVOT_GEAR_RATIO;
    pivotConfig.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / Constants.PIVOT_GEAR_RATIO;
    pivotConfig.MotionMagic.MotionMagicAcceleration =
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    pivotConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * Constants.PIVOT_GEAR_RATIO;
    pivotConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    pivotConfig.ClosedLoopGeneral.ContinuousWrap = false;
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.28;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.4;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    tryUntilOk(5, () -> pivotMotorL1.getConfigurator().apply(pivotConfig, 0.25));

    pivotAngle = pivotEncoder.getPosition();
    pivotStatorCurrent = pivotMotorL1.getStatorCurrent();
    pivotSupplyCurrent = pivotMotorL1.getSupplyCurrent();
    pivotSpeed = pivotMotorL1.getVelocity();
    pivotVoltage = pivotMotorL1.getMotorVoltage();
    pivotOneTemp = pivotMotorL1.getDeviceTemp();

    var extensionPIDConfig = new Slot0Configs();
    extensionPIDConfig.GravityType = GravityTypeValue.Arm_Cosine;
    // pivotPIDConfig.kS = 0.28;
    extensionPIDConfig.kV = 0;
    extensionPIDConfig.kA = 0;
    extensionPIDConfig.kP = 0.5;
    extensionPIDConfig.kI = 0;
    extensionPIDConfig.kD = 0;

    var extensionConfig = new TalonFXConfiguration();
    extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    extensionConfig.Slot0 = extensionPIDConfig; //
    extensionConfig.Feedback.SensorToMechanismRatio = Constants.PIVOT_GEAR_RATIO;
    extensionConfig.MotionMagic.MotionMagicCruiseVelocity = 0.1;
    extensionConfig.MotionMagic.MotionMagicAcceleration =
        extensionConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    extensionConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * Constants.PIVOT_GEAR_RATIO;
    extensionConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    extensionConfig.ClosedLoopGeneral.ContinuousWrap = false;
    extensionConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.004;
    extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.08;
    extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    tryUntilOk(5, () -> extensionMotor.getConfigurator().apply(extensionConfig, 0.25));

    extensionMotor.setPosition(0);

    extensionHeight = extensionMotor.getPosition();
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
        extensionHeight,
        extensionStatorCurrent,
        extensionSupplyCurrent,
        extensionSpeed,
        extensionVoltage,
        extensionTemp);

    ParentDevice.optimizeBusUtilizationForAll(pivotMotorL1);

    pivotMotorL2.setControl(new Follower(pivotMotorL1.getDeviceID(), false));
    pivotMotorR1.setControl(new Follower(pivotMotorL1.getDeviceID(), true));
    pivotMotorR2.setControl(new Follower(pivotMotorL1.getDeviceID(), true));
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    var pivotOneStatus =
        BaseStatusSignal.refreshAll(
            pivotAngle,
            pivotStatorCurrent,
            pivotSupplyCurrent,
            pivotSpeed,
            pivotVoltage,
            pivotOneTemp,
            extensionHeight,
            extensionStatorCurrent,
            extensionSupplyCurrent,
            extensionSpeed,
            extensionVoltage,
            extensionTemp);

    inputs.pivotAngle = pivotAngle.getValueAsDouble();
    inputs.extensionHeight = extensionHeight.getValueAsDouble();

    inputs.extensionMotorConnected = pivotOneStatus.isOK();
    inputs.extensionStatorCurrent = pivotStatorCurrent.getValueAsDouble();
    inputs.extensionSupplyCurrent = pivotSupplyCurrent.getValueAsDouble();
    inputs.extensionSpeed = pivotMotorL1.getVelocity().getValueAsDouble();
    inputs.extensionVoltage = pivotMotorL1.getMotorVoltage().getValueAsDouble();
    inputs.extensionSetpoint = this.pivotSetpoint;

    inputs.pivotMotorOneConnected = pivotOneStatus.isOK();
    inputs.pivotStatorCurrent = pivotStatorCurrent.getValueAsDouble();
    inputs.pivotSupplyCurrent = pivotSupplyCurrent.getValueAsDouble();
    inputs.pivotSpeed = pivotMotorL1.getVelocity().getValueAsDouble();
    inputs.pivotVoltage = pivotMotorL1.getMotorVoltage().getValueAsDouble();
    inputs.pivotSetpoint = this.pivotSetpoint;

    m_request_pivot.FeedForward =
        Math.cos(Units.degreesToRadians(inputs.pivotAngle)) * pivotFeedForward;
  }

  public void setElevatorPosition(ElevatorPosition position){
    setPivotPosition(position.pivotPosition);
    setExtensionPosition(position.extensionPosition);
  }

  public void setPivotPosition(double angle) {
    pivotSetpoint = Units.degreesToRotations(angle);
    pivotMotorL1.setControl(m_request_pivot.withPosition(pivotSetpoint));
  }

  public void setPivotVoltage(double voltage) {
    pivotMotorL1.setVoltage(voltage);
  }

  public void setExtensionPosition(double angle) {
    extensionSetpoint = angle;
    extensionMotor.setControl(m_request_extension.withPosition(extensionSetpoint));
  }

  public void setExtensionVoltage(double voltage) {
    extensionMotor.setVoltage(voltage);
  }
}
