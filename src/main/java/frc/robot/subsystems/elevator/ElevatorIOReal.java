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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
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
  private final StatusSignal<Current> pivotOneStatorCurrent;
  private final StatusSignal<Current> pivotOneSupplyCurrent;
  private final StatusSignal<AngularVelocity> pivotOneSpeed;
  private final StatusSignal<Voltage> pivotOneVoltage;
  private final StatusSignal<Temperature> pivotOneTemp;

  private final StatusSignal<Current> pivotTwoStatorCurrent;
  private final StatusSignal<Current> pivotTwoSupplyCurrent;
  private final StatusSignal<AngularVelocity> pivotTwoSpeed;
  private final StatusSignal<Voltage> pivotTwoVoltage;
  private final StatusSignal<Temperature> pivotTwoTemp;

  private final StatusSignal<Current> pivotThreeStatorCurrent;
  private final StatusSignal<Current> pivotThreeSupplyCurrent;
  private final StatusSignal<AngularVelocity> pivotThreeSpeed;
  private final StatusSignal<Voltage> pivotThreeVoltage;
  private final StatusSignal<Temperature> pivotThreeTemp;

  private final StatusSignal<Current> pivotFourStatorCurrent;
  private final StatusSignal<Current> pivotFourSupplyCurrent;
  private final StatusSignal<AngularVelocity> pivotFourSpeed;
  private final StatusSignal<Voltage> pivotFourVoltage;
  private final StatusSignal<Temperature> pivotFourTemp;

  private final StatusSignal<Angle> extensionHeight;
  private final StatusSignal<Current> extensionStatorCurrent;
  private final StatusSignal<Current> extensionSupplyCurrent;
  private final StatusSignal<AngularVelocity> extensionSpeed;
  private final StatusSignal<Voltage> extensionVoltage;
  private final StatusSignal<Temperature> extensionTemp;

  private final Alert pivotOneAlert;
  private final Alert pivotTwoAlert;
  private final Alert pivotThreeAlert;
  private final Alert pivotFourAlert;
  private final Alert pivotEncoderAlert;

  private double pivotSetpoint = 0.0;
  private double pivotFeedForward = 0.0;
  private double extensionSetpoint = 0.0;
  private double extensionFeedForward = 0.5;

  private final MotionMagicVoltage m_request_pivot = new MotionMagicVoltage(0);
  private final MotionMagicVoltage m_request_extension = new MotionMagicVoltage(0);

  public ElevatorIOReal() {
    var pivotPIDConfig = new Slot0Configs();
    pivotPIDConfig.GravityType = GravityTypeValue.Arm_Cosine;
    // pivotPIDConfig.kS = 0.28;
    pivotPIDConfig.kV = 0;
    pivotPIDConfig.kA = 0;
    pivotPIDConfig.kP = 55;
    pivotPIDConfig.kI = 3;
    pivotPIDConfig.kD = 0;

    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.withMagnetOffset(-0.267090);
    tryUntilOk(5, () -> pivotEncoder.getConfigurator().apply(cancoderConfig, 0.25));

    var pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.Slot0 = pivotPIDConfig; //
    pivotConfig.Feedback.RotorToSensorRatio = Constants.PIVOT_GEAR_RATIO;
    pivotConfig.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 400.0 / Constants.PIVOT_GEAR_RATIO;
    pivotConfig.MotionMagic.MotionMagicAcceleration =
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity / 0.85;
    pivotConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * Constants.PIVOT_GEAR_RATIO;
    pivotConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    pivotConfig.ClosedLoopGeneral.ContinuousWrap = false;
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(100);
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(10);
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    pivotConfig.CurrentLimits.StatorCurrentLimit = Constants.PIVOT_STATOR_CURRENT_LIMIT;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = Constants.PIVOT_SUPPLY_CURRENT_LIMIT;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    tryUntilOk(5, () -> pivotMotorL1.getConfigurator().apply(pivotConfig, 0.25));
    tryUntilOk(5, () -> pivotMotorL2.getConfigurator().apply(pivotConfig, 0.25));
    tryUntilOk(5, () -> pivotMotorR1.getConfigurator().apply(pivotConfig, 0.25));
    tryUntilOk(5, () -> pivotMotorR2.getConfigurator().apply(pivotConfig, 0.25));

    pivotAngle = pivotEncoder.getPosition();
    pivotOneStatorCurrent = pivotMotorL1.getStatorCurrent();
    pivotOneSupplyCurrent = pivotMotorL1.getSupplyCurrent();
    pivotOneSpeed = pivotMotorL1.getVelocity();
    pivotOneVoltage = pivotMotorL1.getMotorVoltage();
    pivotOneTemp = pivotMotorL1.getDeviceTemp();

    pivotTwoStatorCurrent = pivotMotorL2.getStatorCurrent();
    pivotTwoSupplyCurrent = pivotMotorL2.getSupplyCurrent();
    pivotTwoSpeed = pivotMotorL2.getVelocity();
    pivotTwoVoltage = pivotMotorL2.getMotorVoltage();
    pivotTwoTemp = pivotMotorL2.getDeviceTemp();

    pivotThreeStatorCurrent = pivotMotorR1.getStatorCurrent();
    pivotThreeSupplyCurrent = pivotMotorR1.getSupplyCurrent();
    pivotThreeSpeed = pivotMotorR1.getVelocity();
    pivotThreeVoltage = pivotMotorR1.getMotorVoltage();
    pivotThreeTemp = pivotMotorR1.getDeviceTemp();

    pivotFourStatorCurrent = pivotMotorR2.getStatorCurrent();
    pivotFourSupplyCurrent = pivotMotorR2.getSupplyCurrent();
    pivotFourSpeed = pivotMotorR2.getVelocity();
    pivotFourVoltage = pivotMotorR2.getMotorVoltage();
    pivotFourTemp = pivotMotorR2.getDeviceTemp();

    var extensionPIDConfig = new Slot0Configs();
    extensionPIDConfig.GravityType = GravityTypeValue.Arm_Cosine;
    // pivotPIDConfig.kS = 0.28;
    extensionPIDConfig.kV = 0;
    extensionPIDConfig.kA = 0;
    extensionPIDConfig.kP = 25;
    extensionPIDConfig.kI = 0;
    extensionPIDConfig.kD = 0;

    var extensionConfig = new TalonFXConfiguration();
    extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    extensionConfig.Slot0 = extensionPIDConfig;
    extensionConfig.Feedback.SensorToMechanismRatio = Constants.EXTENSION_GEAR_RATIO;
    extensionConfig.MotionMagic.MotionMagicCruiseVelocity = 1.5;
    extensionConfig.MotionMagic.MotionMagicAcceleration =
        extensionConfig.MotionMagic.MotionMagicCruiseVelocity / 0.050;
    extensionConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * Constants.EXTENSION_GEAR_RATIO;
    extensionConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    extensionConfig.ClosedLoopGeneral.ContinuousWrap = false;
    extensionConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.004;
    extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.08;
    extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    extensionConfig.CurrentLimits.StatorCurrentLimit = Constants.EXTENSION_STATOR_CURRENT_LIMIT;
    extensionConfig.CurrentLimits.SupplyCurrentLimit = Constants.EXTENSION_SUPPLY_CURRENT_LIMIT;
    extensionConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    extensionConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
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
        pivotOneStatorCurrent,
        pivotOneSupplyCurrent,
        pivotOneSpeed,
        pivotOneVoltage,
        pivotOneTemp,
        pivotTwoStatorCurrent,
        pivotTwoSupplyCurrent,
        pivotTwoSpeed,
        pivotTwoVoltage,
        pivotTwoTemp,
        pivotThreeStatorCurrent,
        pivotThreeSupplyCurrent,
        pivotThreeSpeed,
        pivotThreeVoltage,
        pivotThreeTemp,
        pivotFourStatorCurrent,
        pivotFourSupplyCurrent,
        pivotFourSpeed,
        pivotFourVoltage,
        pivotFourTemp,
        extensionHeight,
        extensionStatorCurrent,
        extensionSupplyCurrent,
        extensionSpeed,
        extensionVoltage,
        extensionTemp);

    ParentDevice.optimizeBusUtilizationForAll(pivotMotorL1);
    ParentDevice.optimizeBusUtilizationForAll(pivotMotorL2);
    ParentDevice.optimizeBusUtilizationForAll(pivotMotorR1);
    ParentDevice.optimizeBusUtilizationForAll(pivotMotorR2);
    ParentDevice.optimizeBusUtilizationForAll(extensionMotor);

    pivotMotorL2.setControl(new Follower(pivotMotorL1.getDeviceID(), false));
    pivotMotorR1.setControl(new Follower(pivotMotorL1.getDeviceID(), true));
    pivotMotorR2.setControl(new Follower(pivotMotorL1.getDeviceID(), true));

    pivotOneAlert = new Alert("Left One Pivot Motor Disconnected.", AlertType.kWarning);
    pivotTwoAlert = new Alert("Left Two Pivot Motor Disconnected.", AlertType.kWarning);
    pivotThreeAlert = new Alert("Right One Motor Pivot Disconnected.", AlertType.kWarning);
    pivotFourAlert = new Alert("Right Two Motor Pivot Disconnected.", AlertType.kWarning);
    pivotEncoderAlert = new Alert("Pivot CANCoder Disconnected.", AlertType.kWarning);
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    var pivotOneStatus =
        BaseStatusSignal.refreshAll(
            pivotAngle,
            pivotOneStatorCurrent,
            pivotOneSupplyCurrent,
            pivotOneSpeed,
            pivotOneVoltage,
            pivotOneTemp);

    var pivotTwoStatus =
        BaseStatusSignal.refreshAll(
            pivotAngle,
            pivotTwoStatorCurrent,
            pivotTwoSupplyCurrent,
            pivotTwoSpeed,
            pivotTwoVoltage,
            pivotTwoTemp);

    var pivotThreeStatus =
        BaseStatusSignal.refreshAll(
            pivotAngle,
            pivotThreeStatorCurrent,
            pivotThreeSupplyCurrent,
            pivotThreeSpeed,
            pivotThreeVoltage,
            pivotThreeTemp);

    var pivotFourStatus =
        BaseStatusSignal.refreshAll(
            pivotAngle,
            pivotFourStatorCurrent,
            pivotFourSupplyCurrent,
            pivotFourSpeed,
            pivotFourVoltage,
            pivotFourTemp);
    var extensionStatus =
        BaseStatusSignal.refreshAll(
            extensionHeight,
            extensionStatorCurrent,
            extensionSupplyCurrent,
            extensionSpeed,
            extensionVoltage,
            extensionTemp);

    inputs.pivotAngle = pivotAngle.getValueAsDouble();
    inputs.extensionHeight = extensionHeight.getValueAsDouble();

    inputs.extensionMotorConnected = extensionStatus.isOK();
    inputs.extensionStatorCurrent = extensionStatorCurrent.getValueAsDouble();
    inputs.extensionSupplyCurrent = extensionSupplyCurrent.getValueAsDouble();
    inputs.extensionSpeed = extensionMotor.getVelocity().getValueAsDouble();
    inputs.extensionVoltage = extensionMotor.getMotorVoltage().getValueAsDouble();
    inputs.extensionSetpoint = extensionSetpoint;

    m_request_extension.FeedForward =
        Math.sin(Units.rotationsToRadians(inputs.pivotAngle)) * extensionFeedForward;

    inputs.pivotMotorOneConnected = pivotOneStatus.isOK();
    inputs.pivotOneStatorCurrent = pivotOneStatorCurrent.getValueAsDouble();
    inputs.pivotOneSupplyCurrent = pivotOneSupplyCurrent.getValueAsDouble();
    inputs.pivotOneSpeed = pivotMotorL1.getVelocity().getValueAsDouble();
    inputs.pivotOneVoltage = pivotMotorL1.getMotorVoltage().getValueAsDouble();
    inputs.pivotSetpoint = pivotSetpoint;

    inputs.pivotMotorTwoConnected = pivotTwoStatus.isOK();
    inputs.pivotTwoStatorCurrent = pivotTwoStatorCurrent.getValueAsDouble();
    inputs.pivotTwoSupplyCurrent = pivotTwoSupplyCurrent.getValueAsDouble();
    inputs.pivotTwoSpeed = pivotMotorL2.getVelocity().getValueAsDouble();
    inputs.pivotTwoVoltage = pivotMotorL2.getMotorVoltage().getValueAsDouble();

    inputs.pivotMotorThreeConnected = pivotThreeStatus.isOK();
    inputs.pivotThreeStatorCurrent = pivotThreeStatorCurrent.getValueAsDouble();
    inputs.pivotThreeSupplyCurrent = pivotThreeSupplyCurrent.getValueAsDouble();
    inputs.pivotThreeSpeed = pivotMotorR1.getVelocity().getValueAsDouble();
    inputs.pivotThreeVoltage = pivotMotorR1.getMotorVoltage().getValueAsDouble();

    inputs.pivotMotorFourConnected = pivotFourStatus.isOK();
    inputs.pivotFourStatorCurrent = pivotFourStatorCurrent.getValueAsDouble();
    inputs.pivotFourSupplyCurrent = pivotFourSupplyCurrent.getValueAsDouble();
    inputs.pivotFourSpeed = pivotMotorR2.getVelocity().getValueAsDouble();
    inputs.pivotFourVoltage = pivotMotorR2.getMotorVoltage().getValueAsDouble();

    inputs.pivotEncoderConnected = pivotEncoder.isConnected();

    pivotOneAlert.set(!inputs.pivotMotorOneConnected);
    pivotTwoAlert.set(!inputs.pivotMotorTwoConnected);
    pivotThreeAlert.set(!inputs.pivotMotorThreeConnected);
    pivotFourAlert.set(!inputs.pivotMotorFourConnected);
    pivotEncoderAlert.set(!inputs.pivotEncoderConnected);
  }

  public void setTargetPosition(ElevatorPosition position) {
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

  public void setExtensionPosition(double position) {
    // TO-DO MAKE THIS GO AFTER PIVOT WHEN GOING TO STOW
    extensionSetpoint = position;
    extensionMotor.setControl(m_request_extension.withPosition(extensionSetpoint));
  }

  public void setExtensionVoltage(double voltage) {
    extensionMotor.setVoltage(voltage);
  }
}
