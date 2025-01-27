package frc.robot.subsystems.pivot;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class PivotIOReal implements PivotIO {

  private TalonFX pivotMotor = new TalonFX(Constants.pivotMotorID);

  private final StatusSignal<Angle> pivotAngle;
  private final StatusSignal<Current> pivotStatorCurrent;
  private final StatusSignal<Current> pivotSupplyCurrent;
  private final StatusSignal<AngularVelocity> pivotSpeed;
  private final StatusSignal<Voltage> pivotVoltage;
  private final StatusSignal<Temperature> pivotOneTemp;

  private double pivotSetpoint = 0.0;
  private double pivotFeedForward = 0.0;

  private final MotionMagicVoltage m_request_pivot = new MotionMagicVoltage(0);

  public PivotIOReal() {
    var pivotPIDConfig = new Slot0Configs();
    pivotPIDConfig.GravityType = GravityTypeValue.Arm_Cosine;
    // pivotPIDConfig.kS = 0.28;
    pivotPIDConfig.kV = 0;
    pivotPIDConfig.kA = 0;
    pivotPIDConfig.kP = 0.1;
    pivotPIDConfig.kI = 0;
    pivotPIDConfig.kD = 0;

    var pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.Slot0 = pivotPIDConfig; //
    pivotConfig.Feedback.SensorToMechanismRatio = Constants.PIVOT_GEAR_RATIO;
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / Constants.PIVOT_GEAR_RATIO;
    pivotConfig.MotionMagic.MotionMagicAcceleration =
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    pivotConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * Constants.PIVOT_GEAR_RATIO;
    pivotConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    pivotConfig.ClosedLoopGeneral.ContinuousWrap = false;
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(pivotConfig, 0.25));

    pivotMotor.setPosition(-0.01025390625 + 0.003173828125 + 0.000244140625);

    pivotAngle = pivotMotor.getPosition();
    pivotStatorCurrent = pivotMotor.getStatorCurrent();
    pivotSupplyCurrent = pivotMotor.getSupplyCurrent();
    pivotSpeed = pivotMotor.getVelocity();
    pivotVoltage = pivotMotor.getMotorVoltage();
    pivotOneTemp = pivotMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        pivotAngle,
        pivotStatorCurrent,
        pivotSupplyCurrent,
        pivotSpeed,
        pivotVoltage,
        pivotOneTemp);

    ParentDevice.optimizeBusUtilizationForAll(pivotMotor);
  }

  public void updateInputs(PivotIOInputs inputs) {
    var pivotOneStatus =
        BaseStatusSignal.refreshAll(
            pivotAngle,
            pivotStatorCurrent,
            pivotSupplyCurrent,
            pivotSpeed,
            pivotVoltage,
            pivotOneTemp);

    inputs.pivotAngle = pivotAngle.getValueAsDouble();

    inputs.pivotMotorOneConnected = pivotOneStatus.isOK();
    inputs.pivotStatorCurrent = pivotStatorCurrent.getValueAsDouble();
    inputs.pivotSupplyCurrent = pivotSupplyCurrent.getValueAsDouble();
    inputs.pivotSpeed = pivotMotor.getVelocity().getValueAsDouble();
    inputs.pivotVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();
    inputs.pivotSetpoint = this.pivotSetpoint;

    m_request_pivot.FeedForward =
        Math.cos(Units.degreesToRadians(inputs.pivotAngle)) * pivotFeedForward;
  }

  public void setPivotPosition(double angle) {
    pivotSetpoint = angle;
    pivotMotor.setControl(m_request_pivot.withPosition(pivotSetpoint));
  }

  public void setPivotVoltage(double voltage) {
    pivotMotor.setVoltage(voltage);
  }
}
