package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {


      private TalonFX intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_ID);
  private final CANrange canRange = new CANrange(Constants.CANRANGE_ID);
  private final StatusSignal<Current> IntakeStatorCurrent;
  private final StatusSignal<Current> IntakeSupplyCurrent;
  private final StatusSignal<AngularVelocity> IntakeSpeed;
  private final StatusSignal<Voltage> IntakeVoltage;
  private final StatusSignal<Temperature> IntakeTemp;

  public IntakeIOReal() {
    var intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeConfig.CurrentLimits.StatorCurrentLimit = Constants.WRIST_STATOR_CURRENT_LIMIT;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = Constants.WRIST_SUPPLY_CURRENT_LIMIT;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(intakeConfig, 0.25));
    

    CANrangeConfiguration config = new CANrangeConfiguration();
    config.ProximityParams.MinSignalStrengthForValidMeasurement = 2000;
    config.ProximityParams.ProximityThreshold = Units.inchesToMeters(3); // in meters
    config.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
    tryUntilOk(5, () -> canRange.getConfigurator().apply(config, 0.25));

    IntakeStatorCurrent = intakeMotor.getStatorCurrent();
    IntakeSupplyCurrent = intakeMotor.getSupplyCurrent();
    IntakeSpeed = intakeMotor.getVelocity();
    IntakeVoltage = intakeMotor.getMotorVoltage();
    IntakeTemp = intakeMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        IntakeStatorCurrent,
        IntakeSupplyCurrent,
        IntakeSpeed,
        IntakeVoltage,
        IntakeTemp);

    ParentDevice.optimizeBusUtilizationForAll(intakeMotor, canRange);

  
  }

  public void updateInputs(IntakeIOInputs inputs) {
    var IntakeStatus =
        BaseStatusSignal.refreshAll(
            IntakeStatorCurrent,
            IntakeSupplyCurrent,
            IntakeSpeed,
            IntakeVoltage,
            IntakeTemp);

    inputs.intakeVelocity = intakeMotor.getVelocity().getValueAsDouble();
    inputs.intakeCurrentAmps = IntakeStatorCurrent.getValueAsDouble();
    inputs.intakeVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
    inputs.intakeConnected = IntakeStatus.isOK();
    inputs.coralPresent = canRange.getIsDetected().getValue();
    inputs.canRangeConnected = canRange.isConnected();
  }

  public void setOutput(double speed) {
    intakeMotor.set(speed);
  }
}
