package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {
  private final SparkBase intakeMotor =
      new SparkMax(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);

  private final CANrange canRange = new CANrange(Constants.CANRANGE_ID);

  public IntakeIOReal() {
    var intakeConfig = new SparkMaxConfig();

    intakeConfig.idleMode(IdleMode.kBrake);
    intakeConfig.voltageCompensation(12.0);
    intakeConfig.inverted(false);
    intakeConfig.smartCurrentLimit(Constants.INTAKE_CURRENT_LIMIT);
    intakeConfig.limitSwitch.reverseLimitSwitchEnabled(false);
    intakeConfig.limitSwitch.forwardLimitSwitchEnabled(false);

    CANrangeConfiguration config = new CANrangeConfiguration();
    config.ProximityParams.MinSignalStrengthForValidMeasurement = 2000;
    config.ProximityParams.ProximityThreshold = Units.inchesToMeters(3); // in meters
    config.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
    tryUntilOk(5, () -> canRange.getConfigurator().apply(config, 0.25));

    tryUntilOk(
        intakeMotor,
        5,
        () ->
            intakeMotor.configure(
                intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void updateInputs(IntakeIOInputs inputs) {

    inputs.intakeVelocity = intakeMotor.getEncoder().getVelocity();
    inputs.intakeCurrentAmps = intakeMotor.getOutputCurrent();
    inputs.intakeVoltage = intakeMotor.getBusVoltage();
    inputs.intakeConnected = !intakeMotor.hasActiveFault();
    inputs.coralPresent = canRange.getIsDetected().getValue();
    inputs.canRangeConnected = canRange.isConnected();
  }

  public void setOutput(double speed) {
    intakeMotor.set(speed);
  }
}
