package frc.robot.subsystems.intake;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {
  private final SparkBase intakeMotor =
      new SparkMax(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);

  public IntakeIOReal() {
    var intakeConfig = new SparkMaxConfig();
    var limitSwitchConfig = new LimitSwitchConfig();
    limitSwitchConfig.forwardLimitSwitchEnabled(true);
    limitSwitchConfig.forwardLimitSwitchType(Type.kNormallyClosed);

    intakeConfig.idleMode(IdleMode.kBrake);
    // intakeConfig.smartCurrentLimit(20);
    intakeConfig.voltageCompensation(12.0);
    intakeConfig.inverted(false);
    intakeConfig.smartCurrentLimit(Constants.INTAKE_CURRENT_LIMIT);
    intakeConfig.apply(limitSwitchConfig);

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
    inputs.intakeLimitSwitch =
        intakeMotor.getForwardLimitSwitch().isPressed(); // CHANGE THIS TO USE CURRENT
  }

  public void setOutput(double speed) {
    intakeMotor.set(speed);
  }

  public void enableLimitSwitch() {
    var limitSwitchConfig = new LimitSwitchConfig();
    limitSwitchConfig.forwardLimitSwitchEnabled(true);
    limitSwitchConfig.forwardLimitSwitchType(Type.kNormallyOpen);

    tryUntilOk(
        intakeMotor,
        5,
        () ->
            intakeMotor.configure(
                new SparkMaxConfig().apply(limitSwitchConfig),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  public void disableLimitSwitch() {
    var limitSwitchConfig = new LimitSwitchConfig();
    limitSwitchConfig.forwardLimitSwitchEnabled(false);
    limitSwitchConfig.forwardLimitSwitchType(Type.kNormallyOpen);

    tryUntilOk(
        intakeMotor,
        5,
        () ->
            intakeMotor.configure(
                new SparkMaxConfig().apply(limitSwitchConfig),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters));
  }
}
