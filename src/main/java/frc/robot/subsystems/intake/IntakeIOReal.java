package frc.robot.subsystems.intake;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {
  private final SparkBase intakeMotor = new SparkMax(Constants.intakeMotorID, MotorType.kBrushless);

  public IntakeIOReal() {
    var intakeConfig = new SparkMaxConfig();

    intakeConfig.idleMode(IdleMode.kBrake);
    intakeConfig.smartCurrentLimit(20);
    intakeConfig.voltageCompensation(12.0);
    intakeConfig.inverted(false);

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
  }

  public void setOutput(double speed) {
    intakeMotor.set(speed);
  }
}
