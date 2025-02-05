package frc.robot.subsystems.algaeManipulator;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class AlgaeManipulatorIOReal implements AlgaeManipulatorIO {

  private final SparkBase algaeManipulatorMotor =
      new SparkMax(Constants.algaeManipulatorMotorID, MotorType.kBrushless);

  public AlgaeManipulatorIOReal() {
    var algaeManipulatorConfig = new SparkMaxConfig();

    algaeManipulatorConfig.idleMode(IdleMode.kBrake);
    algaeManipulatorConfig.smartCurrentLimit(20);
    algaeManipulatorConfig.voltageCompensation(12.0);
    algaeManipulatorConfig.inverted(false);

    tryUntilOk(
        algaeManipulatorMotor,
        5,
        () ->
            algaeManipulatorMotor.configure(
                algaeManipulatorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  public void updateInputs(AlgaeManipulatorIOInputs inputs) {
    inputs.algaeManipulatorVelocity = algaeManipulatorMotor.getEncoder().getVelocity();
    inputs.algaeManipulatorCurrentAmps = algaeManipulatorMotor.getOutputCurrent();
    inputs.algaeManipulatorVoltage = algaeManipulatorMotor.getBusVoltage();
  }

  public void setOutput(double speed) {
    algaeManipulatorMotor.set(speed);
  }
}
