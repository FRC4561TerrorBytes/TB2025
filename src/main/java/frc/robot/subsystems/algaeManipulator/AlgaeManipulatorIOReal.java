package frc.robot.subsystems.algaeManipulator;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class AlgaeManipulatorIOReal implements AlgaeManipulatorIO {

  private final SparkBase algaeManipulatorMotor =
      new SparkMax(Constants.ALGAE_MANIPULATOR_ID, MotorType.kBrushless);
  private boolean spinning;

  public AlgaeManipulatorIOReal() {
    var algaeManipulatorConfig = new SparkMaxConfig();

    algaeManipulatorConfig.idleMode(IdleMode.kBrake);
    algaeManipulatorConfig.smartCurrentLimit(20);
    algaeManipulatorConfig.voltageCompensation(12.0);
    algaeManipulatorConfig.inverted(true);

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
    inputs.algaeManipulatorConnected = !algaeManipulatorMotor.hasActiveFault();

    SmartDashboard.putBoolean("Algae Manipulator Spinning", spinning);
  }

  public void setOutput(double speed) {
    algaeManipulatorMotor.set(speed);
  }

  @Override
  public void setIfSpinning(boolean spin) {
    spinning = spin;
  }
}
