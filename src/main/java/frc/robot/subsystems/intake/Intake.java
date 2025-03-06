package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final Alert intakeDisconnectedAlert;

  public Intake(IntakeIO io) {
    this.io = io;
    intakeDisconnectedAlert = new Alert("Intake Disconnected", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/IO", inputs);
    intakeDisconnectedAlert.set(!inputs.intakeConnected);
  }

  public void setOutput(double speed) {
    io.setOutput(speed);
  }

  public boolean coralPresent() {
    return inputs.intakeLimitSwitch;
  }

  public Command intakeCoral() {
    return new RunCommand(() -> this.setOutput(0.75), this)
        .until(() -> inputs.intakeLimitSwitch)
        .andThen(() -> this.setOutput(0));
  }

  public Command outtakeCoral() {
    if (Constants.currentMode == Mode.REAL) {
      io.disableLimitSwitch();
    }
    return new RunCommand(() -> this.setOutput(0.75), this)
        // .withTimeout(2.0)
        .andThen(
            () -> {
              if (Constants.currentMode == Mode.REAL) {
                io.enableLimitSwitch();
              }
            });
  }

  public Command outtakeL1Coral() {
    if (Constants.currentMode == Mode.REAL) {
      io.disableLimitSwitch();
    }
    return new RunCommand(() -> this.setOutput(-0.75), this)
        // .withTimeout(2.0)
        .andThen(
            () -> {
              if (Constants.currentMode == Mode.REAL) {
                io.enableLimitSwitch();
              }
            });
  }
}
