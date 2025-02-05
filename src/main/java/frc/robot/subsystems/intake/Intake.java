package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
}
