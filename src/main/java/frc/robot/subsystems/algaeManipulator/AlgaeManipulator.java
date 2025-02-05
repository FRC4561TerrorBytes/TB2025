package frc.robot.subsystems.algaeManipulator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class AlgaeManipulator extends SubsystemBase {
  private AlgaeManipulatorIO io;
  private AlgaeManipulatorIOInputsAutoLogged inputs = new AlgaeManipulatorIOInputsAutoLogged();
  private final Alert algaeManipulatorDisconnectedAlert;

  public AlgaeManipulator(AlgaeManipulatorIO io) {
    this.io = io;
    algaeManipulatorDisconnectedAlert = new Alert("Algae motor disconnected", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("AlgaeManipulator/Io", inputs);
    algaeManipulatorDisconnectedAlert.set(!inputs.algaeManipulatorConnected);
  }

  public void setOutput(double speed) {
    io.setOutput(speed);
  }

  public Command stopAlgaeManipulator() {
    return new RunCommand(() -> this.setOutput(0), this);
  }
}
