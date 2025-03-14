package frc.robot.subsystems.algaeManipulator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.Leds;

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
    setIfSpinning(true);
  }

  public Command runAlgaeManipulator(double output) {
    return new SequentialCommandGroup(Commands.runOnce(() -> Leds.getInstance().algaeRunning = true),
      Commands.run(() -> this.setOutput(output), this));
    }

  public Command stopAlgaeManipulator() {
    return new SequentialCommandGroup(Commands.runOnce(() -> Leds.getInstance().algaeRunning = false),
     Commands.run(() -> setOutput(0), this));
  }

  public void setIfSpinning(boolean spin) {
    io.setIfSpinning(spin);
  }
}
