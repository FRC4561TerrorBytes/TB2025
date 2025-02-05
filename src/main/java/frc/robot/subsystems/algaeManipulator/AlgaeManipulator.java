package frc.robot.subsystems.algaeManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class AlgaeManipulator extends SubsystemBase {
  private AlgaeManipulatorIO io;
  private AlgaeManipulatorIOInputsAutoLogged inputs = new AlgaeManipulatorIOInputsAutoLogged();

  public AlgaeManipulator(AlgaeManipulatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("AlgaeManipulator/Io", inputs);
  }

  public void setOutput(double speed) {
    io.setOutput(speed);
  }

  public Command stopAlgaeManipulator(){
     return new RunCommand(() -> this.setOutput(0), this);
  }
}
