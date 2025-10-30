package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.Leds;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final Alert intakeDisconnectedAlert;
  private final Alert intakeSensorDisconnectedAlert;

  public Intake(IntakeIO io) {
    this.io = io;
    intakeDisconnectedAlert = new Alert("Intake Disconnected", AlertType.kError);
    intakeSensorDisconnectedAlert = new Alert("Intake CANRange Disconnected", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/IO", inputs);
    intakeDisconnectedAlert.set(!inputs.intakeConnected);
    intakeSensorDisconnectedAlert.set(!inputs.canRangeConnected);
  }

  public void setOutput(double speed) {
    io.setOutput(speed);
  }

  public boolean coralPresent() {
    return inputs.coralPresent;
  }

  public Command stopIntake() {
    return new SequentialCommandGroup(
        Commands.runOnce(() -> Leds.getInstance().intakeRunning = false),
        Commands.run(() -> this.setOutput(0), this));
  }

  public Command intakeCoral() {
    return Commands.startEnd(
            () -> {
              this.setOutput(1);
              Leds.getInstance().intakeRunning = true;
            },
            () -> {
              this.setOutput(0);
              Leds.getInstance().intakeRunning = false;
            },
            this)
        .withName("Intake");
  }

  public Command outtakeCoralBack() {
    return Commands.startEnd(
            () -> this.setOutput(-0.75),
            () -> {
              this.setOutput(0);
            },
            this)
        .withName("OuttakeBack");
  }

  public Command outtakeCoralFront() {
    return Commands.startEnd(
            () -> this.setOutput(0.75),
            () -> {
              this.setOutput(0);
            },
            this)
        .withName("OuttakeFront");
  }
}
