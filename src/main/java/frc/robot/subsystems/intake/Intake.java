package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.leds.Leds;
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

  public Command stopIntake() {
    return new SequentialCommandGroup(
        Commands.runOnce(() -> Leds.getInstance().intakeRunning = false),
        Commands.run(() -> this.setOutput(0), this));
  }

  public Command intakeCoral() {
    if (Constants.currentMode.equals(Mode.REAL)) {
      io.enableLimitSwitch();
    }
    return Commands.startEnd(
            () -> {
              this.setOutput(-1);
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
    if (Constants.currentMode.equals(Mode.REAL)) {
      io.disableLimitSwitch();
    }
    return Commands.startEnd(
            () -> this.setOutput(0.75),
            () -> {
              this.setOutput(0);
            },
            this)
        .withName("OuttakeBack");
  }

  public Command outtakeCoralFront() {
    if (Constants.currentMode.equals(Mode.REAL)) {
      io.disableLimitSwitch();
    }
    return Commands.startEnd(
            () -> this.setOutput(-0.75),
            () -> {
              this.setOutput(0);
            },
            this)
        .withName("OuttakeFront");
  }

  public Command outtakeCoralAuto(Pose2d pose) {
    System.out.println("Outtake Auto Command Running");
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      // closest three sides to blue alliance wall
      Logger.recordOutput("DriveX", pose.getX());
      Logger.recordOutput("RotationINTAKE", pose.getRotation().getDegrees());
      if (pose.getX() < 4.5) {
        Logger.recordOutput("Reef Side", "BLUE CLOSE");
        double degrees = pose.getRotation().getDegrees();
        if (degrees > 160.0 && degrees <= -160.0) {
          Logger.recordOutput("Outtake Direction", "FRONT");
          return outtakeCoralFront();
        } else if (degrees > -140.0 && degrees <= -90.0) {
          Logger.recordOutput("Outtake Direction", "FRONT");
          return outtakeCoralFront();
        } else if (degrees > -90.0 && degrees <= -30.0) {
          Logger.recordOutput("Outtake Direction", "BACK");
          return outtakeCoralBack();
        } else if (degrees > -30.0 && degrees <= 30.0) {
          Logger.recordOutput("Outtake Direction", "BACK");
          return outtakeCoralBack();
        } else if (degrees > 30.0 && degrees <= 90.0) {
          Logger.recordOutput("Outtake Direction", "BACK");
          return outtakeCoralBack();
        } else if (degrees > 90.0 && degrees <= 140.0) {
          Logger.recordOutput("Outtake Direction", "FRONT");
          return outtakeCoralFront();
        }
      }
      // three blue faces furthest away from blue alliance wall
      else {
        Logger.recordOutput("Reef Side", "BLUE FAR");
        return outtakeCoralFront();
      }
    } else {
      return outtakeCoralFront();
    }
    return outtakeCoralFront();
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
