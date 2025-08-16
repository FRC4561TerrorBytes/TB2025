package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.Supplier;
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

  public Command outtakeCoralAuto(Supplier<Pose2d> pose) {
    return new InstantCommand(
        () -> {
          Pose2d reef =
              new Pose2d(
                  AllianceFlipUtil.apply(
                      new Translation2d(
                          Units.inchesToMeters(176.746), Units.inchesToMeters(158.501))),
                  new Rotation2d());
          Pose2d centerRobot = pose.get();
          Logger.recordOutput("AutoOuttakePose", pose.get());
          Transform2d forwardOffset =
              new Transform2d(new Translation2d(-0.38, 0.0), new Rotation2d());
          Pose2d frontRobot = centerRobot.transformBy(forwardOffset);

          double centerDistance = centerRobot.getTranslation().getDistance(reef.getTranslation());
          double frontDistance = frontRobot.getTranslation().getDistance(reef.getTranslation());
          Logger.recordOutput("Front Distance To Reef", frontDistance);
          Logger.recordOutput("Center Distance To Reef", centerDistance);

          if (centerDistance < frontDistance) {
            Logger.recordOutput("Outtake Direction", "BACK");
            outtakeCoralBack().schedule();
          } else if (frontDistance < centerDistance) {
            Logger.recordOutput("Outtake Direction", "FRONT");
            outtakeCoralFront().schedule();
          } else {
            Logger.recordOutput("Outtake Direction", "BACK");
            outtakeCoralBack().schedule();
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
