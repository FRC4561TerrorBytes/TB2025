package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants.Reef;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AutoOuttakeDirection extends Command {
  Supplier<Pose2d> pose;
  Intake intake;
  boolean outtakeFront = false;

  public AutoOuttakeDirection(Intake intake, Supplier<Pose2d> pose) {
    this.intake = intake;
    this.pose = pose;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    Pose2d centerRobot = pose.get();
    Logger.recordOutput("Outtake Pose", centerRobot);
    Transform2d forwardOffset = new Transform2d(new Translation2d(-0.38, 0.0), new Rotation2d());
    Pose2d frontRobot = centerRobot.transformBy(forwardOffset);

    double centerDistance =
        centerRobot.getTranslation().getDistance(AllianceFlipUtil.apply(Reef.center));
    double frontDistance =
        frontRobot.getTranslation().getDistance(AllianceFlipUtil.apply(Reef.center));
    Logger.recordOutput("Front Distance To Reef", frontDistance);
    Logger.recordOutput("Center Distance To Reef", centerDistance);

    if (centerDistance < frontDistance) {
      Logger.recordOutput("Outtake Direction", "BACK");
      outtakeFront = false;
    } else if (frontDistance < centerDistance) {
      Logger.recordOutput("Outtake Direction", "FRONT");
      outtakeFront = true;
    } else {
      outtakeFront = false;
    }
  }

  @Override
  public void execute() {
    if (outtakeFront) {
      intake.setOutput(-.25);
    } else {
      intake.setOutput(.25);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.setOutput(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
