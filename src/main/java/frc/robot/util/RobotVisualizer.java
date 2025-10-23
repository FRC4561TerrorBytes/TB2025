package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.wrist.*;
import org.littletonrobotics.junction.Logger;

public class RobotVisualizer {
  private static Elevator elevator;
  private static Wrist wrist;

  public static void initialize(Elevator elevator, Wrist wrist) {
    RobotVisualizer.elevator = elevator;
    RobotVisualizer.wrist = wrist;
  }

  public static void update() {
    Logger.recordOutput(
        "FinalComponentPoses",
        new Pose3d[] {
          elevator.getPivotPose(),
          elevator.getExtensionPose(),
          wrist.getWristPose(elevator.getPivotPosition(), elevator.getExtensionPosition())
        });
  }
}
