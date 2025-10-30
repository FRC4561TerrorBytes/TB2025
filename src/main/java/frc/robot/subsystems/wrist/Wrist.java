package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.ElevatorPosition;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private WristIO io;
  private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private final Alert WristDisconnectedAlert;

  public Wrist(WristIO io) {
    this.io = io;
    WristDisconnectedAlert = new Alert("Wrist motor disconnected", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist/Io", inputs);
    WristDisconnectedAlert.set(!inputs.wristMotorConnected);
  }

  public Pose3d getWristPose(double pivotAngle, double extensionHeight) {
    double extensionLength = 0.685;
    double offsetDown = 0.043;
    return new Pose3d(
        0.03
            - 0.2782
            + Math.cos(Units.rotationsToRadians(-pivotAngle)) * (extensionHeight + extensionLength)
            - Math.sin(Units.rotationsToRadians(-pivotAngle)) * offsetDown,
        0,
        0.37
            - 0.095
            - Math.sin(Units.rotationsToRadians(-pivotAngle)) * (extensionHeight + extensionLength)
            - Math.cos(Units.rotationsToRadians(-pivotAngle)) * offsetDown,
        new Rotation3d(0, Units.rotationsToRadians(-pivotAngle - inputs.wristAngle), 0));
  }

  public void setOutput(double speed) {
    io.setOutput(speed);
  }

  public void setSetpoint(ElevatorPosition position) {
    io.setSetpoint(position.wristPosition);
  }

  public boolean wristAtSetpoint(double tolerance) {
    if (Math.abs(inputs.wristAngle - inputs.wristSetpoint) > tolerance) return false;
    else return true;
  }
}
