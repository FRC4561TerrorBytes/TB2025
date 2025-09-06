package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double intakeVelocity = 0.0;
    public double intakeCurrentAmps = 0.0;
    public double intakeVoltage = 0.0;
    public boolean intakeConnected = true;
    public boolean coralPresent = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void enableLimitSwitch() {}

  public default void disableLimitSwitch() {}

  public default void setOutput(double speed) {}
}
