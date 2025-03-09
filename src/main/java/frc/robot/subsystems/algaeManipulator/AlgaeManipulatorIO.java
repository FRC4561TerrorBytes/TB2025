package frc.robot.subsystems.algaeManipulator;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeManipulatorIO {

  @AutoLog
  public static class AlgaeManipulatorIOInputs {
    public double algaeManipulatorVelocity = 0.0;
    public double algaeManipulatorCurrentAmps = 0.0;
    public double algaeManipulatorVoltage = 0.0;
    public boolean algaeManipulatorConnected = true;
  }

  public default void updateInputs(AlgaeManipulatorIOInputs inputs) {}

  public default void setOutput(double speed) {}

  public default void setIfSpinning(boolean spin) {}
}
