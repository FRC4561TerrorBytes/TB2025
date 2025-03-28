package frc.robot.subsystems.algaeManipulator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class AlgaeManipulatorIOSim implements AlgaeManipulatorIO {

  private static final double LOOP_PERIOD_SECS = 0.02;

  private double algaeManipulatorAppliedVolts = 0.0;

  private DCMotorSim algaeManipulatorMotorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.01, 1.0),
          DCMotor.getNeo550(1));

  @Override
  public void updateInputs(AlgaeManipulatorIOInputs inputs) {
    algaeManipulatorMotorSim.update(LOOP_PERIOD_SECS);
    inputs.algaeManipulatorVelocity =
        Units.radiansToDegrees(algaeManipulatorMotorSim.getAngularVelocityRadPerSec());
    inputs.algaeManipulatorVoltage = algaeManipulatorAppliedVolts;
    inputs.algaeManipulatorCurrentAmps = Math.abs(algaeManipulatorMotorSim.getCurrentDrawAmps());
  }

  public void setOutput(double output) {
    algaeManipulatorAppliedVolts = MathUtil.clamp(output * 12, -12.0, 12.0);
    algaeManipulatorMotorSim.setInputVoltage(algaeManipulatorAppliedVolts);
  }
}
