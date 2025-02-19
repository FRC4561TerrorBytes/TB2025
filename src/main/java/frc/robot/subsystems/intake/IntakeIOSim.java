package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {

  private static final double LOOP_PERIOD_SECS = 0.02;

  private double intakeAppliedVolts = 0.0;

  private DCMotorSim algaeManipulatorMotorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.01, 1.0),
          DCMotor.getNeo550(1));

  @Override
  public void updateInputs(IntakeIOInputs Inputs) {
    algaeManipulatorMotorSim.update(LOOP_PERIOD_SECS);
    Inputs.intakeVelocity = intakeAppliedVolts;
    Inputs.intakeCurrentAmps = Math.abs(algaeManipulatorMotorSim.getCurrentDrawAmps());
  }

  public void setOutput(double volts) {
    intakeAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    algaeManipulatorMotorSim.setInputVoltage(intakeAppliedVolts);
  }
}
