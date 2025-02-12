package frc.robot.io.implementations.motor;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * A "Stub" implementation of a MotorIO that can be used for initial software bring-up/testing in
 * simulation.
 */
public class MotorIOStub extends MotorIOBase {
  private final DCMotorSim motorSim;
  private double appliedVolts = 0;

  public static class MotorSimulationSettings {
    // DC Motor Simulation Settings
    public DCMotor motor = DCMotor.getNEO(1);
    public double moiKgMetersSquared = 1.0;
  }

  public MotorIOStub(
      MotorIOBaseSettings motorSettings, MotorSimulationSettings simulationSettings) {
    super(motorSettings);

    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                simulationSettings.motor,
                simulationSettings.moiKgMetersSquared,
                motorSettings.motor.gearing),
            simulationSettings.motor);
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = calculateSafeVoltage(volts);
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(0.02);

    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = motorSim.getCurrentDrawAmps();
    inputs.velocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
    inputs.positionRad = motorSim.getAngularPositionRad();
    inputs.accelerationRadPerSecSq = motorSim.getAngularAccelerationRadPerSecSq();

    super.updateInputs(inputs);
  }
}
