package frc.robot.io.implementations.motor;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * A "Stub" implementation of a MotorIO that can be used for initial software bring-up/testing in
 * simulation.
 */
public class MotorIOStub extends MotorIOBase {
  private final DCMotorSim motorSim;
  private double appliedVolts = 0;
  private MotorSimulationSettings simSettings;

  public static class MotorSimulationSettings {
    // DC Motor Simulation Settings
    public DCMotor motor = DCMotor.getNEO(1);
    public double moiKgMetersSquared = 1.0;

    // TODO: should not hard-code these values.  Need to be set/passed in from
    // RobotConfigStub.java somehow.
    public double forwardLimitPositionRads = Units.degreesToRadians(85.0);
    public double reverseLimitPositionRads = Units.degreesToRadians(5.0);
  }

  public MotorIOStub(
      MotorIOBaseSettings motorSettings, MotorSimulationSettings simulationSettings) {
    super(motorSettings);

    simSettings = simulationSettings;

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

    // Simulate limit switch behavior
    if (inputs.positionRad < simSettings.reverseLimitPositionRads) {
      inputs.reverseLimit = true;
    } else {
      inputs.reverseLimit = false;
    }

    if (inputs.positionRad > simSettings.forwardLimitPositionRads) {
      inputs.forwardLimit = false;
    } else {
      inputs.forwardLimit = true;
    }

    super.updateInputs(inputs);
  }
}
