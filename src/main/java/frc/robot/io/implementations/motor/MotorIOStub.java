package frc.robot.io.implementations.motor;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.interfaces.SimpleMotor.SimpleMotorSettings;

/**
 * A "Stub" implementation of a MotorIO that can be used for initial software bring-up/testing in
 * simulation.
 */
public class MotorIOStub extends MotorIOBase {
  private final DCMotorSim motorSim;
  private double appliedVolts = 0;
  private SimpleMotorSettings simpleMotorSettings;

  public MotorIOStub(MotorIOBaseSettings motorSettings, SimpleMotorSettings simpleMotorSettings) {
    super(motorSettings);

    this.simpleMotorSettings = simpleMotorSettings;

    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                simpleMotorSettings.motor,
                simpleMotorSettings.moiKgMetersSquared,
                motorSettings.motor.gearing),
            simpleMotorSettings.motor);
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = calculateSafeVoltage(volts, settings.motor.inverted);
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
    if (motorSim.getAngularPositionRad() <= simpleMotorSettings.minPositionInRads) {
      inputs.reverseLimit = true;
    } else {
      inputs.reverseLimit = false;
    }

    if (motorSim.getAngularPositionRad() >= simpleMotorSettings.maxPositionInRads) {
      inputs.forwardLimit = true;
    } else {
      inputs.forwardLimit = false;
    }

    super.updateInputs(inputs);
  }
}
