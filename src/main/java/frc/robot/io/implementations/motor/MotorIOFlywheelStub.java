package frc.robot.io.implementations.motor;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.interfaces.Flywheel.FlywheelSettings;

/**
 * A "Stub" implementation of a MotorIO that can be used for initial software bring-up/testing in
 * simulation. Simulates the physics of a connected Flywheel.
 */
public class MotorIOFlywheelStub extends MotorIOBase {
  private final FlywheelSim flywheelSim;
  private double appliedVolts = 0;

  public MotorIOFlywheelStub(MotorIOBaseSettings motorSettings, FlywheelSettings flywheelSettings) {
    super(motorSettings);

    flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                flywheelSettings.motor,
                flywheelSettings.moiKgMetersSquared,
                motorSettings.motor.gearing),
            flywheelSettings.motor);
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = calculateSafeVoltage(volts, settings.motor.inverted);
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    flywheelSim.setInputVoltage(appliedVolts);
    flywheelSim.update(0.02);

    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = flywheelSim.getCurrentDrawAmps();
    inputs.velocityRadPerSec = flywheelSim.getAngularVelocityRadPerSec();
    inputs.accelerationRadPerSecSq = flywheelSim.getAngularAccelerationRadPerSecSq();

    super.updateInputs(inputs);
  }

  /* Flywheels should only be used with setVelocity, so here we override the setPosition functionality to prevent an erroneous call to it */

  @Override
  public final boolean setPosition(double position, double ffVolts) {
    return false;
  }

  @Override
  public void resetEncoder(double positionRad) {
    System.out.println("Resetting encoder for a flywheel is not supported!");
  }
}
