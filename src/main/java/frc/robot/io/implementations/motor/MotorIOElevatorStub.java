package frc.robot.io.implementations.motor;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.interfaces.Elevator.ElevatorSettings;

/**
 * A "Stub" implementation of a MotorIO that can be used for initial software bring-up/testing in
 * simulation. Simulates the physics of an Elevator connected via a rope/pulley to translated
 * rotational motion to linear motion.
 */
public class MotorIOElevatorStub extends MotorIOBase {
  private final ElevatorSim elevatorSim;
  private double appliedVolts = 0;
  private final ElevatorSettings elevatorSettings;

  /** Constructor that allows setting whether the motor is inverted */
  public MotorIOElevatorStub(MotorIOBaseSettings motorSettings, ElevatorSettings elevatorSettings) {
    super(motorSettings);
    this.elevatorSettings = elevatorSettings;

    // Simulate a Kraken X60 motor with the shaft connected to a mechanism with the specified moment
    // of inertia and gear ratio
    elevatorSim =
        new ElevatorSim(
            elevatorSettings.motor,
            motorSettings.motor.gearing,
            elevatorSettings.carriageMassKg,
            motorSettings.motor.drumRadiusMeters,
            elevatorSettings.minHeightInMeters,
            elevatorSettings.maxHeightInMeters,
            elevatorSettings.simulateGravity,
            elevatorSettings.startingHeightInMeters);
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = calculateSafeVoltage(volts, settings.motor.inverted);
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    elevatorSim.setInputVoltage(appliedVolts);
    elevatorSim.update(0.02);

    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = elevatorSim.getCurrentDrawAmps();
    inputs.velocityRadPerSec =
        elevatorSim.getVelocityMetersPerSecond() / settings.motor.drumRadiusMeters;
    inputs.positionRad = elevatorSim.getPositionMeters() / settings.motor.drumRadiusMeters;

    // Simulate limit switch behavior
    if (inputs.positionMeters < elevatorSettings.minHeightInMeters) {
      inputs.reverseLimit = true;
    } else {
      inputs.reverseLimit = false;
    }

    if (inputs.positionMeters > elevatorSettings.maxHeightInMeters) {
      inputs.forwardLimit = true;
    } else {
      inputs.forwardLimit = false;
    }

    super.updateInputs(inputs);
  }

  /* Elevators should only be used with setPosition, so here we override the setVelocity functionality to prevent an erroneous call to it */

  @Override
  public final boolean setVelocity(double velocityRadPerSec, double ffVolts) {
    return false;
  }
}
