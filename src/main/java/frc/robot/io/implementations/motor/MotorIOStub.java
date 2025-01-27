package frc.robot.io.implementations.motor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.io.interfaces.MotorIO;

/**
 * A "Stub" implementation of a MotorIO that can be used for initial software bring-up/testing in
 * simulation
 */
public class MotorIOStub implements MotorIO {
  private boolean inverted = false;
  private final DCMotorSim motorSim;

  private double appliedVolts = 0.0;

  /** Default constructor that doesn't take any arguments. */
  public MotorIOStub() {
    this(false, 0.025, 1.0);
  }

  /** Constructor that allows setting whether the motor is inverted */
  public MotorIOStub(boolean inverted, double moi, double gearing) {
    this.inverted = inverted;

    // Simulate a Kraken X60 motor with the shaft connected to a mechanism with the specified moment
    // of inertia and gear ratio
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), moi, gearing),
            DCMotor.getKrakenX60(1));
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(0.02);

    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = motorSim.getCurrentDrawAmps();
    inputs.velocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
    inputs.positionRad = motorSim.getAngularPositionRad();
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    if (inverted) {
      appliedVolts = -appliedVolts;
    }
  }
}
