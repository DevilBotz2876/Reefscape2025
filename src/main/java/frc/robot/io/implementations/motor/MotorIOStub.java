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
  /* Here, we are simulating a Kraken X60 motor with the shaft connected to a mechanism with an arbitrary moment of inertia (0.025) and a arbitrary gearbox ratio (50:1) */
  private final DCMotorSim motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.025, 50),
          DCMotor.getKrakenX60(1));

  private double appliedVolts = 0.0;

  /** Default constructor that doesn't take any arguments. */
  public MotorIOStub() {
    this(false);
  }

  /** Constructor that allows setting whether the motor is inverted */
  public MotorIOStub(boolean inverted) {
    this.inverted = inverted;
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
