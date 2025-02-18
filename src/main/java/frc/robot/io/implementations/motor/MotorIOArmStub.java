package frc.robot.io.implementations.motor;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.interfaces.Arm.ArmSettings;

/**
 * A "Stub" implementation of a MotorIO that can be used for initial software bring-up/testing in
 * simulation. Simulates the physics of a connected Arm.
 */
public class MotorIOArmStub extends MotorIOBase {
  private final SingleJointedArmSim armSim;
  private double appliedVolts = 0;
  private ArmSettings armSettings;

  public MotorIOArmStub(MotorIOBaseSettings motorSettings, ArmSettings armSettings) {
    super(motorSettings);

    armSim =
        new SingleJointedArmSim(
            armSettings.motor,
            motorSettings.motor.gearing,
            SingleJointedArmSim.estimateMOI(armSettings.armLengthInMeters, armSettings.armMassInKg),
            armSettings.armLengthInMeters,
            Units.degreesToRadians(armSettings.minAngleInDegrees),
            Units.degreesToRadians(armSettings.maxAngleInDegrees),
            armSettings.simulateGravity,
            Units.degreesToRadians(armSettings.startingAngleInDegrees));

    this.armSettings = armSettings;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = calculateSafeVoltage(volts);
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    armSim.setInputVoltage(appliedVolts);
    armSim.update(0.02);

    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = armSim.getCurrentDrawAmps();
    inputs.velocityRadPerSec = armSim.getVelocityRadPerSec();
    inputs.positionRad = armSim.getAngleRads();

    // Simulate limit switch behavior
    if (Units.radiansToDegrees(armSim.getAngleRads()) <= armSettings.minAngleInDegrees + 1) {
      inputs.reverseLimit = true;
    } else {
      inputs.reverseLimit = false;
    }

    if (Units.radiansToDegrees(armSim.getAngleRads()) >= armSettings.maxAngleInDegrees - 1) {
      inputs.forwardLimit = true;
    } else {
      inputs.forwardLimit = false;
    }

    super.updateInputs(inputs);
  }

  /* Arms should only be used with setPosition, so here we override the setVelocity functionality to prevent an erroneous call to it */
  @Override
  public final boolean setVelocity(double velocityRadPerSec, double ffVolts) {
    return false;
  }
}
