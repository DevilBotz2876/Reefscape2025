package frc.robot.io.implementations.motor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.io.interfaces.MotorIO;

/**
 * The MotorIOBase abstract class implements a *software* based position and velocity PID for all
 * MotorIO implementations.
 *
 * <p>Child classes can override setVelocity()/setPosition() to use a *hardware* PID if desired
 * (e.g. SparkMax).
 */
public abstract class MotorIOBase implements MotorIO {
  public static class MotorIOBaseSettings {
    public MotorIOSettings motor = new MotorIOSettings();
    public PIDController pid = new PIDController(1.0, 0, 0); // PID values are unit-less.

    public int forwardLimitChannel = -1;
    public boolean forwardLimitNegate = false;
    public int reverseLimitChannel = -1;
    public boolean reverseLimitNegate = false;
  }

  private final MotorIOBaseSettings settings;
  private double targetPositionRad = 0;
  private double targetVelocityRadPerSec = 0;
  private double ffVolts = 0;
  private boolean softwarePidEnabled = false;
  private boolean positionPid = false;

  public MotorIOBase(MotorIOBaseSettings settings) {
    this.settings = settings;
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    /* Conversion of native units to friendlier human readable units */
    inputs.velocityDegreesPerSec = Units.radiansToDegrees(inputs.velocityRadPerSec);
    inputs.velocityRPMs = Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
    inputs.positionDegrees = Units.radiansToDegrees(inputs.positionRad);

    /* Software PID Implementation */
    if (softwarePidEnabled) {
      double appliedVolts = 0;

      /* Software based position PID */
      if (positionPid) {
        appliedVolts =
            ffVolts
                + settings.pid.calculate(inputs.positionRad, targetPositionRad)
                    * RobotController.getBatteryVoltage();

      } else {
        /* Software based velocity PID */

        appliedVolts =
            ffVolts
                + settings.pid.calculate(inputs.velocityRadPerSec, targetVelocityRadPerSec)
                    * RobotController.getBatteryVoltage();
      }
      setVoltage(appliedVolts);
    }
  }

  /**
   * Normalizes the voltage by clamping and inverting
   *
   * @param volts
   * @return normalized voltage
   */
  public final double calculateSafeVoltage(double volts) {
    volts = MathUtil.clamp(volts, -12.0, 12.0);
    if (settings.motor.inverted) {
      volts = -volts;
    }
    return volts;
  }

  @Override
  public boolean setVelocity(double velocityRadPerSec, double ffVolts) {
    if (settings.pid != null) {
      this.targetVelocityRadPerSec = velocityRadPerSec;
      this.ffVolts = ffVolts;
      softwarePidEnabled = true;
      positionPid = false;
      return true;
    } else {
      return false;
    }
  }

  @Override
  public boolean setPosition(double positionRad, double ffVolts) {
    if (settings.pid != null) {
      this.targetPositionRad = positionRad;
      this.ffVolts = ffVolts;
      softwarePidEnabled = true;
      positionPid = true;
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void setVoltage(double volts) {
    softwarePidEnabled = false;
    positionPid = false;
  }

  @Override
  public PIDController getPid() {
    return settings.pid;
  }

  @Override
  public void resetEncoder(double positionRad) {
    System.out.println(
        "TODO: reset encoder to: "
            + positionRad
            + " radians ("
            + Units.radiansToDegrees(positionRad)
            + " degrees)");
  }
}
