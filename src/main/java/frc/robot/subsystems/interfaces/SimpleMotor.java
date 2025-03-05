package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public interface SimpleMotor {
  public static class SimpleMotorSettings {
    public double minPositionInRads = 0;
    public double maxPositionInRads = 10 * 2 * Math.PI; // 10 rotations
    public double startingPositionInRads = 0;
    public double targetHeightToleranceInRad = Math.PI;

    public double maxVelocityInRadiansPerSecond = 0.3;
    public double maxAccelerationInRadiansPerSecondSquared = 0.3;

    // feedforward is in *radian* units
    public SimpleMotorFeedforward feedforward =
        new SimpleMotorFeedforward(
            0, 0, 0); // These feedforward values assume the native Motor units of *radians*.

    // 2D Graphic Params
    public Color8Bit color = new Color8Bit(Color.kWhite);

    // Flywheel Simulation Settings
    public DCMotor motor = DCMotor.getNEO(1);
    public double moiKgMetersSquared = 0.025; // The moment of inertia of the winch
  }

  /**
   * Returns the current winch position
   *
   * @return the current winch position in radians
   */
  public double getCurrentPosition();

  /**
   * Returns the desired winch position
   *
   * @return the desired target position in radians
   */
  public double getTargetPosition();

  /**
   * Sets the desired target winch position
   *
   * @param positionRad the desired target position in radians
   */
  public void setTargetPosition(double positionRad);

  /**
   * @return true if position is at (or greater than) the max limit
   */
  public boolean isAtMaxLimit();

  /**
   * @return true if position is at (or less than) the min limit
   */
  public boolean isAtMinLimit();

  /**
   * @return true if the current winch position is at the desired target position
   */
  public boolean isAtSetpoint();

  /**
   * Returns the winch settings
   *
   * @return the current winch settings
   */
  public SimpleMotorSettings getSettings();
}
