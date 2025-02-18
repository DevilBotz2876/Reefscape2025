package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public interface ArmV2 {
  public static class ArmSettings {
    public double minAngleInDegrees = 0.0;
    public double maxAngleInDegrees = 180.0;
    public double startingAngleInDegrees = 0.0;

    public double maxVelocityInDegreesPerSecond = 45;
    public double maxAccelerationInDegreesPerSecondSquared = 120;

    public double targetAngleToleranceInDegrees = 1.0;

    // feedforward is in *radian* units
    public ArmFeedforward feedforward = new ArmFeedforward(0.0, 0, 0);

    // 2D Graphic Params
    public Color8Bit color = new Color8Bit(Color.kWhite);

    // Motor Arm Simulation Settings
    public DCMotor motor = DCMotor.getNEO(1);
    public boolean simulateGravity = true;
    public double armLengthInMeters = 0.5;
    public double armMassInKg = 1;
  }

  /**
   * Returns the current arm angle
   *
   * @return the current angle in degrees
   */
  public double getCurrentAngle();

  /**
   * Returns the desired arm angle
   *
   * @return the desired target angle in degrees
   */
  public double getTargetAngle();

  /**
   * Sets the desired target arm angle
   *
   * @param degrees the desired target angle in degrees
   */
  public void setTargetAngle(double degrees);

  /**
   * @return true if arm angle is at (or greater than) the max limit
   */
  public boolean isAtMaxLimit();

  /**
   * @return true if arm angle is at (or less than) the min limit
   */
  public boolean isAtMinLimit();

  /**
   * @return true if the current arm angle is at the desired target angle
   */
  public boolean isAtSetpoint();

  /**
   * Returns the arm settings
   *
   * @return the current arm settings
   */
  public ArmSettings getSettings();
}
