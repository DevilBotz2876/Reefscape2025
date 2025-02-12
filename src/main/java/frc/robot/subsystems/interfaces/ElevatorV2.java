package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public interface ElevatorV2 {
  public static class ElevatorSettings {
    public double minHeightInMeters = 0.0;
    public double maxHeightInMeters = 1.0;
    public double startingHeightInMeters = 0.0;

    public double maxVelocityInMetersPerSecond = 45;
    public double maxAccelerationInMetersPerSecondSquared = 120;

    public double targetHeightToleranceInMeters = 0.05;

    // feedforward is in *meter* units
    public ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0);

    // 2D Graphic Params
    public Color8Bit color = new Color8Bit(Color.kWhite);

    // Elevator Simulation Settings
    public DCMotor motor = DCMotor.getNEO(1);
    public boolean simulateGravity = true;
    public double carriageMassKg =
        1.0; // The weight of the moveable portion of the elevator mechanism
  }

  /**
   * Returns the current elevator height
   *
   * @return the current elevator height in meters
   */
  public double getCurrentHeight();

  /**
   * Returns the desired elevator height
   *
   * @return the desired target height in meters
   */
  public double getTargetHeight();

  /**
   * Sets the desired target elevator height
   *
   * @param meters the desired target height in meters
   */
  public void setTargetHeight(double meters);

  /**
   * @return true if elevator height is at (or greater than) the max limit
   */
  public boolean isAtMaxLimit();

  /**
   * @return true if elevator height is at (or less than) the min limit
   */
  public boolean isAtMinLimit();

  /**
   * @return true if the current elevator height is at the desired target height
   */
  public boolean isAtSetpoint();

  /**
   * Returns the elevator settings
   *
   * @return the current elevator settings
   */
  public ElevatorSettings getSettings();
}
