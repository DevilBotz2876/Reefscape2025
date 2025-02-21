package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public interface Flywheel {
  public static class FlywheelSettings {
    public double maxVelocityInRPMs = 6000;
    public double targetVelocityToleranceInRPMs = 100;

    // feedforward is in *radian* units
    public SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);

    // 2D Graphic Params
    public Color8Bit color = new Color8Bit(Color.kWhite);

    // Flywheel Simulation Settings
    public DCMotor motor = DCMotor.getNEO(1);
    public double moiKgMetersSquared = 0.025; // The moment of inertia of the flywheel
  }

  /**
   * Returns the current flywheel velocity
   *
   * @return the current flywheel velocity in RPMs
   */
  public double getCurrentVelocity();

  /**
   * Returns the desired flywheel velocity
   *
   * @return the desired target velocity in RPMs
   */
  public double getTargetVelocity();

  /**
   * Sets the desired target flywheel velocity
   *
   * @param velocityRPMs the desired target velocity in RPMs
   */
  public void setTargetVelocity(double velocityRPMs);

  /**
   * @return true if the current flywheel velocity is at the desired target velocity
   */
  public boolean isAtSetpoint();

  /**
   * Returns the flywheel settings
   *
   * @return the current flywheel settings
   */
  public FlywheelSettings getSettings();
}
