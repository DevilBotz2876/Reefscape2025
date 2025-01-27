package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import java.util.Optional;

/**
 * A gizmo is a made up mechanism that consists of the following:
 *
 * <p>A pinwheel connected to a motor that has an encoder
 */
public interface Gizmo {
  /**
   * Sets the speed and rotation of the pinwheel
   *
   * @param speed In the range [-1.0 .. 1.0]. 0.0 is stopped with incre. negative value indicates
   *     counter-clockwise rotation (CCW), positive values indicate clockwise (CW).
   */
  public void setSpeed(double speed);

  /**
   * @return 2D graphical simulation of the pinwheel
   */
  public default Optional<Mechanism2d> create2dSim() {
    return Optional.empty();
  }
}
