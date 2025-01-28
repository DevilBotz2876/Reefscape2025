package frc.robot.commands.common.gizmo;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.Gizmo;
import java.util.function.DoubleSupplier;

/**
 * GizmoCommand implements a simple command that controls any object that implements the Gizmo
 * interface
 */
public class GizmoCommand extends Command {
  Gizmo gizmo;
  DoubleSupplier speedSupplier;

  /**
   * @param gizmo Object that implements the Gizmo interface
   * @param speed A function that returns a speed in the range [-1.0 .. 1.0]. Typically connected to
   *     a joystick/gamepad control
   */
  public GizmoCommand(Gizmo gizmo, DoubleSupplier speed) {
    this.gizmo = gizmo;
    this.speedSupplier = speed;

    addRequirements((Subsystem) gizmo);
  }

  /** Periodically sets the speed that is returned by the speedSupplier */
  @Override
  public void execute() {
    gizmo.setSpeed(speedSupplier.getAsDouble());
  }
}
