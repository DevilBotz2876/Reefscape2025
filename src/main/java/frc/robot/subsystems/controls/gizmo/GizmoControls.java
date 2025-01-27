package frc.robot.subsystems.controls.gizmo;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.gizmo.GizmoCommand;
import frc.robot.subsystems.interfaces.Gizmo;

/* GizmoControls is a wrapped that connects the gizmo subsystem to the rest of the system (e.g. controllers, gui, etc) */
public class GizmoControls {
  /**
   * This function binds the XBoxController commands to control a subsystem that implements the
   * Gizmo interface
   *
   * @param gizmo Object that implements the Gizmo interface
   * @param controller controller object
   */
  public static void setupController(Gizmo gizmo, CommandXboxController controller) {
    /* Since Gizmo is explicitly passed in, but we know it is a GizmoSubsystem, we cast it as a Subsystem object so that we can set a default command for controlling it */
    SubsystemBase gizmoSubsystem = (SubsystemBase) gizmo;

    /* We use the left/right analog trigger to control the gizmo.  Since the trigger returns a value from [0.0 .. 1.0], we treat the left trigger to set a CCW rotation and the right trigger to set a CW rotation.  We subtract the right trigger value from the left to get the single value in the range of [-1.0 .. 1.0] which is what the GizmoCommand needs.  We also set a deadband of 0.05 so that we can ignore any minor drift when the controls are not being pressed */
    gizmoSubsystem.setDefaultCommand(
        new GizmoCommand(
            gizmo,
            () ->
                MathUtil.applyDeadband(
                    controller.getRightTriggerAxis() - controller.getLeftTriggerAxis(), 0.05)));
  }

  /* Here, we get a 2D simulation from the Gizmo.  If one has been implemented, we add it to the Smartdashboard so that we can see if using Shuffleboard/AdvantageScope/Etc */
  public static void setup2dSimulation(Gizmo gizmo) {
    if (gizmo.create2dSim().isPresent()) {
      SmartDashboard.putData("Gizmo 2D", gizmo.create2dSim().get());
    }
  }
}
