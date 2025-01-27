package frc.robot.config.game.reefscape2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.io.implementations.motor.MotorIOStub;
import frc.robot.subsystems.controls.gizmo.GizmoControls;
import frc.robot.subsystems.implementations.drive.DriveSwerveYAGSL;
import frc.robot.subsystems.implementations.gizmo.GizmoSubsystem;

/* Override Phoenix specific constants here */
public class RobotConfigStub extends RobotConfig {
  GizmoSubsystem gizmo;

  public RobotConfigStub() {
    super(false, true, true);

    drive = new DriveSwerveYAGSL("yagsl/stub");
    if (Robot.isSimulation()) {
      drive.setPose(new Pose2d(new Translation2d(1, 1), new Rotation2d()));
    }

    // We create a GizmoSubsystem using the MotorIOStub implementation of a MotorIO.
    //
    // We set inverted=true because we want positive voltage to result in spinning clockwise. The
    // inversion depends on how the motor is physically connected to the gizmo, so specific to this
    // robot configuration.
    //
    // We set moi=0.025 as a random default, but should be set based on CAD/mech teams calculations
    // of the actual mechanism
    //
    // We set the gear ratio to 50:1 so that the simulation is as accurate as possible.
    gizmo = new GizmoSubsystem(new MotorIOStub(true, 0.025, 50));
  }

  @Override
  public void configureBindings() {
    // Configure the default bindings of the parent class
    super.configureBindings();

    // Setup Gizmo specific controls and 2D simulation
    GizmoControls.setupController(gizmo, mainController);
    GizmoControls.setup2dSimulation(gizmo);
  }
}
