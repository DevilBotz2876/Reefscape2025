package frc.robot.subsystems.controls.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.drive.DriveCommand;
import frc.robot.config.game.reefscape2025.RobotConfig;
import frc.robot.subsystems.interfaces.Drive;

public class DriveControls {
  public static void setupController(Drive drive, CommandXboxController controller) {
    SubsystemBase driveSubsystem = (SubsystemBase) drive;
    driveSubsystem.setDefaultCommand(
        new DriveCommand(
            drive,
            () -> MathUtil.applyDeadband(-controller.getLeftY(), 0.05), // Robot Strafe Front/Back
            () -> MathUtil.applyDeadband(-controller.getLeftX(), 0.05), // Robot Strafe Left/Right
            () -> MathUtil.applyDeadband(-controller.getRightX(), 0.05))); // Robot Rotate

    /* Debug/Test Only:
     *    Back Button = Zero Pose
     *    Start Button = Toggle Drive Orientation
     */
    controller.back().onTrue(new InstantCommand(() -> drive.resetOdometry()));
    controller
        .start()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (drive.isFieldOrientedDrive()) {
                    drive.setFieldOrientedDrive(false);
                  } else {
                    drive.setFieldOrientedDrive(true);
                  }
                })); // Toggle Drive Orientation

    //                 List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
    //         new Pose2d(2.0,4.0, Rotation2d.fromDegrees(0)),
    //         new Pose2d(7.0, 4.0, Rotation2d.fromDegrees(0))
    // );
    Pose2d targetPose = new Pose2d(6, 6, Rotation2d.fromDegrees(0));
    PathConstraints constraints = new PathConstraints(4.2672, 9.4664784, 2 * Math.PI, 4 * Math.PI);

    Command pathfindingCommand =
        AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            0.0 // Goal end velocity in meters/sec// Rotation delay distance in meters. This is how
            // far the robot should travel before attempting to rotate.
            );

    // Prevent the path from being flipped if the coordinates are already correct

    SmartDashboard.putData("follow", pathfindingCommand);
    controller.a().whileTrue(pathfindingCommand);
  }

  public static void addGUI(Drive drive, ShuffleboardTab tab) {
    //    int colIndex = 0;
    //    int rowIndex = 0;

    /* Autonomous Chooser */
    tab.add("Autononmous", RobotConfig.autoChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        //        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    /* Field Oriented Drive Indicator */
    tab.addBoolean("Field Oriented", () -> drive.isFieldOrientedDrive())
        .withWidget(BuiltInWidgets.kBooleanBox)
        //      .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);
  }
}
