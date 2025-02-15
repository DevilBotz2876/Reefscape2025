package frc.robot.subsystems.controls.drive;

import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.commands.common.drive.DriveCommand;
import frc.robot.commands.driveAssist.DriveToStationY;
import frc.robot.config.game.reefscape2025.RobotConfig;
import frc.robot.subsystems.interfaces.Drive;

public class DriveControls {
  private enum TargetPoseOptions {
    ORIGIN,
    FEEDER_1,
    FEEDER_2,
    PROCESSOR
  }

  
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
    
    // Define destinations for our "dynamic go-to-pose" functionality
    Pose2d poseOrigin = new Pose2d(0,0,Rotation2d.fromDegrees(0)),
      poseFeeder1 = new Pose2d(1,4,Rotation2d.fromDegrees(0)),
      poseFeeder2 = new Pose2d(4,1,Rotation2d.fromDegrees(0)),
      poseProcessor = new Pose2d(6,6,Rotation2d.fromDegrees(0));
    PathConstraints constraints = new PathConstraints(4.2672, 9.4664784, 2 * Math.PI, 4 * Math.PI);

    // Temporary UI to allow user to modify destination on-the-fly 
    SendableChooser<String> chooser = new SendableChooser<>();
    chooser.setDefaultOption("Origin", "O");
    chooser.addOption("Feeder 1", "F1");
    chooser.addOption("Feeder 2", "F2");
    chooser.addOption("Processor", "P");
    SmartDashboard.putData("Pose choices", chooser);

    
    // Define command to go to specific pose
    Command coolGoToPose = new SelectCommand<>(Map.ofEntries(
      Map.entry(DriveControls.TargetPoseOptions.ORIGIN, AutoBuilder.pathfindToPose(poseOrigin, constraints, 0.0)),
      Map.entry(DriveControls.TargetPoseOptions.FEEDER_1, AutoBuilder.pathfindToPose(poseFeeder1, constraints, 0.0)),
      Map.entry(DriveControls.TargetPoseOptions.FEEDER_2, AutoBuilder.pathfindToPose(poseFeeder2, constraints, 0.0)),
      Map.entry(DriveControls.TargetPoseOptions.PROCESSOR, AutoBuilder.pathfindToPose(poseProcessor, constraints, 0.0))
    ), () -> {
      switch (chooser.getSelected()) {
        case "F1":
          return DriveControls.TargetPoseOptions.FEEDER_1;
        case "F2":
          return DriveControls.TargetPoseOptions.FEEDER_2;
        case "P":
          return DriveControls.TargetPoseOptions.PROCESSOR;
        default:
          return DriveControls.TargetPoseOptions.ORIGIN;
      }
    });

    // dynamically go to destination 
    controller.a().whileTrue(coolGoToPose);
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
