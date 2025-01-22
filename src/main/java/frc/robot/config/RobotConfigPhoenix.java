package frc.robot.config;

import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.auto.AutoNamedCommands;
import frc.robot.subsystems.drive.DriveSwerveYAGSL;
import frc.robot.subsystems.vision.VisionCamera;
import frc.robot.subsystems.vision.VisionSubsystem;
/* Override Phoenix specific constants here */
public class RobotConfigPhoenix extends RobotConfig {
  public RobotConfigPhoenix() {
    super(false, true, true, true, true, true, true);

    ArmConstants.minDistanceInMeters = Units.inchesToMeters(38);
    ArmConstants.maxDistanceInMeters = 4.0;
    ArmConstants.Ax2 = -3.2;
    ArmConstants.Bx = 23.7;
    ArmConstants.C = -10.3;

    // Phoenix has a Swerve drive train
    DriveConstants.rotatePidKp = 0.025;
    DriveConstants.rotatePidKi = 0.0;
    DriveConstants.rotatePidKd = 0.0;
    DriveConstants.rotatePidErrorInDegrees = 1;
    drive = new DriveSwerveYAGSL("yagsl/phoenix");

    cameras = new ArrayList<VisionCamera>();
    cameras.add(
        new VisionCamera(
            "my-first-photonvision",
            "1182",
            new Transform3d(
                new Translation3d(Units.inchesToMeters(13.25), 0, Units.inchesToMeters(6.25)),
                new Rotation3d(0, Units.degreesToRadians(30), 0))));

    VisionConstants.visionDistanceOffsetInMeters = -0.2;
    vision = new VisionSubsystem(cameras,
    AprilTagFields.k2025Reefscape);

    // if (Robot.isSimulation()) {
    //   vision.enableSimulation(() -> RobotConfig.drive.getPose(), false);
    // }

    AutoNamedCommands.configure();
    autoChooser = AutoBuilder.buildAutoChooser("None");
  }
}
