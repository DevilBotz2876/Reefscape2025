package frc.robot.config;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.auto.AutoNamedCommands;
import frc.robot.subsystems.drive.DriveSwerveYAGSL;

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

    // cameras = new ArrayList<VisionCamera>();
    // cameras.add(
    //     new VisionCamera(
    //         "shooter",
    //         "1182",
    //         new Transform3d(
    //             new Translation3d(-Units.inchesToMeters(10.75), 0, Units.inchesToMeters(8)),
    //             new Rotation3d(0, Units.degreesToRadians(-33), Units.degreesToRadians(180)))));

    // VisionConstants.visionDistanceOffsetInMeters = -0.2;
    // vision = new VisionSubsystem(cameras,
    // AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());

    // if (Robot.isSimulation()) {
    //   vision.enableSimulation(() -> RobotConfig.drive.getPose(), false);
    // }

    AutoNamedCommands.configure();
    autoChooser = AutoBuilder.buildAutoChooser("None");
  }
}
