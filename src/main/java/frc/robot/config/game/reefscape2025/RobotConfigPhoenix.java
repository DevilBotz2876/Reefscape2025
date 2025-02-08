package frc.robot.config.game.reefscape2025;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.common.arm.ArmToPosition;
import frc.robot.commands.common.drive.DriveStraightPID;
import frc.robot.commands.common.elevator.ElevatorToPosition;
import frc.robot.subsystems.implementations.drive.DriveBase;
import frc.robot.subsystems.implementations.drive.DriveSwerveYAGSL;
import frc.robot.subsystems.interfaces.Drive;
import frc.robot.subsystems.interfaces.Vision.Camera;

/* Override Phoenix specific constants here */
public class RobotConfigPhoenix extends RobotConfig {
  public RobotConfigPhoenix() {
    super(false, true, false, true, true, true);

    // Phoenix has a Swerve drive train
    Drive.Constants.rotatePidKp = 0.025;
    Drive.Constants.rotatePidKi = 0.0;
    Drive.Constants.rotatePidKd = 0.0;
    DriveBase.Constants.rotatePidErrorInDegrees = 1;
    drive = new DriveSwerveYAGSL("yagsl/phoenix");

    drive.setPose(drive.getPose().transformBy(new Transform2d(2, 5, Rotation2d.kZero)));
    vision.addCamera(
        new Camera(
            "my-first-photonvision",
            new Transform3d(
                new Translation3d(0.221, 0, .164),
                new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(0)))));
  }
}
