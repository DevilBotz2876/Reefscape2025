package frc.robot.config.game.reefscape2025;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.implementations.drive.DriveBase;
import frc.robot.subsystems.implementations.drive.DriveSwerveYAGSL;
import frc.robot.subsystems.implementations.vision.VisionSubsystem;
import frc.robot.subsystems.interfaces.Drive;
import frc.robot.subsystems.interfaces.Vision.Camera;

/* Override Nemo specific constants here */
public class RobotConfigNemo extends RobotConfig {
  public RobotConfigNemo() {
    super(false, true, true);

    // Nemo has a Swerve drive train
    Drive.Constants.rotatePidKp = 0.025;
    Drive.Constants.rotatePidKi = 0.0;
    Drive.Constants.rotatePidKd = 0.0;
    DriveBase.Constants.rotatePidErrorInDegrees = 1;
    drive = new DriveSwerveYAGSL("yagsl/nemo");

    // Camera placement at WPI practice field (2/21/2025)
    vision.addCamera(
        new Camera(
            "my-first-photonvision", // back
            new Transform3d(
                new Translation3d(Units.inchesToMeters(3), Units.inchesToMeters(10.75), Units.inchesToMeters(36.75)),
                new Rotation3d(0.0, Units.degreesToRadians(30), Units.degreesToRadians(10)))));
    vision.addCamera(
        new Camera(
          "left",
          new Transform3d(
              new Translation3d(Units.inchesToMeters(-4.4), Units.inchesToMeters(14.5), Units.inchesToMeters(7)),
              new Rotation3d(0.0, Units.degreesToRadians(-5), Units.degreesToRadians(0.0)))));
    vision.addCamera(
        new Camera(
            "right",
            new Transform3d(
                new Translation3d(Units.inchesToMeters(-4.4), Units.inchesToMeters(-14.5), Units.inchesToMeters(7)),
                new Rotation3d(0.0, Units.degreesToRadians(-5), Units.degreesToRadians(0.0)))));
    vision.addCamera(
        new Camera(
            "front", // back
            new Transform3d(
                new Translation3d(Units.inchesToMeters(-14.5), Units.inchesToMeters(-3.75), Units.inchesToMeters(7)),
                new Rotation3d(0.0, Units.degreesToRadians(-5), Units.degreesToRadians(0.0)))));
    
  }
}
