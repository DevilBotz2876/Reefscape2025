package frc.robot.config.game.reefscape2025;

import frc.robot.subsystems.implementations.drive.DriveBase;
import frc.robot.subsystems.implementations.drive.DriveSwerveYAGSL;
import frc.robot.subsystems.interfaces.Drive;

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
  }
}
