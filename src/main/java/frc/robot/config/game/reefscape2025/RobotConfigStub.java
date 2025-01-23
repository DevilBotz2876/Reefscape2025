package frc.robot.config.game.reefscape2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.subsystems.implementations.drive.DriveSwerveYAGSL;

/* Override Phoenix specific constants here */
public class RobotConfigStub extends RobotConfig {
  public RobotConfigStub() {
    super(false, true, true);

    drive = new DriveSwerveYAGSL("yagsl/stub");
    if (Robot.isSimulation()) {
      drive.setPose(new Pose2d(new Translation2d(1, 1), new Rotation2d()));
    }
  }
}
