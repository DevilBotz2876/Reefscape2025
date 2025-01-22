package frc.robot.config.game.reefscape2025;

import frc.robot.subsystems.implementations.drive.DriveSwerveYAGSL;

/* Override Phoenix specific constants here */
public class RobotConfigStub extends RobotConfig {
  public RobotConfigStub() {
    super(false, true, true);

    drive = new DriveSwerveYAGSL("yagsl/stub");
  }
}
