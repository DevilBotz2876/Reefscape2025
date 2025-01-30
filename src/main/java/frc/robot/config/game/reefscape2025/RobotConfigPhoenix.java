package frc.robot.config.game.reefscape2025;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.common.arm.ArmCommand;
import frc.robot.commands.common.arm.ArmToPosition;
import frc.robot.commands.common.elevator.ElevatorCommand;
import frc.robot.subsystems.implementations.drive.DriveBase;
import frc.robot.subsystems.implementations.drive.DriveSwerveYAGSL;
import frc.robot.subsystems.interfaces.Drive;
import frc.robot.subsystems.interfaces.Vision.Camera;

/* Override Phoenix specific constants here */
public class RobotConfigPhoenix extends RobotConfig {
  public RobotConfigPhoenix() {
    super(false, false, false);

    // Phoenix has a Swerve drive train
    Drive.Constants.rotatePidKp = 0.025;
    Drive.Constants.rotatePidKi = 0.0;
    Drive.Constants.rotatePidKd = 0.0;
    DriveBase.Constants.rotatePidErrorInDegrees = 1;
    drive = new DriveSwerveYAGSL("yagsl/phoenix");

    vision.addCamera(
        new Camera(
            "photonvision",
            new Transform3d(
                new Translation3d(0.221, 0, .164),
                new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(0)))));

    NamedCommands.registerCommand("Move Elevator", new ElevatorCommand(RobotConfig.elevator, () -> 1.0).withTimeout(0.05));
    NamedCommands.registerCommand("Move Arm", new ArmToPosition(RobotConfig.arm, () -> 45.0));
    autoChooser = AutoBuilder.buildAutoChooser("Sit Still");
  }
}
