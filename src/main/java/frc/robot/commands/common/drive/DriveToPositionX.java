package frc.robot.commands.common.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.interfaces.Drive;
import java.util.function.DoubleSupplier;

public class DriveToPositionX extends Command {
  Drive drive;
  DoubleSupplier distanceMeters;
  Pose2d targetPose;

  public DriveToPositionX(Drive drive, DoubleSupplier distanceMeters) {
    this.drive = drive;
    this.distanceMeters = distanceMeters;
    addRequirements((Subsystem) drive);
  }

  @Override
  public void initialize() {
    System.out.println("  START: " + this.getClass().getSimpleName());
    double d = this.distanceMeters.getAsDouble();
    targetPose = drive.getPose().transformBy(new Transform2d(d, 0, new Rotation2d()));
  }

  @Override
  public void execute() {

    drive.runVelocity(
        new ChassisSpeeds(
            Math.signum(distanceMeters.getAsDouble()) * drive.getMaxLinearSpeed(), 0, 0));
  }

  @Override
  public boolean isFinished() {
    double currentX = drive.getPose().getX();
    double targetX = targetPose.getX();
    double distanceLeft = currentX - targetX;
    System.out.println("Distance Away: " + distanceLeft);
    return Math.abs(distanceLeft) >= Math.abs(distanceMeters.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.err.println("INTERRUPTED: " + this.getClass().getSimpleName());
    }

    drive.runVelocity(new ChassisSpeeds());
    if (Constants.debugCommands) {
      System.out.println("  END: " + this.getClass().getSimpleName());
    }
  }
}
