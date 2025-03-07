package frc.robot.commands.common.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.implementations.drive.DriveBase;
import frc.robot.subsystems.interfaces.Drive;
import java.util.function.DoubleSupplier;

public class DriveToPositionX extends Command {
  Drive drive;
  DoubleSupplier distanceMeters;
  Pose2d startingPose, targetPose;
  // HACK: manually copied from Swerve subsystem translation PID
  PIDController positionPID = new PIDController(7.5, 0.0, 0.0);
  Timer timer = new Timer();

  public DriveToPositionX(Drive drive, DoubleSupplier distanceMeters) {
    this.drive = drive;
    this.distanceMeters = distanceMeters;

    // HACK: not sure what a good tolerance would be...
    positionPID.setTolerance(0.25);
    addRequirements((Subsystem) drive);
  }

  @Override
  public void initialize() {
    double d = this.distanceMeters.getAsDouble();
    targetPose = drive.getPose().transformBy(new Transform2d(d, 0, new Rotation2d()));
    positionPID.reset();
    positionPID.setSetpoint(d);
    timer.reset();
    if (Constants.debugCommands) {
      System.out.println(
          "START: " + this.getClass().getSimpleName() + " distance: " + d + " currentPosition: 0");
    }
  }

  @Override
  public void execute() {
    double linearSpeed = positionPID.calculate(targetPose.relativeTo(drive.getPose()).getX());
    drive.runVelocity(new ChassisSpeeds(linearSpeed * drive.getMaxLinearSpeed(), 0, 0));
  }

  @Override
  public boolean isFinished() {
    if (positionPID.atSetpoint()) {
      if (timer.get() >= DriveBase.Constants.pidSettlingTimeInSeconds) {
        return true;
      }
    } else {
      timer.reset();
    }
    return false;
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
