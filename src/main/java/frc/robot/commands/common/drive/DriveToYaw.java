package frc.robot.commands.common.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.implementations.drive.DriveBase;
import frc.robot.subsystems.interfaces.Drive;
import java.util.function.DoubleSupplier;

public class DriveToYaw extends Command {
  Drive drive;
  DoubleSupplier yawDegrees;
  double targetYaw;
  PIDController turnPID =
      new PIDController(
          Drive.Constants.rotatePidKp, Drive.Constants.rotatePidKi, Drive.Constants.rotatePidKd);
  Timer timer = new Timer();

  public DriveToYaw(Drive drive, DoubleSupplier yawDegrees) {
    this.drive = drive;
    this.yawDegrees = yawDegrees;

    turnPID.setTolerance(DriveBase.Constants.rotatePidErrorInDegrees);
    turnPID.enableContinuousInput(-180, 180);
    addRequirements((Subsystem) drive);
  }

  @Override
  public void initialize() {
    // System.out.println("  START: " + this.getClass().getSimpleName());
    targetYaw = this.yawDegrees.getAsDouble();
    turnPID.reset();
    turnPID.setSetpoint(targetYaw);
    timer.reset();
  }

  @Override
  public void execute() {
    double rotate = turnPID.calculate(drive.getAngle());
    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, rotate * (drive.getMaxAngularSpeed() / 8));
    drive.runVelocity(speeds);
    targetYaw = this.yawDegrees.getAsDouble();
    double yawLeft = targetYaw - drive.getAngle();
    System.out.println("Angle Away: " + yawLeft);
  }

  @Override
  public boolean isFinished() {
    if (turnPID.atSetpoint()) {
      return true;
    } else {
      return false;
    }
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
