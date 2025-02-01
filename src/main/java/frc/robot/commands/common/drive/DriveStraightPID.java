// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.common.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.implementations.drive.DriveBase;
import frc.robot.subsystems.interfaces.Drive;

/** This command is a PID controller that drives the robot straight to a set distance. */
public class DriveStraightPID extends Command {
  Drive drive;
  PIDController distancePid = new PIDController(1.5, 0, 0);
  PIDController straightPid = new PIDController(1, 0, 0);
  double distance;
  double startAngle;
  double startDistance;
   final SlewRateLimiter speedSlewRateLimiter = new SlewRateLimiter(2.75);
   double maxSpeed = 0; // in meters/sec

  /**
   * The constructor for the Drive Straight PID command.
   *
   * @param drivetrain The drive train subsystem.
   * @param distance The distance (in meters) the robot needs to cover.
   */
  public DriveStraightPID(Drive drive, double distance) {
    this.drive = drive;
    this.distance = distance;
    straightPid.enableContinuousInput(0, 360);
    distancePid.setTolerance(0.02);
    straightPid.setTolerance(DriveBase.Constants.rotatePidErrorInDegrees);
    addRequirements((Subsystem) drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /**
   * The constructor for the Drive Straight PID command.
   *
   * @param drive The drive train subsystem.
   * @param distance The distance (in meters) the robot needs to cover.
   * @param maxSpeed The max speed the robot should travel
   */
  public DriveStraightPID(Drive drive, double distance, double maxSpeed) {
    this(drive, distance);

    this.maxSpeed = maxSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startAngle = drive.getAngle();
    startDistance = drive.getPose().getX();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = distancePid.calculate(drive.getPose().getX() - startDistance, distance);
    double turnError = straightPid.calculate(drive.getAngle(), startAngle);
    double speed = speedSlewRateLimiter.calculate(output);
    if (0 != maxSpeed) {
      speed = MathUtil.clamp(speed, -maxSpeed, maxSpeed);
    }
    ChassisSpeeds speeds =
        new ChassisSpeeds(speed,0, -turnError);
     drive.runVelocity(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      if(interrupted != true){
        System.out.println("  END: " + this.getClass().getSimpleName());
      }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distancePid.atSetpoint();
  }

  /**
   * Sets the max speed for driving straight
   *
   * @param maxSpeed max speed in meters/second. 0 indicates no speed limit.
   */
  protected void setMaxSpeed(double maxSpeed) {
    this.maxSpeed = maxSpeed;
  }
}
