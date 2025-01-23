package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;

public interface Drive {
  public static class Constants {
    public static double maxVelocityMetersPerSec = 4.5;
    public static double maxAngularVelocityRadiansSec = 2 * Math.PI;

    public static double rotatePidKp = 0.05;
    public static double rotatePidKi = 0.0;
    public static double rotatePidKd = 0.0;
    public static double pidTimeoutInSeconds = 0.5;
  }

  /**
   * Sets the desired chassis speed
   *
   * @param velocity specifies the setpoint velocity for the chassis
   */
  public void runVelocity(ChassisSpeeds velocity);

  /**
   * This returns the robot's max linear speed in meter/sec
   *
   * @return speed in meter/sec
   */
  public double getMaxLinearSpeed();

  /**
   * This returns the robot's max angular speed in radian/sec
   *
   * @return speed in radian/sec
   */
  public double getMaxAngularSpeed();

  public default void setFieldOrientedDrive(boolean enable) {}
  ;

  public default boolean isFieldOrientedDrive() {
    return false;
  }

  public default void resetOdometry() {}
  ;

  public default void setPoseToMatchField() {}

  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public default Command sysIdDriveMotorCommand() {
    return null;
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public default Command sysIdAngleMotorCommand() {
    return null;
  }

  public default Pose2d getPose() {
    return new Pose2d();
  }

  public default double getAngle() {
    return 0;
  }

  public default void lockPose() {}
  ;

  public default void addVisionMeasurement(
      Pose2d robotPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {}
}
