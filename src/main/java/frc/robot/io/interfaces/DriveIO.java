package frc.robot.io.interfaces;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;
import swervelib.SwerveDrive;

public class DriveIO {
  @AutoLog
  public static class DriveIOInputs {
    public Pose2d pose;
    public double poseX = 0.0;
    public double poseY = 0.0;
    public double poseRotInDegrees = 0.0;
    public Pose2d flippedPose;
    public double flippedPoseX = 0.0;
    public double flippedPoseY = 0.0;
    public double flippedPoseRotInDegrees = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(DriveIOInputs inputs, SwerveDrive swerveDrive) {
    inputs.pose = swerveDrive.getPose();
    inputs.poseX = inputs.pose.getTranslation().getX();
    inputs.poseY = inputs.pose.getTranslation().getY();
    inputs.poseRotInDegrees = inputs.pose.getRotation().getDegrees();
    inputs.flippedPose = FlippingUtil.flipFieldPose(inputs.pose);
    inputs.flippedPoseX = inputs.pose.getTranslation().getX();
    inputs.flippedPoseY = inputs.pose.getTranslation().getY();
    inputs.flippedPoseRotInDegrees = inputs.pose.getRotation().getDegrees();
  }
  // Other methods for controlling the drive subsystem...
}
