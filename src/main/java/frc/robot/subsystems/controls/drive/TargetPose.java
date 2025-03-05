package frc.robot.subsystems.controls.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public enum TargetPose {
  // NOTE: WE ARE IDENTIFYING THESE POSITIONS ON THE BLUE SIDE OF THE FIELD!!!
  // THEN, DEPENDING ON THE ALLIANCE GIVEN TO THE DRIVER STATION,
  // PATHPLANNER WILL DECIDE WHETHER THE DESTINATION POSE IS
  // ON THE RED OR BLUE SIDE OF THE FIELD
  ORIGIN(0, "O", "Origin", new Pose2d(0.0, 0.0, new Rotation2d(0)), 0),
  FEEDER_R(1, "FR", "Feeder Right", new Pose2d(1.05, 1, new Rotation2d(50)), 0),
  FEEDER_L(2, "FL", "Feeder Left", new Pose2d(1.05, 7, Rotation2d.fromDegrees(-50)), 0),
  REEF_A(3, "A", "Reef A", new Pose2d(3.25, 4.05, Rotation2d.fromDegrees(0)), -1),
  REEF_G(9, "G", "Reef G", new Pose2d(5.5, 3.95, Rotation2d.fromDegrees(180)), -1),
  PROCESSOR(15, "P", "Processor", new Pose2d(6, 0.75, new Rotation2d(270)), 0);

  private int index;
  private String shortName;
  private String longName;

  private Pose2d pose;
  private Pose2d prepPose;
  private Pose2d endPose;

  // NOTE differential poses ~= 0.5m away from target

  public int getIndex() {
    return this.index;
  }

  public String getShortName() {
    return this.shortName;
  }

  public String getLongName() {
    return this.longName;
  }

  public Pose2d getPose() {
    return this.pose;
  }

  public Pose2d getPrepPose() {
    return this.prepPose;
  }

  public Pose2d getEndPose(boolean isRed) {
    return this.endPose;
  }

  private TargetPose(int idx, String sName, String lName, Pose2d targetPose, int reefPosition) {
    this.index = idx;
    this.shortName = sName;
    this.longName = lName;
    this.pose = targetPose;

    // TODO handle error case of accessing prep and end poses on non-reef position

    // If this pose is not on the reef, don't set extra poses
    if (reefPosition == 0) {
      this.prepPose = null;
      this.endPose = null;
    } else {
      double distance = (reefPosition < 0) ? -0.5 : 0.5;
      this.prepPose =
          targetPose.transformBy(new Transform2d(new Translation2d(0, distance), new Rotation2d()));
      this.endPose =
          targetPose.transformBy(new Transform2d(new Translation2d(-0.5, 0), new Rotation2d()));
    }
  }
}
