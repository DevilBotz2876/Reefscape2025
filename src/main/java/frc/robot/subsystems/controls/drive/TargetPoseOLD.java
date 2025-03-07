package frc.robot.subsystems.controls.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public enum TargetPoseOLD {
  ORIGIN(
      0,
      "O",
      "Origin",
      new Pose2d(0.0, 0.0, new Rotation2d(0)),
      new Pose2d(0.0, 0.0, new Rotation2d(0)),
      0),
  FEEDER_R(
      1,
      "FR",
      "Feeder Right",
      new Pose2d(16.44, 7.25, new Rotation2d(230)),
      new Pose2d(1.05, 1, new Rotation2d(50)),
      0),
  FEEDER_L(
      2,
      "FL",
      "Feeder Left",
      new Pose2d(16.02, 1, Rotation2d.fromDegrees(-230)),
      new Pose2d(1.05, 7, Rotation2d.fromDegrees(-50)),
      0),
  REEF_A(
      3,
      "A",
      "Reef A",
      new Pose2d(14.425, 4.175, Rotation2d.fromDegrees(180)),
      new Pose2d(3.25, 4.05, Rotation2d.fromDegrees(0)),
      -1),
  REEF_G(
      9,
      "G",
      "Reef G",
      new Pose2d(11.5, 4.175, Rotation2d.fromDegrees(0)),
      new Pose2d(5.5, 3.95, Rotation2d.fromDegrees(180)),
      -1),
  PROCESSOR(
      15,
      "P",
      "Processor",
      new Pose2d(11.5, 7.3, new Rotation2d(90)),
      new Pose2d(6, 0.75, new Rotation2d(270)),
      0);

  private int index;
  private String shortName;
  private String longName;
  private Pose2d redPose;
  private Pose2d bluePose;

  private Pose2d redPrepPose;
  private Pose2d redEndPose;
  private Pose2d bluePrepPose;
  private Pose2d blueEndPose;

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

  public Pose2d getPose(boolean isRed) {
    return (isRed) ? this.redPose : this.bluePose;
  }

  public Pose2d getMyPose() {
    return this.bluePose;
  }

  public Pose2d getMyPrepPose() {
    return this.bluePrepPose;
  }

  public Pose2d getPrepPose(boolean isRed) {
    return (isRed) ? this.redPrepPose : this.bluePrepPose;
  }

  public Pose2d getEndPose(boolean isRed) {
    return (isRed) ? this.redEndPose : this.blueEndPose;
  }

  private void calcPrepPoses(boolean isReefStickLeft) {
    double distance = isReefStickLeft ? -0.5 : 0.5;
    this.redPrepPose =
        this.redPose.transformBy(new Transform2d(new Translation2d(0, distance), new Rotation2d()));
    this.bluePrepPose =
        this.bluePose.transformBy(
            new Transform2d(new Translation2d(0, distance), new Rotation2d()));
  }

  private void calcEndPoses() {
    this.redEndPose =
        this.redPose.transformBy(new Transform2d(new Translation2d(-0.5, 0), new Rotation2d()));
    this.blueEndPose =
        this.bluePose.transformBy(new Transform2d(new Translation2d(-0.5, 0), new Rotation2d()));
  }

  private TargetPoseOLD(
      int idx, String sName, String lName, Pose2d redPose, Pose2d bluePose, int reefPosition) {
    this.index = idx;
    this.shortName = sName;
    this.longName = lName;
    this.redPose = redPose;
    this.bluePose = bluePose;

    // If this pose is not on the reef, skip
    if (reefPosition == 0) return;

    calcPrepPoses(reefPosition < 0);
    calcEndPoses();
  }
}
