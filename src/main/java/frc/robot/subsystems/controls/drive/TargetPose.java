package frc.robot.subsystems.controls.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public enum TargetPose {
  // NOTE: WE ARE IDENTIFYING THESE POSITIONS ON THE BLUE SIDE OF THE FIELD!!!
  // THEN, DEPENDING ON THE ALLIANCE GIVEN TO THE DRIVER STATION,
  // PATHPLANNER WILL DECIDE WHETHER THE DESTINATION POSE IS
  // ON THE RED OR BLUE SIDE OF THE FIELD
  ORIGIN(0, "O", "Origin", new Pose2d(0.0, 0.0, new Rotation2d(0)), 0),
  FEEDER_R(1, "FR", "Feeder Right", new Pose2d(1.05, 1, new Rotation2d(50)), 0),
  FEEDER_L(2, "FL", "Feeder Left", new Pose2d(1.05, 7, Rotation2d.fromDegrees(-50)), 0),
  REEF_A(3, "A", "Reef A", new Pose2d(3.14, 4.18, Rotation2d.fromDegrees(0)), -1),
  REEF_B(4, "B", "Reef B", new Pose2d(3.14, 3.79, Rotation2d.fromDegrees(0)), 1),
  REEF_C(5, "C", "Reef C", new Pose2d(3.71, 2.98, Rotation2d.fromDegrees(60)), -1),
  REEF_D(6, "D", "Reef D", new Pose2d(4.10, 2.76, Rotation2d.fromDegrees(60)), 1),
  REEF_E(7, "E", "Reef E", new Pose2d(5.00, 2.83, Rotation2d.fromDegrees(120)), -1),
  REEF_F(8, "F", "Reef F", new Pose2d(5.36, 3.00, Rotation2d.fromDegrees(122)), 1),
  REEF_G(9, "G", "Reef G", new Pose2d(5.71, 4.00, Rotation2d.fromDegrees(183)), -1),
  REEF_H(10, "H", "Reef H", new Pose2d(5.80, 4.20, Rotation2d.fromDegrees(180)), 1),
  REEF_I(11, "I", "Reef I", new Pose2d(5.28, 5.06, Rotation2d.fromDegrees(-120)), -1),
  REEF_J(12, "J", "Reef J", new Pose2d(4.97, 5.28, Rotation2d.fromDegrees(-120)), 1),
  REEF_K(13, "K", "Reef K", new Pose2d(3.95, 5.18, Rotation2d.fromDegrees(-60)), -1),
  REEF_L(14, "L", "Reef L", new Pose2d(3.69, 5.04, Rotation2d.fromDegrees(-60)), 1),
  PROCESSOR(15, "P", "Processor", new Pose2d(6, 0.75, new Rotation2d(270)), 0);

  /*
   *  Practice field testing
   * A: 3.13 4.18 0           14.36 3.88 180
   * B: 3.13 3.79 0           14.42 4.26 180
   * C: 3.71 2.98 59.6        13.83 5.07 -120.3
   * D: 4.10 2.76 60          13.45 5.29 -120
   * E: 5.00 2.83 120.0       12.55 5.23 -60
   * F: 5.36 3.00 122.5~      12.19 5.06 -57.7    ** this one potentially uncentered
   * G: 5.71 4.00 -177.23     11.84 4.07 2.77
   * H:
   * I: 5.28 5.06 -119.74     12.26 3.00 60.26
   * J: 4.97 5.28 -119.62     12.57 2.78 60.38
   * K: 3.95 5.18 -59         13.59 2.87 121
   * L: 3.69 5.04 -58.6~      13.86 3.01 121.3
   */

  /*
   *  During practice matches
   * A:
   * B:
   * C:
   * D:
   * E:
   * F:
   * G: 5.55 3.83 178.7       11.99 4.22 -1.30
   * H: 5.56 4.20 177.95      11.99 3.85 -3.05
   * I: 5.27 5.03 -125.37     12.28 3.02 54.63    ?
   * J: 4.71 5.14 -123.69     12.84 2.92 56.31    ??
   * K:
   * L? 3.49 4.61 -57.59      14.06 3.44 122.41   ??
   */

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
      double distance = (reefPosition < 0) ? -0.7 : 0.7;
      this.prepPose = targetPose.transformBy(new Transform2d(0, distance, new Rotation2d()));
      this.endPose = targetPose.transformBy(new Transform2d(-0.7, 0, new Rotation2d()));
    }
  }
}
