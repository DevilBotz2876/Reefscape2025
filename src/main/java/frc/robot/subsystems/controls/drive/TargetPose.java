package frc.robot.subsystems.controls.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public enum TargetPose {
  // NOTE: WE ARE IDENTIFYING THESE POSITIONS ON THE BLUE SIDE OF THE FIELD!!!
  // THEN, DEPENDING ON THE ALLIANCE GIVEN TO THE DRIVER STATION,
  // PATHPLANNER WILL DECIDE WHETHER THE DESTINATION POSE IS
  // ON THE RED OR BLUE SIDE OF THE FIELD

  ORIGIN(0, "O", "Origin", new Pose2d(0.0, 0.0, new Rotation2d(0))),
  FEEDER_R(1, "FR", "Feeder Right", new Pose2d(1.05, 1, new Rotation2d(50))),
  FEEDER_L(2, "FL", "Feeder Left", new Pose2d(1.05, 7, Rotation2d.fromDegrees(-50))),
  REEF_A(3, "A", "Reef A", 0.0, true),
  REEF_B(4, "B", "Reef B", 0.0, false),
  REEF_C(5, "C", "Reef C", 60.0, true),
  REEF_D(6, "D", "Reef D", 60.0, false),
  REEF_E(7, "E", "Reef E", 120.0, true),
  REEF_F(8, "F", "Reef F", 120.0, false),
  REEF_G(9, "G", "Reef G", 180.0, true),
  REEF_H(10, "H", "Reef H", 180.0, false),
  REEF_I(11, "I", "Reef I", 240.0, true),
  REEF_J(12, "J", "Reef J", 240.0, false),
  REEF_K(13, "K", "Reef K", 300.0, true),
  REEF_L(14, "L", "Reef L", 300.0, false),
  PROCESSOR(15, "P", "Processor", new Pose2d(6, 0.75, new Rotation2d(270)));

  // REMEMBER: actual distance between reef poles is 13 inches,
  // currently using 9 inches because robot currently overshoots target pose during traversal

  private final Pose2d blueReefCenter =
      new Pose2d(new Translation2d(4.485, 4.00), new Rotation2d());

  // NOTE: I pushed the target position closer to the reef to be more realistic to the actual game
  // the original position was x: 3.16, y: 3.82
  private final Pose2d blueReefBPos = new Pose2d(new Translation2d(3.175, 3.82), new Rotation2d());

  public static TargetPose getReefTargetWithAngle(double angleDegrees) {
    int reefPosIdx = (int) (angleDegrees / 30) + 7;
    if (reefPosIdx > 14) reefPosIdx -= 12;
    for (TargetPose x : values()) {
      if (x.getIndex() == reefPosIdx) return x;
    }
    throw new IllegalArgumentException(reefPosIdx + " does not translate to valid reef position.");
  }

  /*
   *  Home positions (blue)
   * A:
   * B: 3.16 3.80
   * C: 3.72 3.03 60
   * D:
   * E:
   * F:
   * G:
   * H:
   * I:
   * J:
   * K:
   * L:
   *
   *
   * camera
   * x -7
   * y 2
   * z 22.5in
   * 23.75
   * roll 0
   * pitch 21
   * yaw 10
   */

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
  private Pose2d lowerLevelPose;
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

  public static Pose2d getPosewWithIndex(int index) {
    for (TargetPose x : values()) {
      if (x.getIndex() == index) {
        return x.pose;
      }
    }
    return new Pose2d();
  }

  public Pose2d getPrepPose() {
    return this.prepPose;
  }

  public Pose2d getEndPose(boolean isRed) {
    return this.endPose;
  }

  public Pose2d getLowerScoringPose() {
    return this.lowerLevelPose;
  }

  private TargetPose(int idx, String sName, String lName, Pose2d targetPose) {
    this.index = idx;
    this.shortName = sName;
    this.longName = lName;
    this.pose = targetPose;

    // TODO handle error case of accessing prep and end poses on non-reef position

    this.lowerLevelPose = null;
    this.prepPose = null;
    this.endPose = null;
  }

  private TargetPose(
      int idx, String sName, String lName, double reefWallRotDegrees, boolean isLeftPosition) {
    this.index = idx;
    this.shortName = sName;
    this.longName = lName;

    this.pose =
        blueReefBPos.rotateAround(
            blueReefCenter.getTranslation(), Rotation2d.fromDegrees(reefWallRotDegrees));
    double distance = 0.6;
    double rot = -Units.degreesToRadians(25.0);
    if (isLeftPosition) {
      this.pose =
          this.pose.transformBy(new Transform2d(0, Units.inchesToMeters(13), new Rotation2d()));
      distance = -distance;
      rot = -rot;
    }
    this.prepPose = this.pose.transformBy(new Transform2d(-0.2, distance, new Rotation2d(rot)));
    this.lowerLevelPose = this.pose.transformBy(new Transform2d(0.04445, 0, new Rotation2d()));
    this.endPose = this.pose.transformBy(new Transform2d(-0.7, 0, new Rotation2d()));
  }
}
