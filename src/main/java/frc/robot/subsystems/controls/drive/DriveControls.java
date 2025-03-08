package frc.robot.subsystems.controls.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.arm.ArmToPosition;
import frc.robot.commands.common.drive.DriveCommand;
import frc.robot.subsystems.controls.combination.DriverControls;
import frc.robot.subsystems.interfaces.Arm;
import frc.robot.subsystems.interfaces.Drive;
import frc.robot.subsystems.interfaces.Elevator;

import java.util.Map;
import java.util.Map.Entry;

public class DriveControls {

  protected static int myCoolPoseKeyIdx = TargetPose.REEF_A.getIndex();

  protected static int coolestNumberEver = 0;

  protected static final String[] orderedReefPositions = {
    "E", "F", "G", "H", "I", "J", "K", "L", "A", "B", "C", "D"
  };

  public static void setupController(Drive drive, Elevator elevator, Arm arm, CommandXboxController controller) {
    SubsystemBase driveSubsystem = (SubsystemBase) drive;
    driveSubsystem.setDefaultCommand(
        new DriveCommand(
            drive,
            () -> MathUtil.applyDeadband(-controller.getLeftY(), 0.05), // Robot Strafe Front/Back
            () -> MathUtil.applyDeadband(-controller.getLeftX(), 0.05), // Robot Strafe Left/Right
            () -> MathUtil.applyDeadband(-controller.getRightX(), 0.05))); // Robot Rotate

    /* Debug/Test Only:
     *    Back Button = Zero Pose
     *    Start Button = Toggle Drive Orientation
     */
    controller.back().onTrue(new InstantCommand(() -> drive.resetOdometry()));
    controller
        .start()
        .onTrue(
            new InstantCommand(
                () ->
                    drive.setFieldOrientedDrive(
                        !drive.isFieldOrientedDrive()))); // Toggle Drive Orientation

    // BLUE
    // // Define destinations for our "dynamic go-to-pose" functionality
    // Pose2d poseOrigin = new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    //     poseFeeder1 = new Pose2d(1.05, 7, Rotation2d.fromDegrees(130)),
    //     poseFeeder2 = new Pose2d(1.05, 1, Rotation2d.fromDegrees(230)),
    //     poseProcessor = new Pose2d(6, 0.75, Rotation2d.fromDegrees(270)),
    //     poseReefA = new Pose2d(3.25, 4.05, Rotation2d.fromDegrees(0)),
    //     poseReefG = new Pose2d(5.5, 3.95, Rotation2d.fromDegrees(180));

    // Define destinations for our "dynamic go-to-pose" functionality
    Pose2d poseOrigin = new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        poseFeeder1 = new Pose2d(16.44, 7.25, Rotation2d.fromDegrees(230)),
        poseFeeder2 = new Pose2d(16.02, 1, Rotation2d.fromDegrees(-230)),
        poseProcessor = new Pose2d(11.5, 7.3, Rotation2d.fromDegrees(90)),
        poseReefA = new Pose2d(15, 4.175, Rotation2d.fromDegrees(180)),
        poseReefAClose = new Pose2d(14.425, 4.175, Rotation2d.fromDegrees(180)),
        poseReefG = new Pose2d(11.57, 4.17, Rotation2d.fromDegrees(0)),
        poseReefGClose = new Pose2d(11.5, 4.175, Rotation2d.fromDegrees(0));
    //  PathConstraints constraints = new PathConstraints(4.9672, 9.3664784, 2 * Math.PI, 4 *
    //  Math.PI);
    // PathConstraints constraints = new PathConstraints(2, 1.5, 2 * Math.PI, 4 * Math.PI);
    // PathConstraints constraints = new PathConstraints(0.5, 4.5, Math.PI / 4, 4 * Math.PI);
    PathConstraints constraints =
        new PathConstraints(
            drive.getMaxLinearSpeed(), 1.5, drive.getMaxAngularSpeed(), Math.PI / 4);

    // Temporary UI to allow user to modify destination on-the-fly
    SendableChooser<TargetPose> chooser = new SendableChooser<>();
    // chooser.setDefaultOption("Origin", TargetPose.ORIGIN);
    // chooser.addOption("Feeder Left", TargetPose.FEEDER_L);
    // chooser.addOption("Feeder Right", TargetPose.FEEDER_R);
    // chooser.addOption("Processor", TargetPose.PROCESSOR);
    chooser.setDefaultOption("Reef A", TargetPose.REEF_A);
    // chooser.addOption("Reef A", TargetPose.REEF_A);
    chooser.addOption("Reef B", TargetPose.REEF_B);
    chooser.addOption("Reef C", TargetPose.REEF_C);
    chooser.addOption("Reef D", TargetPose.REEF_D);
    chooser.addOption("Reef E", TargetPose.REEF_E);
    chooser.addOption("Reef F", TargetPose.REEF_F);
    chooser.addOption("Reef G", TargetPose.REEF_G);
    chooser.addOption("Reef H", TargetPose.REEF_H);
    chooser.addOption("Reef I", TargetPose.REEF_I);
    chooser.addOption("Reef J", TargetPose.REEF_J);
    chooser.addOption("Reef K", TargetPose.REEF_K);
    chooser.addOption("Reef L", TargetPose.REEF_L);
    SmartDashboard.putData("Pose choices", chooser);

    // Define behavior for chosing destination of on-the-fly pose
    SmartDashboard.putNumber("Chosen Pose Index", myCoolPoseKeyIdx);
    chooser.onChange(
        (chosenPose) -> {
          myCoolPoseKeyIdx = chosenPose.getIndex();
          SmartDashboard.putNumber("Chosen Pose Index", myCoolPoseKeyIdx);
        });
    // controller
    //     .x()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               if (++myCoolPoseKeyIdx == TargetPoseOption.values().length) myCoolPoseKeyIdx =
    // 1;
    //               SmartDashboard.putNumber("Chosen Pose Index", myCoolPoseKeyIdx);
    //             }));

    // Define command to go to specific pose
    Command coolGoToPose =
        new SelectCommand<>(
            Map.ofEntries(
                coolDynamicPathScoringCommand(TargetPose.REEF_A, constraints, DriverControls.getPrepareToScoreCommand(elevator, arm), new ArmToPosition(arm, () -> 0)),
                coolDynamicPathScoringCommand(TargetPose.REEF_B, constraints, DriverControls.getPrepareToScoreCommand(elevator, arm), new ArmToPosition(arm, () -> 0)),
                coolDynamicPathScoringCommand(TargetPose.REEF_C, constraints, DriverControls.getPrepareToScoreCommand(elevator, arm), new ArmToPosition(arm, () -> 0)),
                coolDynamicPathScoringCommand(TargetPose.REEF_D, constraints, DriverControls.getPrepareToScoreCommand(elevator, arm), new ArmToPosition(arm, () -> 0)),
                coolDynamicPathScoringCommand(TargetPose.REEF_E, constraints, DriverControls.getPrepareToScoreCommand(elevator, arm), new ArmToPosition(arm, () -> 0)),
                coolDynamicPathScoringCommand(TargetPose.REEF_F, constraints, DriverControls.getPrepareToScoreCommand(elevator, arm), new ArmToPosition(arm, () -> 0)),
                coolDynamicPathScoringCommand(TargetPose.REEF_G, constraints, DriverControls.getPrepareToScoreCommand(elevator, arm), new ArmToPosition(arm, () -> 0)),
                coolDynamicPathScoringCommand(TargetPose.REEF_H, constraints, DriverControls.getPrepareToScoreCommand(elevator, arm), new ArmToPosition(arm, () -> 0)),
                coolDynamicPathScoringCommand(TargetPose.REEF_I, constraints, DriverControls.getPrepareToScoreCommand(elevator, arm), new ArmToPosition(arm, () -> 0)),
                coolDynamicPathScoringCommand(TargetPose.REEF_J, constraints, DriverControls.getPrepareToScoreCommand(elevator, arm), new ArmToPosition(arm, () -> 0)),
                coolDynamicPathScoringCommand(TargetPose.REEF_K, constraints, DriverControls.getPrepareToScoreCommand(elevator, arm), new ArmToPosition(arm, () -> 0)),
                coolDynamicPathScoringCommand(TargetPose.REEF_L, constraints, DriverControls.getPrepareToScoreCommand(elevator, arm), new ArmToPosition(arm, () -> 0))
            ),
            () -> {
              return myCoolPoseKeyIdx;
            });

    // TODO: new dynamic path planning command creation
    // 1. SelectCommand ONE: go to initial location (based on user-chosen option X)
    // 2. SelectCommand TWO: determine next action (based on option X and potentially pre-decided
    // reef elevation value)
    // 2a. if (1) was a reef position, assume we are scoring --> get reef elevation to score on -->
    // set elevator/arm
    // 2b. if not, choose an empty command and short-circuit (exit) the sequence
    // 3. SelectCommand THREE: go to scoring position (which maps directly from initial chosen
    // location)
    // 4. Execute <arm down command>

    // dynamically go to destination
    controller.rightTrigger().whileTrue(coolGoToPose);

    // TEMP FUNCTION TO TEST FIELD FLIPPING
    // controller.b().whileTrue(new SequentialCommandGroup(
    //     AutoBuilder.pathfindToPoseFlipped(TargetPose.REEF_A.getMyPrepPose(), constraints, 0.0),
    //     // DriverControls.Constants.prepareScoreCommand,
    //     AutoBuilder.pathfindToPoseFlipped(TargetPose.REEF_A.getMyPose(), constraints, 0.0),
    //     new ArmToPosition(arm, () -> 0)));

    /*  Angles -> Reef positions
     * 0    30  : E
     * 30   60  : F
     * 60   90  : G
     * 90   120 : H
     * 120  150 : I
     * 150  180 : J
     * 180  210 : K
     * 210  240 : L
     * 240  270 : A
     * 270  300 : B
     * 300  330 : C
     * 330  360 : D
     */
  }

  public static void setupAssistantController(Drive drive, CommandXboxController controller) {
    controller
        .x()
        .whileTrue(
            new RunCommand(
                () -> {
                  double myX = controller.getLeftX();
                  double myY = -controller.getLeftY();

                  // TODO maybe keep calculation in radians?
                  double myNumber = Math.atan2(myY, myX) * (180 / Math.PI);
                  if (myNumber < 0) myNumber += 360; // obtain this angle as a positive number

                  // This number can be used to index into the ordered reef positions array!
                  coolestNumberEver = (int) myNumber / 30;

                  // TODO remove Smartdashboard number; only display reef position
                  SmartDashboard.putNumber("AAAAAA", coolestNumberEver);
                  SmartDashboard.putString(
                      "AAAAAA Reef Position", orderedReefPositions[coolestNumberEver]);
                }));
  }

  private static Entry<Integer, Command> coolDynamicPathScoringCommand(
      TargetPose target,
      PathConstraints constraints,
      Command prepareToScoreCommand,
      Command scoreCommand) {
    // FIXME initialize a fresh instance of the prepareToScoreCommand each time!!
    // OTHERWISE CODE WILL CRASH
    Entry<Integer, Command> entry =
        Map.entry(
            target.getIndex(),
            new SequentialCommandGroup(
                AutoBuilder.pathfindToPoseFlipped(target.getPrepPose(), constraints, 0.0),
                prepareToScoreCommand,
                AutoBuilder.pathfindToPoseFlipped(target.getPose(), constraints, 0.0),
                scoreCommand));
    return entry;
  }
}
