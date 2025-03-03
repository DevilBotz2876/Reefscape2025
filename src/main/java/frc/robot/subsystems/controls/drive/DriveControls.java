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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.arm.ArmToPosition;
import frc.robot.commands.common.drive.DriveCommand;
import frc.robot.commands.common.elevator.ElevatorToPosition;
import frc.robot.subsystems.interfaces.Arm;
import frc.robot.subsystems.interfaces.Drive;
import frc.robot.subsystems.interfaces.Elevator;
import java.util.Map;

public class DriveControls {
  private enum TargetPoseOption {
    ORIGIN(0),
    FEEDER_1(1),
    FEEDER_2(2),
    PROCESSOR(3),
    REEF_A(4),
    REEF_G(10),
    REEF_C(6),
    REEF_D(7),
    REEF_L(15),
    // WOW_Test(9),
    // WOW_Test1(10),
    REEF_B(5),
    REEF_E(8),
    REEF_F(9),
    REEF_H(11),
    REEF_I(12),
    REEF_J(13),
    REEF_K(14);

    private int index;

    public int getIndex() {
      return this.index;
    }

    // public
    private TargetPoseOption(int idx) {
      this.index = idx;
    }
  }

  protected static int myCoolPoseKeyIdx = 0;

  public static void setupController(
      Drive drive, Elevator elevator, Arm arm, CommandXboxController controller) {
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
        poseReefGClose = new Pose2d(11.5, 4.175, Rotation2d.fromDegrees(0)),
        poseReefC = new Pose2d(14.17, 5.25, Rotation2d.fromDegrees(-124)),
        poseReefD = new Pose2d(13.63, 5.56, Rotation2d.fromDegrees(-124)),
        poseReefL = new Pose2d(14.14, 2.80, Rotation2d.fromDegrees(124)),
        // poseTest = new Pose2d(2.50, 5.57, Rotation2d.fromDegrees(-124)),
        // poseTest2 = new Pose2d(2.46, 2.73, Rotation2d.fromDegrees(-124)),
        poseReefB = new Pose2d(14.59, 4.34, Rotation2d.fromDegrees(180)),
        poseReefE = new Pose2d(12.46, 5.52, Rotation2d.fromDegrees(-60)),
        poseReefF = new Pose2d(12.00, 5.25, Rotation2d.fromDegrees(-60)),
        poseReefHStart = new Pose2d(11.00, 3.92, Rotation2d.fromDegrees(-1)),
        poseReefHEnd = new Pose2d(11.76, 3.92, Rotation2d.fromDegrees(-1)),
        poseReefI = new Pose2d(12.00, 2.75, Rotation2d.fromDegrees(60)),
        poseReefJ = new Pose2d(12.50, 2.50, Rotation2d.fromDegrees(60)),
        poseReefK = new Pose2d(13.65, 2.50, Rotation2d.fromDegrees(120));
    //  PathConstraints constraints = new PathConstraints(4.9672, 9.3664784, 2 * Math.PI, 4 *
    //  Math.PI);
    // PathConstraints constraints = new PathConstraints(2, 1.5, 2 * Math.PI, 4 * Math.PI);
    // PathConstraints constraints = new PathConstraints(4.5, 1.5, 2 * Math.PI, Math.PI / 4);
    PathConstraints constraints =
        new PathConstraints(
            drive.getMaxLinearSpeed(), 1.5, drive.getMaxAngularSpeed(), Math.PI / 4);

    // PathConstraints constraints = new PathConstraints(0.5, 4.5, Math.PI / 4, 4 * Math.PI);

    // Temporary UI to allow user to modify destination on-the-fly
    SendableChooser<TargetPoseOption> chooser = new SendableChooser<>();
    chooser.setDefaultOption("Origin", TargetPoseOption.ORIGIN);
    chooser.addOption("Feeder 1", TargetPoseOption.FEEDER_1);
    chooser.addOption("Feeder 2", TargetPoseOption.FEEDER_2);
    chooser.addOption("Processor", TargetPoseOption.PROCESSOR);
    chooser.addOption("Reef A", TargetPoseOption.REEF_A);
    chooser.addOption("Reef G", TargetPoseOption.REEF_G);
    chooser.addOption("Reef C", TargetPoseOption.REEF_C);
    chooser.addOption("Reef D", TargetPoseOption.REEF_D);
    // chooser.addOption("Reef L", TargetPoseOption.REEF_L);
    // chooser.addOption("Test", TargetPoseOption.WOW_Test);
    // chooser.addOption("Test1", TargetPoseOption.WOW_Test1);
    chooser.addOption("Reef B", TargetPoseOption.REEF_B);
    chooser.addOption("Reef E", TargetPoseOption.REEF_E);
    chooser.addOption("Reef F", TargetPoseOption.REEF_F);
    chooser.addOption("Reef H", TargetPoseOption.REEF_H);
    chooser.addOption("Reef I", TargetPoseOption.REEF_I);
    chooser.addOption("Reef J", TargetPoseOption.REEF_J);
    chooser.addOption("Reef K", TargetPoseOption.REEF_K);
    SmartDashboard.putData("Pose choices", chooser);

    // Define behavior for chosing destination of on-the-fly pose
    SmartDashboard.putNumber("Chosen Pose Index", myCoolPoseKeyIdx);
    chooser.onChange(
        (chosenPose) -> {
          myCoolPoseKeyIdx = chosenPose.getIndex();
          SmartDashboard.putNumber("Chosen Pose Index", myCoolPoseKeyIdx);
        });
    controller
        .x()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (myCoolPoseKeyIdx == 11) myCoolPoseKeyIdx = 2;
                  else myCoolPoseKeyIdx = 11;
                  //   if (++myCoolPoseKeyIdx == TargetPoseOption.values().length) myCoolPoseKeyIdx
                  // = 1;
                  SmartDashboard.putNumber("Chosen Pose Index", myCoolPoseKeyIdx);
                }));

    // Define command to go to specific pose
    Command coolGoToPose =
        new SelectCommand<>(
            Map.ofEntries(
                Map.entry(
                    TargetPoseOption.ORIGIN.getIndex(),
                    AutoBuilder.pathfindToPose(poseOrigin, constraints, 0.0)),
                Map.entry(
                    TargetPoseOption.FEEDER_1.getIndex(),
                    AutoBuilder.pathfindToPose(poseFeeder1, constraints, 0.0)),
                Map.entry(
                    TargetPoseOption.FEEDER_2.getIndex(),
                    AutoBuilder.pathfindToPose(poseFeeder2, constraints, 0.0)),
                Map.entry(
                    TargetPoseOption.PROCESSOR.getIndex(),
                    AutoBuilder.pathfindToPose(poseProcessor, constraints, 0.0)),
                Map.entry(
                    TargetPoseOption.REEF_A.getIndex(),
                    AutoBuilder.pathfindToPose(poseReefAClose, constraints, 0.0)),
                // AutoBuilder.pathfindToPose(poseReefA, constraints, 0.0)
                //     .andThen(AutoBuilder.pathfindToPose(poseReefAClose, constraints, 0.0))),
                Map.entry(
                    TargetPoseOption.REEF_G.getIndex(),
                    AutoBuilder.pathfindToPose(poseReefG, constraints, 0.5)
                        .andThen(AutoBuilder.pathfindToPose(poseReefGClose, constraints, 0.0))),
                // AutoBuilder.pathfindToPose(poseReefGClose, constraints, 0.0)),
                Map.entry(
                    TargetPoseOption.REEF_C.getIndex(),
                    AutoBuilder.pathfindToPose(poseReefC, constraints, 0.0)),
                Map.entry(
                    TargetPoseOption.REEF_D.getIndex(),
                    AutoBuilder.pathfindToPose(poseReefD, constraints, 0.0)),
                Map.entry(
                    TargetPoseOption.REEF_L.getIndex(),
                    AutoBuilder.pathfindToPose(poseReefL, constraints, 0.0)),
                // Map.entry(
                //     TargetPoseOption.WOW_Test.getIndex(),
                //     AutoBuilder.pathfindToPose(poseTest, constraints, 0.0)
                //         .andThen(AutoBuilder.pathfindToPose(poseTest, constraints, 0.0))),
                // Map.entry(
                //     TargetPoseOption.WOW_Test1.getIndex(),
                //     AutoBuilder.pathfindToPose(poseTest2, constraints, 0.0)
                //         .andThen(AutoBuilder.pathfindToPose(poseTest2, constraints, 0.0))),
                Map.entry(
                    TargetPoseOption.REEF_B.getIndex(),
                    AutoBuilder.pathfindToPose(poseReefB, constraints, 0.0)),
                Map.entry(
                    TargetPoseOption.REEF_E.getIndex(),
                    AutoBuilder.pathfindToPose(poseReefE, constraints, 0.0)),
                Map.entry(
                    TargetPoseOption.REEF_F.getIndex(),
                    AutoBuilder.pathfindToPose(poseReefF, constraints, 0.0)),
                Map.entry(
                    TargetPoseOption.REEF_H.getIndex(),
                    AutoBuilder.pathfindToPose(poseReefHStart, constraints, 0.0)
                        .andThen(
                            new SequentialCommandGroup(
                                new ElevatorToPosition(elevator, () -> 0.6),
                                new ParallelCommandGroup(
                                    new ArmToPosition(arm, () -> 75).withTimeout(1),
                                    new ElevatorToPosition(elevator, () -> 1.553))))
                        .andThen(AutoBuilder.pathfindToPose(poseReefHEnd, constraints, 0.0))
                        .andThen(new ArmToPosition(arm, () -> 0))),
                Map.entry(
                    TargetPoseOption.REEF_I.getIndex(),
                    AutoBuilder.pathfindToPose(poseReefI, constraints, 0.0)),
                Map.entry(
                    TargetPoseOption.REEF_J.getIndex(),
                    AutoBuilder.pathfindToPose(poseReefJ, constraints, 0.0)),
                Map.entry(
                    TargetPoseOption.REEF_K.getIndex(),
                    AutoBuilder.pathfindToPose(poseReefK, constraints, 0.0))),
            () -> {
              return myCoolPoseKeyIdx;
            });
    // ), () -> { return chooser.getSelected(); });

    // dynamically go to destination
    controller.b().whileTrue(coolGoToPose);
  }
}
