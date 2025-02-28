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
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.drive.DriveCommand;
import frc.robot.subsystems.interfaces.Drive;
import java.util.Map;

public class DriveControls {
  private enum TargetPoseOption {
    ORIGIN(0),
    FEEDER_1(1),
    FEEDER_2(2),
    PROCESSOR(3),
    REEF_A(4),
    REEF_G(5),
    REEF_C(6),
    REEF_D(7),
    WOW_Test(8),
    WOW_Test1(9);

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

  public static void setupController(Drive drive, CommandXboxController controller) {
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
        poseFeeder2 = new Pose2d(16.02, 1, Rotation2d.fromDegrees(-50)),
        poseProcessor = new Pose2d(11.5, 7.3, Rotation2d.fromDegrees(90)),
        poseReefA = new Pose2d(15, 4.175, Rotation2d.fromDegrees(180)),
        poseReefAClose = new Pose2d(14.425, 4.175, Rotation2d.fromDegrees(180)),
        poseReefG = new Pose2d(11, 4.175, Rotation2d.fromDegrees(0)),
        poseReefGClose = new Pose2d(11.5, 4.175, Rotation2d.fromDegrees(0)),
        poseReefC = new Pose2d(14.17, 5.25, Rotation2d.fromDegrees(-124)),
        poseReefD = new Pose2d(13.63, 5.56, Rotation2d.fromDegrees(-124)),
        poseTest = new Pose2d(2.50, 5.57, Rotation2d.fromDegrees(-124)),
        poseTest2 = new Pose2d(2.46, 2.73, Rotation2d.fromDegrees(-124));
    //  PathConstraints constraints = new PathConstraints(4.9672, 9.3664784, 2 * Math.PI, 4 *
    //  Math.PI);
    // PathConstraints constraints = new PathConstraints(2, 1.5, 2 * Math.PI, 4 * Math.PI);
    PathConstraints constraints = new PathConstraints(2, 1.5, Math.PI / 2, Math.PI / 4);
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
    chooser.addOption("Test", TargetPoseOption.WOW_Test);
    chooser.addOption("Test1", TargetPoseOption.WOW_Test1);
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
                  if (++myCoolPoseKeyIdx == TargetPoseOption.values().length) myCoolPoseKeyIdx = 1;
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
                    AutoBuilder.pathfindToPose(poseReefC, constraints, 0.0)
                        .andThen(AutoBuilder.pathfindToPose(poseReefC, constraints, 0.0))),
                Map.entry(
                    TargetPoseOption.REEF_D.getIndex(),
                    AutoBuilder.pathfindToPose(poseReefD, constraints, 0.0)
                        .andThen(AutoBuilder.pathfindToPose(poseReefD, constraints, 0.0))),
                Map.entry(
                    TargetPoseOption.WOW_Test.getIndex(),
                    AutoBuilder.pathfindToPose(poseTest, constraints, 0.0)
                        .andThen(AutoBuilder.pathfindToPose(poseTest, constraints, 0.0))),
                Map.entry(
                    TargetPoseOption.WOW_Test1.getIndex(),
                    AutoBuilder.pathfindToPose(poseTest2, constraints, 0.0)
                        .andThen(AutoBuilder.pathfindToPose(poseTest2, constraints, 0.0)))),
            () -> {
              return myCoolPoseKeyIdx;
            });
    // ), () -> { return chooser.getSelected(); });

    // dynamically go to destination
    controller.b().whileTrue(coolGoToPose);
  }
}
