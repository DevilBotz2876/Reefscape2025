package frc.robot.subsystems.controls.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.arm.ArmToPosition;
import frc.robot.commands.common.drive.DriveCommand;
import frc.robot.commands.common.drive.DriveToPositionX;
import frc.robot.commands.common.drive.DriveToYaw;
import frc.robot.commands.common.elevator.ElevatorToPosition;
import frc.robot.subsystems.controls.combination.DriverControls;
import frc.robot.subsystems.interfaces.Arm;
import frc.robot.subsystems.interfaces.Drive;
import frc.robot.subsystems.interfaces.Elevator;
import frc.robot.util.DevilBotState;
import java.util.Map;
import java.util.Map.Entry;
import org.littletonrobotics.junction.Logger;

public class DriveControls {

  protected static TargetPose chosenTarget = TargetPose.REEF_A;
  static int value = 0;

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

    PathConstraints constraints =
        new PathConstraints(
            drive.getMaxLinearSpeed(), 1.5, drive.getMaxAngularSpeed(), Math.PI / 4);

    SmartDashboard.putData("Go Forward 1 Meter", new DriveToPositionX(drive, () -> 1.0));

    SmartDashboard.putData(
        "Go Forward 1 Meter and Turn 180",
        new SequentialCommandGroup(
            new DriveToPositionX(drive, () -> 1.0), new DriveToYaw(drive, () -> 180.0)));

    SmartDashboard.putData(
        "Go in a Square Path",
        new SequentialCommandGroup(
            new DriveToPositionX(drive, () -> 1.0),
            new WaitCommand(0.5),
            new DriveToYaw(drive, () -> 90).withTimeout(2),
            new WaitCommand(0.5),
            new DriveToPositionX(drive, () -> 1.0),
            new WaitCommand(0.5),
            new DriveToYaw(drive, () -> 180).withTimeout(2),
            new WaitCommand(0.5),
            new DriveToPositionX(drive, () -> 1.0),
            new WaitCommand(0.5),
            new DriveToYaw(drive, () -> -90).withTimeout(2),
            new WaitCommand(0.5),
            new DriveToPositionX(drive, () -> 1.0),
            new WaitCommand(0.5),
            new DriveToYaw(drive, () -> 0).withTimeout(2)));
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
    SmartDashboard.putNumber("Chosen Pose Index", chosenTarget.getIndex());
    SmartDashboard.putString("Chosen Reef Position", chosenTarget.getShortName());
    chooser.onChange(
        (chosenOption) -> {
          chosenTarget = chosenOption;
          SmartDashboard.putNumber("Chosen Pose Index", chosenTarget.getIndex());
          SmartDashboard.putString("Reef Position", chosenTarget.getShortName());
        });

    // Define command to go to specific pose
    Command coolGoToPose =
        new SelectCommand<>(
            Map.ofEntries(
                coolDynamicPathScoringCommand(
                    TargetPose.REEF_A,
                    constraints,
                    getPrepareToScoreCommand(elevator, arm),
                    new ArmToPosition(arm, () -> 0)),
                coolDynamicPathScoringCommand(
                    TargetPose.REEF_B,
                    constraints,
                    getPrepareToScoreCommand(elevator, arm),
                    new ArmToPosition(arm, () -> 0)),
                coolDynamicPathScoringCommand(
                    TargetPose.REEF_C,
                    constraints,
                    getPrepareToScoreCommand(elevator, arm),
                    new ArmToPosition(arm, () -> 0)),
                coolDynamicPathScoringCommand(
                    TargetPose.REEF_D,
                    constraints,
                    getPrepareToScoreCommand(elevator, arm),
                    new ArmToPosition(arm, () -> 0)),
                coolDynamicPathScoringCommand(
                    TargetPose.REEF_E,
                    constraints,
                    getPrepareToScoreCommand(elevator, arm),
                    new ArmToPosition(arm, () -> 0)),
                coolDynamicPathScoringCommand(
                    TargetPose.REEF_F,
                    constraints,
                    getPrepareToScoreCommand(elevator, arm),
                    new ArmToPosition(arm, () -> 0)),
                coolDynamicPathScoringCommand(
                    TargetPose.REEF_G,
                    constraints,
                    getPrepareToScoreCommand(elevator, arm),
                    new ArmToPosition(arm, () -> 0)),
                coolDynamicPathScoringCommand(
                    TargetPose.REEF_H,
                    constraints,
                    getPrepareToScoreCommand(elevator, arm),
                    new ArmToPosition(arm, () -> 0)),
                coolDynamicPathScoringCommand(
                    TargetPose.REEF_I,
                    constraints,
                    getPrepareToScoreCommand(elevator, arm),
                    new ArmToPosition(arm, () -> 0)),
                coolDynamicPathScoringCommand(
                    TargetPose.REEF_J,
                    constraints,
                    getPrepareToScoreCommand(elevator, arm),
                    new ArmToPosition(arm, () -> 0)),
                coolDynamicPathScoringCommand(
                    TargetPose.REEF_K,
                    constraints,
                    getPrepareToScoreCommand(elevator, arm),
                    new ArmToPosition(arm, () -> 0)),
                coolDynamicPathScoringCommand(
                    TargetPose.REEF_L,
                    constraints,
                    getPrepareToScoreCommand(elevator, arm),
                    new ArmToPosition(arm, () -> 0))),
            () -> {
              return chosenTarget.getIndex();
            });

    // dynamically go to destination
    controller.rightTrigger().whileTrue(coolGoToPose);

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
        .rightTrigger()
        .whileTrue(
            new RunCommand(
                () -> {
                  double myX = controller.getLeftX();
                  double myY = -controller.getLeftY();

                  // TODO maybe keep calculation in radians?
                  double myNumber = Math.atan2(myY, myX) * (180 / Math.PI);
                  if (myNumber < 0) myNumber += 360; // obtain this angle as a positive number

                  // Convert the joystick angle to a reef position
                  chosenTarget = TargetPose.getReefTargetWithAngle(myNumber);
                  SmartDashboard.putNumber("Chosen Pose Index", chosenTarget.getIndex());
                  SmartDashboard.putString("Chosen Reef Position", chosenTarget.getShortName());

                  // Log target pose (for debugging)
                  Pose2d p =
                      DevilBotState.isRedAlliance()
                          ? chosenTarget.getPose()
                          : FlippingUtil.flipFieldPose(chosenTarget.getPose());
                  Logger.recordOutput("DriveSwerveYAGSL/Chosen Reef Pose", p);
                  SmartDashboard.putNumber("Chosen Reef X", p.getX());
                  SmartDashboard.putNumber("Chosen Reef Y", p.getY());
                  SmartDashboard.putNumber("Chosen Reef Rot", p.getRotation().getDegrees());
                }));
  }

  private static Entry<Integer, Command> coolDynamicPathScoringCommand(
      TargetPose target,
      PathConstraints constraints,
      Command prepareToScoreCommand,
      Command scoreCommand) {
    return Map.entry(
        target.getIndex(),
        new SequentialCommandGroup(
            AutoBuilder.pathfindToPoseFlipped(target.getPrepPose(), constraints, 0.0),
            prepareToScoreCommand,
            new ConditionalCommand(
                AutoBuilder.pathfindToPoseFlipped(target.getPose(), constraints, 0.0),
                AutoBuilder.pathfindToPoseFlipped(target.getLowerScoringPose(), constraints, 0.0),
                () -> DriverControls.Constants.prepareScoreSelctedIndex < 4),
            scoreCommand));
  }

  private static Command getPrepareToScoreCommand(Elevator elevator, Arm coralArm) {
    return new SelectCommand<>(
        Map.ofEntries(
            Map.entry(
                1,
                new SequentialCommandGroup(
                        new ElevatorToPosition(elevator, () -> 0.63),
                        new ArmToPosition(coralArm, () -> 47))
                    .withTimeout(5.0)),
            Map.entry(
                2,
                new SequentialCommandGroup(
                        new ElevatorToPosition(elevator, () -> 0.63),
                        new ArmToPosition(coralArm, () -> 47))
                    .withTimeout(5.0)),
            Map.entry(
                3,
                new SequentialCommandGroup(
                        new ElevatorToPosition(elevator, () -> 1.0),
                        new ArmToPosition(coralArm, () -> 47))
                    .withTimeout(5.0)),
            Map.entry(
                4,
                new SequentialCommandGroup(
                        new ElevatorToPosition(elevator, () -> 0.6)
                            .unless(() -> elevator.getCurrentHeight() > 0.6),
                        new ParallelCommandGroup(
                            new ArmToPosition(coralArm, () -> 48).withTimeout(1.0),
                            new ElevatorToPosition(elevator, () -> 1.553)))
                    .withTimeout(5.0))),
        () -> {
          return DriverControls.Constants.prepareScoreSelctedIndex;
        });
  }
}
