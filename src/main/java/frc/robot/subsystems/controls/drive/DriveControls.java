package frc.robot.subsystems.controls.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.arm.ArmToPosition;
import frc.robot.commands.common.drive.DriveCommand;
import frc.robot.commands.common.elevator.ElevatorToPosition;
import frc.robot.subsystems.interfaces.Arm;
import frc.robot.subsystems.interfaces.Drive;
import frc.robot.subsystems.interfaces.Elevator;

public class DriveControls {
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
                () -> {
                  if (drive.isFieldOrientedDrive()) {
                    drive.setFieldOrientedDrive(false);
                  } else {
                    drive.setFieldOrientedDrive(true);
                  }
                })); // Toggle Drive Orientation

    SmartDashboard.putData(
        "Drive " + "/Commands/Prepare To Score L4",
        new SequentialCommandGroup(
            new ElevatorToPosition(elevator, () -> Units.inchesToMeters(55)),
            new ArmToPosition(arm, () -> 45)));
    SmartDashboard.putData(
        "Drive " + "/Commands/Score L4",
        new SequentialCommandGroup(new ArmToPosition(arm, () -> -40)));

    SmartDashboard.putData(
        "Drive " + "/Commands/Pick Up Coral",
        new SequentialCommandGroup(
            new ArmToPosition(arm, () -> -90).withTimeout(1.5),
            new ElevatorToPosition(elevator, () -> 0.1)));
  }
}
