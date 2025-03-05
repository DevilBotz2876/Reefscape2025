package frc.robot.subsystems.controls.combination;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.arm.ArmToPosition;
import frc.robot.commands.common.elevator.ElevatorToPosition;
import frc.robot.subsystems.controls.arm.CoralArmControls;
import frc.robot.subsystems.interfaces.Arm;
import frc.robot.subsystems.interfaces.Elevator;

public class DriverAssistControls {
  public static void setupController(Elevator elevator, Arm arm, CommandXboxController controller) {
    SmartDashboard.putData(
        "Driver " + "/Commands/Prepare For Intake",
        new SequentialCommandGroup(
            new ArmToPosition(arm, () -> -90).withTimeout(0),
            new ElevatorToPosition(elevator, () -> 0.8)));

    SmartDashboard.putData(
        "Driver " + "/Commands/Intake",
        new SequentialCommandGroup(new ElevatorToPosition(elevator, () -> 0.3)));

    SmartDashboard.putData(
        "Driver " + "/Commands/Score", new SequentialCommandGroup(new ArmToPosition(arm, () -> 0)));

    // Do not work
    SmartDashboard.putData(
        "Driver " + "/Commands/Prepare To Remove Algae L2",
        new SequentialCommandGroup(
            new ElevatorToPosition(elevator, () -> 1.2),
            new ArmToPosition(arm, () -> 0),
            new ElevatorToPosition(elevator, () -> 1.3)));

    // Do not work
    SmartDashboard.putData(
        "Driver " + "/Commands/Prepare To Remove Algae L3",
        new SequentialCommandGroup(
            new ElevatorToPosition(elevator, () -> 1.4),
            new ArmToPosition(arm, () -> 0),
            new ElevatorToPosition(elevator, () -> 1.3)));

    controller
        .y()
        .onTrue(
            new InstantCommand(
                () -> {
                  // DriverControls.Constants.prepareScoreChooser.close();
                  DriverControls.Constants.prepareScoreSelctedIndex = 4;
                  SmartDashboard.putNumber(
                      "Driver " + "/Commands/Prepare To Score Selection",
                      DriverControls.Constants.prepareScoreSelctedIndex);
                }));

    controller
        .x()
        .onTrue(
            new InstantCommand(
                () -> {
                  // DriverControls.Constants.prepareScoreChooser.close();
                  DriverControls.Constants.prepareScoreSelctedIndex = 3;
                  SmartDashboard.putNumber(
                      "Driver " + "/Commands/Prepare To Score Selection",
                      DriverControls.Constants.prepareScoreSelctedIndex);
                }));

    controller
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  // DriverControls.Constants.prepareScoreChooser.close();
                  DriverControls.Constants.prepareScoreSelctedIndex = 2;
                  SmartDashboard.putNumber(
                      "Driver " + "/Commands/Prepare To Score Selection",
                      DriverControls.Constants.prepareScoreSelctedIndex);
                }));

    DriverControls.Constants.prepareScoreChooser.onChange(
        (index) -> {
          DriverControls.Constants.prepareScoreSelctedIndex = index;
          SmartDashboard.putNumber("Driver " + "/Commands/Prepare To Score Selection", index);
        });

    // controller.rightBumper().onTrue(CoralArmControls.Constants.autoCalibrateCommand.unless(() -> (CoralArmControls.Constants.autoCalibrateCommand == null)));
  }
}
