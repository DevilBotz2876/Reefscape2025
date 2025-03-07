package frc.robot.subsystems.controls.combination;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.arm.ArmToPosition;
import frc.robot.commands.common.elevator.ElevatorToPosition;
import frc.robot.subsystems.interfaces.Arm;
import frc.robot.subsystems.interfaces.Elevator;
import frc.robot.subsystems.interfaces.SimpleMotor;

public class DriverAssistControls {
  public static void setupController(
      Elevator elevator, Arm coralArm, SimpleMotor climber, CommandXboxController controller) {
    SmartDashboard.putData(
        "Driver " + "/Commands/Prepare For Intake",
        new SequentialCommandGroup(
            new ArmToPosition(coralArm, () -> -90).withTimeout(0),
            new ElevatorToPosition(elevator, () -> 0.8)));

    SmartDashboard.putData(
        "Driver " + "/Commands/Intake",
        new SequentialCommandGroup(new ElevatorToPosition(elevator, () -> 0.3)));

    SmartDashboard.putData(
        "Driver " + "/Commands/Score",
        new SequentialCommandGroup(new ArmToPosition(coralArm, () -> 0)));

    // Do not work
    SmartDashboard.putData(
        "Driver " + "/Commands/Prepare To Remove Algae L2",
        new SequentialCommandGroup(
            new ElevatorToPosition(elevator, () -> 1.2),
            new ArmToPosition(coralArm, () -> 0),
            new ElevatorToPosition(elevator, () -> 1.3)));

    // Do not work
    SmartDashboard.putData(
        "Driver " + "/Commands/Prepare To Remove Algae L3",
        new SequentialCommandGroup(
            new ElevatorToPosition(elevator, () -> 1.4),
            new ArmToPosition(coralArm, () -> 0),
            new ElevatorToPosition(elevator, () -> 1.3)));

    Command prepareIntakeCoralCommand =
        new SequentialCommandGroup(
            new ArmToPosition(coralArm, () -> -90).withTimeout(0),
            new ElevatorToPosition(elevator, () -> 0.8));
    controller
        .b()
        .onTrue(
            prepareIntakeCoralCommand.andThen(
                new InstantCommand(
                    () -> {
                      DriverControls.Constants.prepareScoreSelctedIndex = 1;
                      SmartDashboard.putNumber(
                          "Driver " + "/Misc/Prepare Selection",
                          DriverControls.Constants.prepareScoreSelctedIndex);
                    })));

    controller
        .y()
        .onTrue(
            new InstantCommand(
                () -> {
                  DriverControls.Constants.prepareScoreSelctedIndex = 4;
                  SmartDashboard.putNumber(
                      "Driver " + "/Misc/Prepare Selection",
                      DriverControls.Constants.prepareScoreSelctedIndex);
                }));

    controller
        .x()
        .onTrue(
            new InstantCommand(
                () -> {
                  DriverControls.Constants.prepareScoreSelctedIndex = 3;
                  SmartDashboard.putNumber(
                      "Driver " + "/Misc/Prepare Selection",
                      DriverControls.Constants.prepareScoreSelctedIndex);
                }));

    controller
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  DriverControls.Constants.prepareScoreSelctedIndex = 2;
                  SmartDashboard.putNumber(
                      "Driver " + "/Misc/Prepare Selection",
                      DriverControls.Constants.prepareScoreSelctedIndex);
                }));

    // controller
    //     .rightTrigger()
    //     .onTrue(
    //         new MotorAutoResetEncoderCommand(
    //             (Motor) coralArm, CoralArmControls.Constants.autoZeroSettings));

    // climber
    SubsystemBase climberSubsystem = (SubsystemBase) climber;
    // prepare to climb
    controller
        .leftBumper()
        .onTrue(
            new InstantCommand(
                () -> climber.setTargetPosition(climber.getSettings().maxPositionInRads),
                climberSubsystem));

    // climb
    controller
        .rightBumper()
        .onTrue(
            new InstantCommand(
                () -> climber.setTargetPosition(climber.getSettings().minPositionInRads),
                climberSubsystem));
  }
}
