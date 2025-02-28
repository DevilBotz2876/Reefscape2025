package frc.robot.subsystems.controls.combination;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.arm.ArmToPosition;
import frc.robot.commands.common.elevator.ElevatorToPosition;
import frc.robot.subsystems.interfaces.Arm;
import frc.robot.subsystems.interfaces.Elevator;

public class DriverAssistControls {
  public static void setupController(Elevator elevator, Arm arm, CommandXboxController controller) {

    SmartDashboard.putData(
        "Driver " + "/Commands/Prepare For Intake",
        new SequentialCommandGroup(
            new ArmToPosition(arm, () -> -90).withTimeout(0),
            new ElevatorToPosition(elevator, () -> 0.6)));

    SmartDashboard.putData(
        "Driver " + "/Commands/Intake",
        new SequentialCommandGroup(new ElevatorToPosition(elevator, () -> 0.2)));

    SmartDashboard.putData(
        "Driver " + "/Commands/Prepare To Score L4",
        new SequentialCommandGroup(
            new ElevatorToPosition(elevator, () -> 1.0),
            new ArmToPosition(arm, () -> 75),
            new ElevatorToPosition(elevator, () -> 1.3)));
    SmartDashboard.putData(
        "Driver " + "/Commands/Score L4",
        new SequentialCommandGroup(new ArmToPosition(arm, () -> 0)));

    SmartDashboard.putData(
        "Driver " + "/Commands/Prepare To Score L3",
        new SequentialCommandGroup(
            new ElevatorToPosition(elevator, () -> 0.8),
            new ArmToPosition(arm, () -> 75)));

    SmartDashboard.putData(
        "Driver " + "/Commands/Prepare To Score L2",
        new SequentialCommandGroup(
            new ElevatorToPosition(elevator, () -> 0.6),
            new ArmToPosition(arm, () -> 75)));

    SmartDashboard.putData(
        "Driver " + "/Commands/Score L3|L2",
        new SequentialCommandGroup(
            new ArmToPosition(arm, () -> 0)
            ));

    SmartDashboard.putData(
        "Driver " + "/Commands/Prepare To Remove Algae L2",
        new SequentialCommandGroup(
            new ElevatorToPosition(elevator, () -> 0.7),
            new ArmToPosition(arm, () -> 0)));

    SmartDashboard.putData(
        "Driver " + "/Commands/Prepare To Remove Algae L3",
        new SequentialCommandGroup(
            new ElevatorToPosition(elevator, () -> 0.9),
            new ArmToPosition(arm, () -> 0)));
  }
}
