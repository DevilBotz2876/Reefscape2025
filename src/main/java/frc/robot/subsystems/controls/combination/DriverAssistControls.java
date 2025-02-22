package frc.robot.subsystems.controls.combination;

import edu.wpi.first.math.MathUtil;
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

public class DriverAssistControls {
  public static void setupController(
      Drive drive, Elevator elevator, Arm arm, CommandXboxController controller) {
    SmartDashboard.putData(
        "Drive " + "/Commands/Prepare For Intake",
        new SequentialCommandGroup(
            new ArmToPosition(arm, () -> -90).withTimeout(0),
            new ElevatorToPosition(elevator, () -> 0.6)));

    SmartDashboard.putData(
        "Drive " + "/Commands/Intake",
        new SequentialCommandGroup(new ElevatorToPosition(elevator, () -> 0.2)));

    SmartDashboard.putData(
        "Driver Assist" + "/Commands/Prepare To Score L4",
        new SequentialCommandGroup(
            new ElevatorToPosition(elevator, () -> 1.0),
            new ArmToPosition(arm, () -> 75),
            new ElevatorToPosition(elevator, () -> 1.4)));
    SmartDashboard.putData(
        "Drive Assist" + "/Commands/Score L4",
        new SequentialCommandGroup(new ArmToPosition(arm, () -> -45)));
  }
}
