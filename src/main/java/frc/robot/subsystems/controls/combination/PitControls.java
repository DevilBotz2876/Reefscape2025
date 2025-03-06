package frc.robot.subsystems.controls.combination;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.common.motor.MotorPitCommand;
import frc.robot.subsystems.interfaces.Arm;
import frc.robot.subsystems.interfaces.Elevator;
import frc.robot.subsystems.interfaces.Motor;

public class PitControls {
  public static void setupPitControls(Elevator elevator, Arm arm, Arm climber) {
    PitControls.setupElevatorPitControls(elevator);
    PitControls.setupArmPitControls(arm);
    PitControls.setupArmPitControls(climber);
  }

  private static void setupElevatorPitControls(Elevator elevator) {
    SubsystemBase elevatorSubsystem = (SubsystemBase) elevator;

    Command runVoltsCommand =
        new MotorPitCommand(
            (Motor) elevatorSubsystem,
            "Pit/" + elevatorSubsystem.getName() + "/Commands/Run Voltage/Volts Amount");
    SmartDashboard.putData(
        "Pit/" + elevatorSubsystem.getName() + "/Commands/Run Voltage/Command", runVoltsCommand);
  }

  private static void setupArmPitControls(Arm arm) {

    SubsystemBase armSubsystem = (SubsystemBase) arm;

    Command runVoltsCommand =
        new MotorPitCommand(
            (Motor) armSubsystem,
            "Pit/" + armSubsystem.getName() + "/Commands/Run Voltage/Volts Amount");
    SmartDashboard.putData(
        "Pit/" + armSubsystem.getName() + "/Commands/Run Voltage/Command", runVoltsCommand);
  }
}
