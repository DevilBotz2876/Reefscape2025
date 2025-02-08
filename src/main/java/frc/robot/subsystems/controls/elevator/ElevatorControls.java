package frc.robot.subsystems.controls.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.interfaces.Elevator;

public class ElevatorControls {
  // UP POV = Up elevator
  // DOWN POV = Down elevator
  public static void setupController(Elevator elevator, CommandXboxController controller) {
    SubsystemBase elevatorSubsystem = (SubsystemBase) elevator;
    controller
        .povUp()
        .onTrue(
            new InstantCommand(
                () -> {
                  elevator.runVoltage(6.0);
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  if (controller.povCenter().getAsBoolean()) {
                    elevator.runVoltage(0.0);
                  }
                }));
    ;

    controller
        .povDown()
        .onTrue(
            new InstantCommand(
                () -> {
                  elevator.runVoltage(-1.0);
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  if (controller.povCenter().getAsBoolean()) {
                    elevator.runVoltage(0.0);
                  }
                }));
  }

  public static void addSysId(Elevator elevator) {
    SmartDashboard.putData(
        "Elevator Dynamic Forward", elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData(
        "Elevator Dynamic Reverse", elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData(
        "Elevator Quasistatic Forward", elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData(
        "Elevator Quasistatic Reverse", elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
  }
}
