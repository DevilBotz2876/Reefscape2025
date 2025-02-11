package frc.robot.subsystems.controls.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.common.elevator.ElevatorL1;
import frc.robot.commands.common.elevator.ElevatorL3;
import frc.robot.commands.common.elevator.ElevatorL4;
import frc.robot.commands.common.elevator.ElevatorOpenLoop;
import frc.robot.subsystems.interfaces.Elevator;

public class ElevatorControls {
  // UP POV = Up elevator
  // DOWN POV = Down elevator
  public static void setupController(Elevator elevator, CommandXboxController controller) {

    controller
        .povUp()
        .onTrue(new ElevatorOpenLoop(elevator, Elevator.Constants.upOpenLoopVoltsMax))
        .onFalse(new ElevatorOpenLoop(elevator, 0.0));
    ;

    controller
        .povDown()
        .onTrue(new ElevatorOpenLoop(elevator, Elevator.Constants.downOpenLoopVoltsMax))
        .onFalse(new ElevatorOpenLoop(elevator, 0.0));

    // maps to x on 8bitdo
    // controller.y().whileTrue(new ElevatorL4(elevator, Elevator.Constants.reefL4InMeters));

    // maps to a on 8bitdo
    // controller.a().whileTrue(new ElevatorL3(elevator, Elevator.Constants.reefL3InMeters));

    //maps to a on 8bitdo
    // controller.b().whileTrue(new ElevatorL1(elevator, Elevator.Constants.reefL1InMeters));
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
