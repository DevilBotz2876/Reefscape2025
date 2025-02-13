package frc.robot.subsystems.controls.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.interfaces.Motor;

public class CoralArmControls {
  // RIGHT POV = up arm
  // LEFT POV = down arm
  public static void setupController(Motor arm, CommandXboxController controller) {
    SubsystemBase armSubsystem = (SubsystemBase) arm;
    // armSubsystem.setDefaultCommand(
    //     new ArmCommandV2(
    //         arm,
    //         () -> {
    //           if (controller.pov(90).getAsBoolean()) {
    //             return 1.0;
    //           } else if (controller.pov(270).getAsBoolean()) {
    //             return -1.0;
    //           }
    //           return 0.0;
    //         }));

    controller
        .povRight()
        .whileTrue(
            new InstantCommand(
                () -> {
                  arm.runVoltage(1.0);
                },
                (SubsystemBase) arm));

    controller
        .povLeft()
        .whileTrue(
            new InstantCommand(
                () -> {
                  arm.runVoltage(-1.0);
                },
                (SubsystemBase) arm));

    controller
        .povRight()
        .onFalse(
            new InstantCommand(
                () -> {
                  arm.runVoltage(0.0);
                },
                (SubsystemBase) arm));

    controller
        .povLeft()
        .onFalse(
            new InstantCommand(
                () -> {
                  arm.runVoltage(0.0);
                },
                (SubsystemBase) arm));
  }
}
