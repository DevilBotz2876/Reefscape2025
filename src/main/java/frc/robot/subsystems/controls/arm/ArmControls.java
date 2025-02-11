package frc.robot.subsystems.controls.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.arm.ArmCommand;
import frc.robot.subsystems.interfaces.Arm;

public class ArmControls {
  // RIGHT POV = up arm
  // LEFT POV = down arm
  public static void setupController(Arm arm, CommandXboxController controller) {
    SubsystemBase armSubsystem = (SubsystemBase) arm;
    // armSubsystem.setDefaultCommand(
        // new ArmCommand(
        //     arm,
        //     () -> {
        //       if (controller.pov(90).getAsBoolean()) {
        //         return 1.0;
        //       } else if (controller.pov(270).getAsBoolean()) {
        //         return -1.0;
        //       }
        //       return 0.0;
        //     }));

    controller
        .povUp()
        .whileTrue(
            new InstantCommand(
                () -> {
                  arm.runVoltage(1.0);
                },
                arm));

    controller
        .povDown()
        .whileTrue(
            new InstantCommand(
                () -> {
                  arm.runVoltage(-1.0);
                },
                arm));

    controller
        .povUp()
        .onFalse(
            new InstantCommand(
                () -> {
                  arm.runVoltage(0.0);
                },
                arm));

    controller
        .povDown()
        .onFalse(
            new InstantCommand(
                () -> {
                  arm.runVoltage(0.0);
                },
                arm));
  }
}
