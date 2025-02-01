package frc.robot.subsystems.controls.algae;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.interfaces.Algae;

public class AlgaeControls {
    public static void setupController(Algae algae, CommandXboxController controller) {
        controller.rightBumper().onTrue(
            algae.getTurnRightIntakeCommand()
        ).onFalse(algae.getTurnOffIntakeCommand());

        controller.leftBumper().onTrue(
            algae.getTurnLeftIntakeCommand()
        ).onFalse(algae.getTurnOffIntakeCommand());

        controller.rightTrigger().onTrue( new InstantCommand(() -> {
            algae.runVoltageArm(6.0);
        }, algae));

        controller.leftTrigger().onTrue( new InstantCommand(() -> {
            algae.runVoltageArm(-3.0);
        }, algae));

        controller.rightTrigger().onFalse(new InstantCommand(() -> {
            algae.runVoltageArm(0.0);
        }, algae));

        controller.leftTrigger().onFalse(new InstantCommand(() -> {
            algae.runVoltageArm(0.0);
        }, algae));
  }
}
