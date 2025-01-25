package frc.robot.subsystems.controls.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.arm.ArmCommand;
import frc.robot.subsystems.interfaces.Arm;

public class ArmControls {
  public static void setupController(Arm arm, CommandXboxController controller) {
    SubsystemBase armSubsystem = (SubsystemBase) arm;
    armSubsystem.setDefaultCommand(
        new ArmCommand(
            arm,
            () -> MathUtil.applyDeadband(-controller.getRightY(), 0.05)
            ));
  }
}
