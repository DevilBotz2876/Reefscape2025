package frc.robot.subsystems.controls.arm;

import javax.print.DocFlavor.READER;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.arm.ArmCommand;
import frc.robot.commands.common.arm.ArmCommandWithButton;
import frc.robot.commands.common.arm.ArmCommand;
import frc.robot.subsystems.interfaces.Arm;
public class ClimberControls {
    public static void setupController(Arm climber, CommandXboxController controller) {
    SubsystemBase armSubsystem = (SubsystemBase) climber;
    armSubsystem.setDefaultCommand(
        new ArmCommand(
            climber,
            () -> {
              if (controller.y().getAsBoolean()) {
                return 1.0;
              } else if (controller.a().getAsBoolean()) {
                return -1.0;
              }
              return 0.0;
            }));
    controller.x().toggleOnTrue(new ArmCommandWithButton(climber, 90.0));
  }
}
