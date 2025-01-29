package frc.robot.subsystems.controls.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.arm.ArmCommand;
import frc.robot.subsystems.interfaces.Arm;

public class ArmControls {
  public static void setupController(Arm arm, CommandXboxController controller) {
    SubsystemBase armSubsystem = (SubsystemBase) arm;
    armSubsystem.setDefaultCommand(
        new ArmCommand(arm, () -> {
            if (controller.pov(90).getAsBoolean()) {
              return 1.0;
            } else if(controller.pov(270).getAsBoolean()) {
              return -1.0;
            }
            return 0.0;
          }
        )
      );
  }
}
