package frc.robot.subsystems.controls.flywheel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.flywheel.FlywheelVelocity;
import frc.robot.commands.common.motor.MotorBringUpCommand;
import frc.robot.subsystems.interfaces.Flywheel;
import frc.robot.subsystems.interfaces.Motor;

public class FlywheelControls {
  // Right Bumper = Increase Rotation CW
  // Left Bumper = Increase Rotation CCW
  public static void setupController(Flywheel flywheel, CommandXboxController controller) {
    SubsystemBase flywheelSubsystem = (SubsystemBase) flywheel;
    flywheelSubsystem.setDefaultCommand(
        new MotorBringUpCommand(
            (Motor) flywheel,
            () -> {
              if (controller.rightBumper().getAsBoolean()) {
                return 1.0;
              } else if (controller.leftBumper().getAsBoolean()) {
                return -1.0;
              }
              return 0.0;
            }));
    
      SmartDashboard.putData(
          flywheelSubsystem.getName() + "/Commands/Flywheel at 1200 RPM", new FlywheelVelocity(flywheel, () -> 1200));
      SmartDashboard.putData(
          flywheelSubsystem.getName() + "/Commands/Flywheel at 600 RPM", new FlywheelVelocity(flywheel, () -> 600));
      SmartDashboard.putData(
          flywheelSubsystem.getName() + "/Commands/Flywheel at 300 RPM", new FlywheelVelocity(flywheel, () -> 300));
      SmartDashboard.putData(
          flywheelSubsystem.getName() + "/Commands/Flywheel at -300 RPM", new FlywheelVelocity(flywheel, () -> -300));
      SmartDashboard.putData(
          flywheelSubsystem.getName() + "/Commands/Flywheel at -600 RPM", new FlywheelVelocity(flywheel, () -> -600));
      SmartDashboard.putData(
          flywheelSubsystem.getName() + "/Commands/Flywheel at -1200 RPM", new FlywheelVelocity(flywheel, () -> -1200));

  }
}
