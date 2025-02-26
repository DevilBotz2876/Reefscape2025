package frc.robot.subsystems.controls.led;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.implementations.led.LEDSubsystem;
import frc.robot.subsystems.interfaces.LED;

public class LEDControls {
  public static void setupController(LED led, CommandXboxController controller) {
    SubsystemBase LEDSubsystemm = (SubsystemBase) led;
    LEDSubsystemm.setDefaultCommand(
        new InstantCommand(
            () -> {
              if (RobotState.isAutonomous()) {
                 led.startRainbow(255, 128, 1);
              }
              if (RobotState.isTeleop()) {
                led.setColor(new Color(0, 255, 255));
              }
              if (RobotState.isDisabled()) {
                led.setColor(new Color(0, 0, 255));
              }
            }, LEDSubsystemm));
  }

  public static void setupController2(LED led, CommandXboxController controller) {
    SubsystemBase LEDSubsystem = (SubsystemBase) led;
    LEDSubsystem.setDefaultCommand(
        new InstantCommand(
            () -> {
              if (RobotState.isAutonomous()) {
                led.setColor(new Color(0, 0, 0));
              }
              if (RobotState.isTeleop()) {
                led.setColor(new Color(0, 255, 0));
              }
              if (RobotState.isDisabled()) {
                led.setColor(new Color(0, 0, 255));
              }
            }, LEDSubsystem));
  }
}
