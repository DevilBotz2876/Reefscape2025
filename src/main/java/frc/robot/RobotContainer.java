// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.common.arm.ArmToPosition;
import frc.robot.commands.common.elevator.ElevatorCommand;
import frc.robot.config.game.reefscape2025.*;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.Properties;

import frc.robot.subsystems.implementations.arm.ArmSubsystem;
import frc.robot.subsystems.interfaces.Arm;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
  public final RobotConfig robotConfig;
  private static final String robotNameKey = "Robot Name";

  public RobotContainer() {
    String robotName = "UNKNOWN";

    // Load robot name from configuration file
    // Check if the robot is running in simulation
    if (RobotBase.isSimulation()) {

      Properties simulationProperties = new Properties();

      try (FileInputStream input = new FileInputStream("simulation.properties")) {
        simulationProperties.load(input);
        robotName = simulationProperties.getProperty("robot.name", robotName);
      } catch (IOException e) {
        System.err.println("Failed to load simulation configuration file: " + e.getMessage());
        System.exit(1);
      }
    } else {
      Preferences.initString(robotNameKey, robotName);
      robotName = Preferences.getString(robotNameKey, robotName);
    }

    System.out.println("Loading Settings for Robot Name = " + robotName);
    switch (robotName) {
      case "PHOENIX":
        robotConfig = new RobotConfigPhoenix();
        break;
      default:
        System.err.println("Failed to determine robot name.");
        System.exit(1);

        // Suppress Warnings.  This is unreachable, but the compiler doesn't know that
        robotConfig = new RobotConfigPhoenix();
        // robotConfig = new RobotConfigStub();
        // robotConfig = new RobotConfigSherman();
        //  robotConfig = new RobotConfigStub();
    }

    robotConfig.configureBindings();
  }

  public Command getAutonomousCommand() {
    return RobotConfig.autoChooser.getSelected();
  }
}
