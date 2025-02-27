// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.game.reefscape2025.*;
import frc.robot.util.Elastic;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.Properties;

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
    Elastic.sendNotification(
        new Elastic.Notification()
            .withDescription("Loading Settings for Robot Name = " + robotName));
    switch (robotName) {
      case "PHOENIX":
        robotConfig = new RobotConfigPhoenix();
        break;
      case "NEMO":
        robotConfig = new RobotConfigNemo();
        break;
      case "STUB":
        robotConfig = new RobotConfigStub();
        break;
      default:
        DriverStation.reportError("failed to determine robot name.  Default to NEMO", true);
        robotConfig = new RobotConfigNemo();
    }

    robotConfig.configureBindings();
  }

  public Command getAutonomousCommand() {
    return RobotConfig.autoChooser.getSelected();
  }
}
