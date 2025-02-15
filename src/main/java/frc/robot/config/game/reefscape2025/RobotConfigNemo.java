package frc.robot.config.game.reefscape2025;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.io.implementations.motor.MotorIOBase.MotorIOBaseSettings;
import frc.robot.io.implementations.motor.MotorIOSparkMax;
import frc.robot.io.implementations.motor.MotorIOSparkMax.SparkMaxSettings;
import frc.robot.io.implementations.motor.MotorIOTalonFx;
import frc.robot.io.implementations.motor.MotorIOTalonFx.TalonFxSettings;
import frc.robot.subsystems.implementations.drive.DriveBase;
import frc.robot.subsystems.implementations.drive.DriveSwerveYAGSL;
import frc.robot.subsystems.implementations.motor.ArmMotorSubsystem;
import frc.robot.subsystems.interfaces.ArmV2.ArmSettings;
import frc.robot.subsystems.interfaces.Drive;

/* Override Nemo specific constants here */
public class RobotConfigNemo extends RobotConfig {
  public RobotConfigNemo() {
    super(false, true, true, true, false, true);

    // Nemo has a Swerve drive train
    Drive.Constants.rotatePidKp = 0.025;
    Drive.Constants.rotatePidKi = 0.0;
    Drive.Constants.rotatePidKd = 0.0;
    DriveBase.Constants.rotatePidErrorInDegrees = 1;
    drive = new DriveSwerveYAGSL("yagsl/nemo");

    // algae arm
    {
      MotorIOBaseSettings algaeArmMotorSettings = new MotorIOBaseSettings();
      algaeArmMotorSettings.motor.gearing = 50;
      algaeArmMotorSettings.motor.inverted = false;
      algaeArmMotorSettings.pid = new PIDController(0, 0, 0);

      ArmSettings algaeArmSettings = new ArmSettings();
      algaeArmSettings.minAngleInDegrees = -15;
      algaeArmSettings.maxAngleInDegrees = 105;
      algaeArmSettings.startingAngleInDegrees = algaeArmSettings.maxAngleInDegrees;
      algaeArmSettings.color = new Color8Bit(Color.kGreen);
      algaeArmSettings.feedforward = new ArmFeedforward(0, 0, 0, 0);
      algaeArmSettings.armLengthInMeters = 0.75;
      algaeArmSettings.armMassInKg = 1.0;
      algaeArmSettings.motor = DCMotor.getKrakenX60(1);
      algaeArmSettings.simulateGravity = false;

      SparkMaxSettings algaeArmSparkMaxSettings = new SparkMaxSettings();
      algaeArmSparkMaxSettings.canId = 21;

      algaeArm =
          new ArmMotorSubsystem(
              new MotorIOSparkMax(algaeArmMotorSettings, algaeArmSparkMaxSettings),
              "Algae",
              algaeArmSettings);
    }

    // Coral Arm
    {
      MotorIOBaseSettings motorSettings = new MotorIOBaseSettings();
      // 20:1 gear box, 30 teeth on the arm cog and 15 teeth on the motor cog
      motorSettings.motor.gearing = 20 * (30.0 / 15.0);
      motorSettings.motor.inverted = true; // false for Sim
      motorSettings.pid = new PIDController(0.0, 0, 0);

      ArmSettings armSettings = new ArmSettings();
      armSettings.minAngleInDegrees = 0;
      armSettings.maxAngleInDegrees = 150;
      armSettings.startingAngleInDegrees = armSettings.minAngleInDegrees;
      armSettings.feedforward = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);
      armSettings.color = new Color8Bit(Color.kRed);
      armSettings.armLengthInMeters = 0.5;
      armSettings.armMassInKg = 1.0;
      armSettings.motor = DCMotor.getKrakenX60(1);
      armSettings.simulateGravity = true;

      TalonFxSettings talonFxSettings = new TalonFxSettings();
      talonFxSettings.canId = 21;

      coralArm =
          new ArmMotorSubsystem(
              // new MotorIOArmStub(motorSettings, armSettings), "Coral", armSettings);
              new MotorIOTalonFx(motorSettings, talonFxSettings), "Coral", armSettings);
    }
  }
}
