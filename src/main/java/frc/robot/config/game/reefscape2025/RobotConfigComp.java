package frc.robot.config.game.reefscape2025;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.commands.common.arm.ArmToPosition;
import frc.robot.commands.common.elevator.ElevatorToPosition;
import frc.robot.io.implementations.motor.MotorIOBase.MotorIOBaseSettings;
import frc.robot.io.implementations.motor.MotorIOTalonFx;
import frc.robot.io.implementations.motor.MotorIOTalonFx.TalonFxSettings;
import frc.robot.subsystems.controls.arm.ClimberArmControls;
import frc.robot.subsystems.controls.arm.CoralArmControls;
import frc.robot.subsystems.controls.elevator.ElevatorControls;
import frc.robot.subsystems.implementations.drive.DriveBase;
import frc.robot.subsystems.implementations.drive.DriveSwerveYAGSL;
import frc.robot.subsystems.implementations.motor.ArmMotorSubsystem;
import frc.robot.subsystems.implementations.motor.ElevatorMotorSubsystem;
import frc.robot.subsystems.implementations.motor.SimpleMotorSubsystem;
import frc.robot.subsystems.interfaces.Arm.ArmSettings;
import frc.robot.subsystems.interfaces.Drive;
import frc.robot.subsystems.interfaces.Elevator.ElevatorSettings;
import frc.robot.subsystems.interfaces.SimpleMotor.SimpleMotorSettings;
import frc.robot.subsystems.interfaces.Vision.Camera;

public class RobotConfigComp extends RobotConfig {
  private final AddressableLED led;
  private final AddressableLEDBuffer ledBuffer;

  public RobotConfigComp() {
    super(false, false, false, false, false, true, false);

    // Comp has a Swerve drive train
    Drive.Constants.rotatePidKp = 0.025;
    Drive.Constants.rotatePidKi = 0.0;
    Drive.Constants.rotatePidKd = 0.0;
    DriveBase.Constants.rotatePidErrorInDegrees = 1;
    drive = new DriveSwerveYAGSL("yagsl/comp");

    // Cameras
    // vision.addCamera(
    //     new Camera(
    //         "rear_cam",
    //         new Transform3d(
    //             new Translation3d(
    //                 Units.inchesToMeters(0.295),
    //                 Units.inchesToMeters(-11.443),
    //                 Units.inchesToMeters(39.663)),
    //             new Rotation3d(
    //                 Units.degreesToRadians(12),
    //                 Units.degreesToRadians(-33),
    //                 Units.degreesToRadians(170)))));
    vision.addCamera(
        new Camera(
            "left_cam", // left
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(3.250),
                    Units.inchesToMeters(13.592),
                    Units.inchesToMeters(7.201)),
                new Rotation3d(0.0, Units.degreesToRadians(-5.0), Units.degreesToRadians(90.0)))));

    vision.addCamera(
        new Camera(
            "right_cam", // right
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(3.250),
                    Units.inchesToMeters(-13.592),
                    Units.inchesToMeters(7.201)),
                new Rotation3d(0.0, Units.degreesToRadians(-5.0), Units.degreesToRadians(270.0)))));
    vision.addCamera(
        new Camera(
            "front_cam", // front
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(13.592),
                    Units.inchesToMeters(2.75),
                    Units.inchesToMeters(7.201)),
                new Rotation3d(0.0, Units.degreesToRadians(-5.0), 0.0))));

    // Elevator
    {
      MotorIOBaseSettings motorSettings = new MotorIOBaseSettings();
      motorSettings.motor.gearing = 9; /* 2x 3:1 gear boxes */
      motorSettings.motor.inverted = true;
      // TODO: Get the DIO ports
      // motorSettings.forwardLimitChannel = 7;
      // motorSettings.forwardLimitNegate = true;
      // motorSettings.reverseLimitChannel = 2;
      // motorSettings.reverseLimitNegate = true;
      motorSettings.motor.drumRadiusMeters =
          Units.inchesToMeters(
              ((1.16 + 0.75) / 2) / 2); // 3/4" inner diameter to 1.16" outer when fully spooled.
      motorSettings.pid = new PIDController(0.2, 0, 0);
      ElevatorSettings elevatorSettings = new ElevatorSettings();
      elevatorSettings.minHeightInMeters = 0.09 + 0.02;
      elevatorSettings.maxHeightInMeters = 0.02 + 0.85 + 0.76;
      elevatorSettings.startingHeightInMeters = 0.3; // The elevator height when piece is in intake
      elevatorSettings.color = new Color8Bit(Color.kSilver);
      elevatorSettings.feedforward =
          new ElevatorFeedforward(0.010472, 0.17328, 0.16928, 0.010615); // SysID 2025-02-28
      elevatorSettings.carriageMassKg = 5.0;
      elevatorSettings.motor = DCMotor.getKrakenX60(1);
      elevatorSettings.simulateGravity = true;

      TalonFxSettings talonFxSettings = new TalonFxSettings();
      talonFxSettings.canId = 20;

      elevatorSettings.maxVelocityInMetersPerSecond = 1;
      elevatorSettings.maxAccelerationInMetersPerSecondSquared = 8;

      ElevatorControls.Constants.autoZeroSettings.voltage = 1.5;
      ElevatorControls.Constants.autoZeroSettings.minResetCurrent = 8;
      ElevatorControls.Constants.autoZeroSettings.resetPositionRad =
          elevatorSettings.maxHeightInMeters / motorSettings.motor.drumRadiusMeters;
      ElevatorControls.Constants.autoZeroSettings.initialReverseDuration =
          0.0; // Set the seconds of reverse before zero. Set to zero if there shound be no reverse

      elevator =
          new ElevatorMotorSubsystem(
              new MotorIOTalonFx(motorSettings, talonFxSettings), "Coral", elevatorSettings);
    }

    // Coral Arm
    {
      MotorIOBaseSettings motorSettings = new MotorIOBaseSettings();
      // 20:1 gear box, 30 teeth on the arm cog and 15 teeth on the motor cog
      motorSettings.motor.gearing = 40;
      motorSettings.motor.inverted = true; // false for Sim
      motorSettings.pid = new PIDController(1.0, 0.0, 0);
      // TODO: Get the DIO ports
      // motorSettings.reverseLimitChannel = 1;
      // motorSettings.reverseLimitNegate = true;

      ArmSettings armSettings = new ArmSettings();
      armSettings.minAngleInDegrees = -90;
      armSettings.maxAngleInDegrees = 60;
      armSettings.startingAngleInDegrees = armSettings.minAngleInDegrees;
      armSettings.feedforward = new ArmFeedforward(0.0, 0.17, 0.7, 0.0);
      armSettings.color = new Color8Bit(Color.kRed);
      armSettings.armLengthInMeters = 0.5;
      armSettings.armMassInKg = 1.0;
      armSettings.motor = DCMotor.getKrakenX60(1);
      armSettings.simulateGravity = true;
      armSettings.maxVelocityInDegreesPerSecond = 180;
      armSettings.maxAccelerationInDegreesPerSecondSquared = 720;

      TalonFxSettings talonFxSettings = new TalonFxSettings();
      talonFxSettings.canId = 21;

      CoralArmControls.Constants.autoZeroSettings.voltage = -0.5;
      CoralArmControls.Constants.autoZeroSettings.minResetCurrent = 0.5;
      CoralArmControls.Constants.autoZeroSettings.resetPositionRad =
          Units.degreesToRadians(
              armSettings.minAngleInDegrees - 10); // We have an offest about 15 degrees
      CoralArmControls.Constants.autoZeroSettings.initialReverseDuration =
          0.0; // Set the seconds of reverse before zero. Set to zero if there shound be no reverse

      coralArm =
          new ArmMotorSubsystem(
              // new MotorIOArmStub(motorSettings, armSettings), "Coral", armSettings);
              new MotorIOTalonFx(motorSettings, talonFxSettings), "Coral", armSettings);
    }

    // climber
    {
      MotorIOBaseSettings motorSettings = new MotorIOBaseSettings();
      // 25:1 gear box ratio
      motorSettings.motor.gearing = 25;
      motorSettings.motor.inverted = false;
      motorSettings.pid = new PIDController(1.0, 0, 0);
      motorSettings.reverseLimitChannel = 1;
      motorSettings.reverseLimitNegate = true;

      SimpleMotorSettings simpleMotorSettings = new SimpleMotorSettings();
      simpleMotorSettings.minPositionInRads = 0;
      simpleMotorSettings.maxPositionInRads = 36.1;
      simpleMotorSettings.startingPositionInRads = simpleMotorSettings.minPositionInRads;
      simpleMotorSettings.color = new Color8Bit(Color.kRed);
      simpleMotorSettings.feedforward = new SimpleMotorFeedforward(0, 0, 0);
      simpleMotorSettings.motor = DCMotor.getNEO(1);

      TalonFxSettings settings = new TalonFxSettings();
      settings.canId = 50;

      ClimberArmControls.Constants.autoZeroSettings.voltage = -1;
      // Set this to something big, we are never going to use stall current to detect if climber has
      // reached it's end of range of motion.
      ClimberArmControls.Constants.autoZeroSettings.minResetCurrent = 0.5;
      ClimberArmControls.Constants.autoZeroSettings.resetPositionRad =
          simpleMotorSettings.minPositionInRads;
      ClimberArmControls.Constants.autoZeroSettings.initialReverseDuration = 0;

      climberArm =
          new SimpleMotorSubsystem(
              new MotorIOTalonFx(motorSettings, settings), "Climber", simpleMotorSettings);
    }

    // LED
    {
      led = new AddressableLED(0);
      ledBuffer = new AddressableLEDBuffer(120);

      LEDPattern purple = LEDPattern.solid(Color.kPurple);

      // Apply the LED pattern to the data buffer
      purple.applyTo(ledBuffer);

      led.setLength(ledBuffer.getLength());
      led.setData(ledBuffer);
      led.start();
    }

    // Auto(s)
    NamedCommands.registerCommand(
        "Move Elevator to 0.5 meter", new ElevatorToPosition(elevator, () -> 0.5));
    NamedCommands.registerCommand(
        "Move Elevator to 1.553 meter", new ElevatorToPosition(elevator, () -> 1.553));
    NamedCommands.registerCommand(
        "Move Arm 75 degrees", new ArmToPosition(coralArm, () -> 70).withTimeout(1.5));
    NamedCommands.registerCommand(
        "Move Arm 0 degrees", new ArmToPosition(coralArm, () -> 0).withTimeout(1.5));
    NamedCommands.registerCommand(
        "Move Elevator to 0.8 meter", new ElevatorToPosition(elevator, () -> 0.8));
    NamedCommands.registerCommand(
        "Move Elevator to 0.4 meter", new ElevatorToPosition(elevator, () -> 0.4));
    NamedCommands.registerCommand(
        "Move Arm for Intake", new ArmToPosition(coralArm, () -> -90).withTimeout(0));
    autoChooser = AutoBuilder.buildAutoChooser("Sit Still");

    // Start webcam
    CameraServer.startAutomaticCapture();
  }
}
