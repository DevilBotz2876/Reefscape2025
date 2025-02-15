This is the top-level folder for the DevilBotz robot code

The code uses Java WPILib and the [WPILib command-based](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html) programming paradigm


# Goals/Features
* Extensibility and Reusability
   * One code base, multiple robots
* Debuggability
   * Logging
   * Simulation
* Vision Based Odometry
* Odometry-based Path Following

# General Resources:
* [WPILib Documentation](https://docs.wpilib.org/en/stable/)
* [DevilBotz 2876 Standard Laptop Configuration](https://docs.google.com/document/d/1NRJyu0b7zgzJpNy1R-b7AjI3fnhwZ_R6fZo0EVbo2pM)
* [FRC Programming in WPILib](https://youtube.com/playlist?list=PL4GNHenJg9JD5xdRxByaZZEZP1PPajPeV&si=1dgx1ZFQP8GEq4w_)
* [Git Tutorial](https://learngitbranching.js.org/)
* [Java Tutorial](https://youtube.com/playlist?list=PLZPZq0r_RZOMhCAyywfnYLlrjiVOkdAI1&si=ImrjG_c4-wThJqy3)

# Getting Started
1. Install [WPILib](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/index.html)
1. Install Git
    1. Windows
    1. MacOS
1. Get Code from GitHub
    1. Create GitHub Account (if you don't already have one)
    1. Start WPILib VSCode
    1. Clone Repository
    1. Login to GitHub Account (if prompted)
    1. Repo: https://github.com/DevilBotz2876/XxxYYYY (where Xxx is the name of the FRC game and YYYY is the year E.g. "Reefscape2025"
      * Note: The initial compilation will take a while because of all the dependencies that need to be downloaded. If all works well, in the Terminal view at the bottom, you should see:
1. Simulating the Code
   1. Starting a Simulation
      * "F5" starts the simulation
         * Alternate Steps:
            * "Ctrl-Shift-P" or "Command-Shift-P"
            * "Simulate Robot Code"
      * Enable: "Sim GUI"
         * If you get the following error: "cannot find frc.robot.main", then:
            * "Ctrl-Shift-P" --> "Clean Language Server Workspace"
      * Optional: Enable DriverStation to better mimic real world usage
   1. Robot Simulation (aka Sim GUI) Overview
      1. Configure XBox Controller
         1. Make sure joystick is in "X-Box Mode". Plug in Joystick.
         1. Verify Joystick is Detected in "System Joysticks" window
            1. On Mac OS: You must connect via a bluetooth wireless enabled controller
         1. Drag controller from "System Joysticks" to "Joysticks" (Joystick[0])
         1. Make sure to "Map Gamepad" is checked
      1. Open 2D Field View
         1. NetworkTables --> "SmartDashboard" --> "Field"
         1. To Hide Swerve Modules:
            1. Click on the "Hamburger Button"
            1. Goto "XModules"
               * Box/Image --> Hidden
               * Arrow Size --> 10% (slider)
      1. Set Robot State
         1. Set Mode to "Teleoperated"
         1. Thumb Controls should now move the robot
   1. 2D Mechanism Simulation
      1. NetworkTables --> "SmartDashboard" --> "Robot 2D Simulation"
   1. Vision Field
       1.  NetworkTables --> "SmartDashboard" --> "VisionSystemSim-main" --> Sim Field
   1. Alliance Selection
   1. Autonomous
   1. AdvantageScope

# Directory Structure
* [`.github/workflows`](.github/workflows/): Contains various GitHub automation scripts that are run automatically at various stages of the code development/
integration process.
* [`vendordeps`](vendordeps/): Contains the various external libraries and versions that the code base utilizes.
* [`src/main`](src/main/): All of the robot code and config files are here
   * [`deploy`](src/main/deploy/): Config files, resources, etc that need to be stored directly on the actual robot.  These can be read using standard Java File I/O calls
   * [`java/frc/robot`](src/main/java/frc/robot/): This is the root of all of the actual Java code for the robot
      * [`commands/`](src/main/java/frc/robot/commands/): All commands are stored in this folder
         * [`common/`](src/main/java/frc/robot/commands/common/): Common commands are ones that operate on the Subsystem *interface*, so doesn't rely on any game specific implementations/APIs.
            * `motor`
               * [`MotorBringUpCommand.java`](src/main/java/frc/robot/commands/common/motor/MotorBringUpCommand.java): _TODO_
            * `gizmo`
               * `GizmoCommand.java`
            * `arm/`
            * `drive/`
               * `DriveCommand.java`
               * ...
            * ...
      * [`config/`](src/main/java/frc/robot/config/): The config directory is used to support multiple robot versions using the same code base.
         * `game/`(src/main/java/frc/robot/config/game/): Each game will generally have it's own robot configuration.
            * `reefscape2025/`: game-specific robot configurations are stored here.
               * `RobotConfig.java`: Defines all of the subsystems and controls required for a "robot". Each robot _variant_ extends RobotConfig and instantiates the subsystems and overrides constants as needed. Implements simulation stubs for all subsystems.
              * `RobotConfigPhoenix..java`: Robot config for the "Phoenix" robot that instantiates specific subsystems (e.g. drivetrain and vision) using specific hardware IO instances. Everything else not explicitly overrideen will use the default _stub _versions.
               * ...
      * [`io/`](src/main/java/frc/robot/io): generic *low-level* hardware IO (e.g. Motors, Limit Switches, Sensors, etc)
         * [`interfaces/`](src/main/java/frc/robot/io/interfaces/): *hardware interfaces* that are common for all possible implementations.  E.g. a MotorIO would generally have a way to at least `setVoltage` and get status such as `voltage` applied and actual `amperage` used.
            * [`MotorIO.java`](src/main/java/frc/robot/io/interfaces/MotorIO.java): Defines the minimum expected interface (settings, control, and status) for a generic motor.
               * `setVoltage(double volts)`
               * `setVelocity(double velocityRadPerSec, double ffVolts)`
               * `setPosition(double positionRad, double ffVolts)`
               * `getPid()`
            * `GizmoIO.java`
            * `ArmIO.java`
            * `ClimberIO.java`
            * ...
         * [`implementations/`](src/main/java/frc/robot/io/implementations/): Hardware specific implementations of the hardware IO interfaces.  Each interface may have one or more implementations depending on manufacturer, model, etc.  _Each interface must have a simulation stub implementation_
            * `motor/`
               * [`MotorIOBase.java`](src/main/java/frc/robot/io/implementations/motor/MotorIOBase.java): The base *abstract* implementation of the [Motor](src/main/java/frc/robot/io/interfaces/MotorIO.java) interface.  All motor implementations should extend this base class.  It implements the following functionality:
                  * generates status in other units (e.g. from radians to degrees, RPMs, etc)
                  * implements _software_ based velocity and position PID control
               * Stub Implementations - for software bring-up in simulation. Each implements the physics of different types of mechanisms
                  * [`MotorIOStub.java`](src/main/java/frc/robot/io/implementations/motor/MotorIOStub.java): Generic DC Motor based Mechanism (uses DCMotorSim)
                  * [`MotorIOArmStub.java`](src/main/java/frc/robot/io/implementations/motor/MotorIOArmStub.java): Motor connected to an [Arm](src\main\java\frc\robot\subsystems\interfaces\ArmV2.java) Mechanism (uses SingleJointedArmSim)
                  * [`MotorIOFlywheelStub.java`](src/main/java/frc/robot/io/implementations/motor/MotorIOFlywheelStub.java): Motor connected to a [Flywheel](src\main\java\frc\robot\subsystems\interfaces\Flywheel.java)/Roller Mechanism (uses FlywheelSim)
                  * [`MotorIOElevatorStub.java`](src/main/java/frc/robot/io/implementations/motor/MotorIOElevatorStub.java): Motor connected to an [Elevator](src\main\java\frc\robot\subsystems\interfaces\ElevatorV2.java) Mechanism (uses ElevatorSim)
               * Real Implementations
                  * [`MotorIOSparkMax.java`](src/main/java/frc/robot/io/implementations/motor/MotorIOSparkMax.java): SparkMax Motor Controller implementation
                  * [`MotorIOTalonFx.java`](src/main/java/frc/robot/io/implementations/motor/MotorIOTalonFx.java): TalonFx Motor Controller implementation
            * `gizmo/`
               * `GizmoIOStub.java`: Dummy/Stub implementation of a "Gizmo"
               * `GizmoIOAcme.java`: Implementation of the Acme brand/model of a "Gizmo"
            * `arm/`
               * `ArmIOStub.java`
               * `ArmIOSparkMax.java`
               * ...
            * `climber/`
            * ...
      * [`subsystems/`](src/main/java/frc/robot/subsystems): subsystems are implemented using one or more Hardware IO instances.  E.g. an Arm may contains an instance of a MotorIO connected to gears (to move the Arm up/down) and an AbsoluteEncoderIO (to measure the actual current angle).
         * [`interfaces/`](src/main/java/frc/robot/subsystems/interfaces/): *subsystem interfaces* that are common for all possible implementations
            * [`Motor.java`](src/main/java/frc/robot/subsystems/interfaces/Motor.java): _TODO_
            * [`ArmV2.java`](src/main/java/frc/robot/subsystems/interfaces/ArmV2.java): _TODO_
            * [`Flywheel.java`](src/main/java/frc/robot/subsystems/interfaces/Flywheel.java): _TODO_
            * [`ElevatorV2.java`](src/main/java/frc/robot/subsystems/interfaces/ElevatorV2.java): _TODO_
            * `Gizmo.java`
            * `Arm.java`
            * `Climber.java`
            * ...
         * [`controls/`](src/main/java/frc/robot/subsystems/controls/): All controls including driver, debug, pit, and sysid are defined here.  This includes both joystick and GUI-based controls. Contains the logic to connect specific user interfaces (e.g. ShuffleBoard, XBox Controllers, etc) to subsystem functionality. Ideally, the controls should be operating on the subsystem's _interface_ and *not* the actual implementation.
             * `gizmo`
               * `GizmoControls.java`
            * `drive`
               * `DriveControls.java`
            * `vision`
               * `VisionControls.java`
         * [`implementations/`](src/main/java/frc/robot/subsystems/implementations/): Subsystem specific implementations that use one or more hardware IO instances.  May need to be configured differently depending on the Robot Configuration. E.g. for an Arm, the absolute encoder offset will likely be different in different implementations of the Arm.  The subsystem should provide a mechanism to configure it accordingly.
             * `motor`
               * [`MotorSubsystem.java`](src/main/java/frc/robot/subsystems/implementations/motor/MotorSubsystem.java): _TODO_
               * [`ArmMotorSubsystem.java`](src/main/java/frc/robot/subsystems/implementations/motor/ArmMotorSubsystem.java): _TODO_
               * [`FlywheelMotorSubsystem.java`](src/main/java/frc/robot/subsystems/implementations/motor/FlywheelMotorSubsystem.java): _TODO_
               * [`ElevatorMotorSubsystem.java`](src/main/java/frc/robot/subsystems/implementations/motor/ElevatorMotorSubsystem.java): _TODO_
            * `gizmo/`
               * `GizmoSubsystem.java`
            * `arm/`
               * `ArmSubsystem.java`
            * `climber/`
               * `ClimberSubsystem.java`
            * ...


# *Note: The content below may be out of date and needs to be reviewed/updated*
## Useful Resources:
* [Crescendo 2024 Hardware/Software Calibration](https://docs.google.com/document/d/1msJO2dKCxqzbMlSSOtfs8W_ITjltEbtXaQBxc-CpNMo)


## 2024
We had 3 different robots with different capabilities for dev/testing purposes:

1. Sherman: Tank Drive + Initial Arm Prototype
2. Phoenix: Initial Swerve Drive + Vision Prototype
3. Inferno: Final Robot


| Robot | [Drive](../subsystems/drive/) | [Shooter](../subsystems/shooter/) | [Intake](../subsystems/intake/) | [Arm](../subsystems/arm/) | [Auto](../../../../deploy/pathplanner/) | [Climber](../subsystems/climber/) | [Vision](../subsystems/vision/) | [LED](../subsystems/led/) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| [Sherman](RobotConfigSherman.java) | Differential | SparkMax(2) | SparkMax(1) | TalonSRX(3) | No (stub) | No (stub) | No (stub) | No (stub) | No (stub) |
| [Phoenix](RobotConfigPhoenix.java) | Swerve(yagsl) | No (stub) | No (stub) | No (stub) | Yes | No (stub) | Shooter | No (stub) |
| [Inferno](RobotConfigInferno.java) | Swerve(yagsl) | SparkMax(2) | SparkMax(3) | SparkMax(4) | Yes | SparkMax(7) SparkMax(6) | Shooter Intake Right Left | Yes |


## Subsystems
Subsystem and IO interfaces and implementations

Subsystems utilize the [AdvantageKit IO Layer](https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/RECORDING-INPUTS.md) paradigm.


Each subsystem is implemented using the following breakdown.  Here, assume we have a mechanism called a "Gizmo":
* Hardware IO
   * _Simple_ Interface (defines the desired _hardware_ functionality)
      * Gizmo*IO*.java
        ```
        public static class GizmoIOInputs {
            public double velocityRadPerSec = 0.0;
            public double appliedVolts = 0.0;
            public double current;
        }

        void updateInputs(GizmoIOInputs inputs); ← gets current sensor readings

        void setVoltage(double volts); ← sets desired voltage
        ```
   * One _or more_ Implementation(s)
      * GizmoIO*Stub*.java ← Simulated implementation
        ```
        public class GizmoIOStub implements GizmoIO {
        ```
      * GizmoIO*SparkMax*.java ← SparkMax motor based implementation
      * GizmoIO*Etc*.java
* Subsystem (utilizes one or more Hardware IO implementation)
   * _Single_ Interface (defines the desired _subsystem_ functionality)
      * Gizmo*Subsystem*.java
        * Low Level/Debug Controls
            ```
            void runVoltage(double volts);
            double getCurrentVoltage();
            void add2dSim(Mechanism2d mech2d);
            ```
        * High Level Controls
            ```
            Command getTurnOffCommand();
            Command getTurnOnCommand();
            ```
   * One _or more_ Implementation(s)
      * GizmoSubsystem*Simple*.java
        ```
        public class GizmoSubsystemSimple implements Gizmo {
            GizmoSubsystemSimple(GizmoIO io); ← an IO instance is passed into the GizmoSubsystemSimple

            void periodic() {
                IO.updateInputs(inputs);
                Logger.processInputs("Intake", inputs)
            }
            ...
        }
        ```
      * GizmoSubsystem*Advanced*.java
        ```
        public class GizmoSubsystemAdvanced implements Gizmo {
            ...
        }
        ```
      * GizmoSubsystem*Etc*.java

It is important to note that the *same* Hardware IO can be shared by completely different subsystems.  E.g. we may just want a Hardware IO layer abstraction for basic functionality  E.g.
* MotorIO
   * MotorIOSparkMax
   * MotorIOTalonSRX
* LimitSwitchIO

And then a subsystem can consist of 1 or more MotorIO instances and 1 or more LimitSwitchIO instances.




## Code Flow
1. [Main.java](Main.java) --> [Robot.java](Robot.java) --> [RobotContainer.java](RobotContainer.java)
   1. Setup Robot Config
      * A persistent preference with the key "Robot Name" is hard-coded onto the RoboRio flash memory
      * The "Robot Name" key is read from the RoboRio at start and determines which robot config to load.
   1. Setup Controls (Bindings)
   1. Setup Autonomous


## Pathplanner
Path and Auto routine configuration files generated by the PathPlanner App

### Useful Resources:
* [PathPlanner](https://pathplanner.dev/pathplanner-gui.html)

## Yagsl
YAGSL swerve configuration files for each robot

### Useful Resources
* [YAGSL](https://yagsl.gitbook.io/yagsl)
* [Crescendo 2024 Swerve Bring-Up Checklist - Phoenix](https://docs.google.com/document/d/1E4FAC1YTaWKI4XfINkX6v6q33PmBH5FilczPb08y0cg)
* [Crescendo 2024 Swerve Bring-Up Checklist - Inferno](https://docs.google.com/document/d/104QvJt_8DLLTVLK-61OBEZm78WMeOHpgzdBgT3CPT0M)

## Vision
### Useful Resources:
* [PhotonVision](https://docs.photonvision.org/en/latest/)

1. Initial Setup
    1. For each camera
        * On the PhotonVision co-processor (e.g.. Raspberry Pi)
            * Assign a unique camera name
            * [2D AprilTag tracking](https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/2D-tracking-tuning.html) will work out of the box
                * You can get camera relative yaw, pitch, and roll information for each visible/detected AprilTag w/o calibrating the camera
            * [3D AprilTag tracking](https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/3D-tracking.html) requires calibration for each resolution:
                * [Calibrate](https://docs.photonvision.org/en/latest/docs/calibration/calibration.html) FOV
                * Measure location of camera relative to the center of the robot (See Transform3D [VisionCamera.VisionCamera](./VisionCamera.java))
                    * Measure translation (x,y,z)
                    * Measure rotation (roll, pitch, yaw)
    1. For each field
        * Load the AprilTag map layout into PhotonVision (See AprilTagFields.class)
2. Periodically at runtime
    1. For each camera
        * If estimated robot pose is available (i.e. AprilTag is visible and 3D tracking is enabled)
            * addVisionMeasurement to drivetrain odometry (See [Drive.addVisionMeasurement()](../drive/Drive.java))
