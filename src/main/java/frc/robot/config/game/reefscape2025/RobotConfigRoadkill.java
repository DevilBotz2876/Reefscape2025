package frc.robot.config.game.reefscape2025;
import  frc.robot.commands.common.climber.ClimbCommand;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.io.implementations.motorcontroller.MotorControllerIOSparkMax;
import frc.robot.subsystems.implementations.climberprototype.ClimberSubsystem;



/* Override Roadkill specific constants here */
public class RobotConfigRoadkill extends RobotConfig {
  private final ClimberSubsystem climber = new ClimberSubsystem(new MotorControllerIOSparkMax(1));

  public RobotConfigRoadkill() {
    super(false, true, false);
    
    final CommandXboxController mainController = new CommandXboxController(0);
    // ClimbCommand climbCommand = new ClimbCommand((ClimberPrototype) climber, () -> mainController.getLeftY());

    climber.setDefaultCommand(new ClimbCommand(climber, () -> mainController.getLeftY()));
  }
}
