package frc.robot.commands.common.climber;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.interfaces.ClimberPrototype;;

public class ClimbCommand extends Command {
  ClimberPrototype climber;
  Supplier<Double> ccwSpeedSupplier;
  DoubleSupplier velocityRPM;
  Supplier<Double> cwSpeedSupplier;
  double targetVelocityRPM;

  public ClimbCommand(ClimberPrototype climber, Supplier<Double> cwSpeedSupplier)
  {
    this.climber = climber;
    this.cwSpeedSupplier = cwSpeedSupplier;
    addRequirements((Subsystem) climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    if (Constants.debugCommands) {
      System.out.println(
          "START: " + this.getClass().getSimpleName() + " velocity: " + velocityRPM.getAsDouble());
    }

    targetVelocityRPM = velocityRPM.getAsDouble();
    climber.setVolts(targetVelocityRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (cwSpeedSupplier.get() > 0) {
      climber.setVolts(cwSpeedSupplier.get()*24);
      System.out.println("Seting motor volts to: " + cwSpeedSupplier.get()*24);
    } else if (ccwSpeedSupplier.get() > 0) {
      climber.setVolts(-ccwSpeedSupplier.get()*24);
      System.out.println("Seting motor volts to: " + -ccwSpeedSupplier.get()*24);
    } else {
      climber.setVolts(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
    
}
