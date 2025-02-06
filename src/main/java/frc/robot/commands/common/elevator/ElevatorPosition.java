package frc.robot.commands.common.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.Elevator;

public class ElevatorPosition extends Command {
  private final Elevator elevator;
  private double position;

  public ElevatorPosition(Elevator elevator, double position) {
    this.elevator = elevator;
    this.position = position;

    addRequirements((Subsystem) elevator);
  }

  @Override
  public void initialize() {

    elevator.setPosition(position);
  }

  @Override
  public void execute() {}

  // This command will never finish.  When elevator reaches desired position we still want
  // closed-loop control running to maintain position.  If the command exits then default command
  // would run which might disable closed-loop or perform some other action.

}
