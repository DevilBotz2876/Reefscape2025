package frc.robot.io.implementations.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.io.interfaces.ElevatorIO;
import frc.robot.subsystems.interfaces.Elevator;

public class ElevatorIOStub implements ElevatorIO {
  private final double elevatorGearing = 10.0;
  private final double elevatorCarriageMassInKg = 4.0;
  private final double elevatorDrumRadius = Units.inchesToMeters(2.0);

  private final DCMotor elevatorGearbox = DCMotor.getNEO(1);

  private double currentVolts = 0.0;
  private double targetMeters = 0.0;

  // There is no PID or other closed-loop control here.  This class is meant to simulate real HW.
  // Some real HW does have closed-loop control available.  If we wanted to simulate that we could
  // add it here?

  private final ElevatorSim elevatorSim;

  public ElevatorIOStub() {

    this.elevatorSim =
        new ElevatorSim(
            elevatorGearbox,
            elevatorGearing,
            elevatorCarriageMassInKg,
            elevatorDrumRadius,
            Elevator.Constants.minPositionInMeters,
            Elevator.Constants.maxPositionInMeters,
            true,
            0,
            0.01,
            0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.appliedVolts = currentVolts;
    inputs.currentAmps = elevatorSim.getCurrentDrawAmps();
    inputs.positionMeters = elevatorSim.getPositionMeters();
    inputs.velocityInMetersPerSecond = elevatorSim.getVelocityMetersPerSecond();

    inputs.upperLimit = elevatorSim.hasHitUpperLimit();
    inputs.lowerLimit = elevatorSim.hasHitLowerLimit();

    elevatorSim.setInput(currentVolts);
    elevatorSim.update(0.020);
  }

  @Override
  public void setVoltage(double volts) {
    currentVolts = volts;
  }

  @Override
  public void setPosition(double meters) {
    targetMeters = meters;
  }
}
