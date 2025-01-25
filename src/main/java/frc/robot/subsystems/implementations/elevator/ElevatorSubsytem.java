package frc.robot.subsystems.implementations.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.Elevator;
import frc.robot.io.interfaces.ElevatorIO;
import frc.robot.io.interfaces.ElevatorIOInputsAutoLogged;

public class ElevatorSubsytem extends SubsystemBase implements Elevator{

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private MechanismLigament2d elevator2d = null;
    private final double elevatorLigament2dScale = 40;
    private final double elevatorLigament2doffset = 0.05;

    public ElevatorSubsytem(ElevatorIO io) {
        this.io = io;
        
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        if (null != elevator2d) {
            elevator2d.setLength((inputs.positionMeters + elevatorLigament2doffset) * elevatorLigament2dScale );
        }
    }

    @Override
    public double getPosition() {
        return inputs.positionMeters;
    }
 
    @Override
    public double getTargetPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTargetPosition'");
    }

    @Override
    public boolean isAtUpperLimit() {
        return inputs.upperLimit;
    }

    @Override
    public boolean isAtLowerLimit() {
        return inputs.lowerLimit;
    }

    @Override
    public double getVelocityPerSecond() {
        return inputs.velocityInMetersPerSecond;
    }

    @Override
    public void setPosition(double meters) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
    }

    @Override
    public void runVoltage(double volts) {
        io.setVoltage(volts);
    }

    @Override
    public boolean isAtSetpoint() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'isAtSetpoint'");
    }

    @Override
    public void add2dSim(Mechanism2d mech2d) {
        MechanismRoot2d elevatorRoot2d = mech2d.getRoot("Elevator",10, 5);
        elevator2d = elevatorRoot2d.append(new MechanismLigament2d("elevator", 5, 90, 10, new Color8Bit(Color.kLightSlateGray)));
    }
    
}
