package frc.robot.subsystems.implementations.algae;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.Algae;

import frc.robot.io.interfaces.IntakeIO;
import frc.robot.io.interfaces.ArmIO;
import frc.robot.io.interfaces.ArmIOInputsAutoLogged;
import frc.robot.io.interfaces.IntakeIOInputsAutoLogged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase implements Algae  {

    public static class Constants {
        public static double pidAngleErrorInDegrees = 2.0;
    }

    private final ArmIO armIo;
    private final IntakeIO intakeIo;

    private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();
    private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();
    private ArmFeedforward feedforward =
      new ArmFeedforward(
          Algae.Constants.ffKs, Algae.Constants.ffKg, Algae.Constants.ffKv, Algae.Constants.ffKa);
    private double targetDegrees;

    private double kG, kV, kA, kS;
    private ArrayList<MechanismLigament2d> intakeLigament2d;
    private MechanismLigament2d armLigament2d;

    private int currentSimAngle = 0;

    public AlgaeSubsystem(IntakeIO intakeIo, ArmIO armIo) {
        this.armIo = armIo;
        this.intakeIo = intakeIo;

        kG = Algae.Constants.ffKg;
        kV = Algae.Constants.ffKv;
        kA = Algae.Constants.ffKa;
        kS = Algae.Constants.ffKs;
        
    }

    @Override
    public void periodic() {
        armIo.updateInputs(armInputs);
        intakeIo.updateInputs(intakeInputs);

        Logger.processInputs("Algae/Arm", armInputs);
        Logger.processInputs("Algae/Intake", intakeInputs);


        if (null != armLigament2d) {

            armLigament2d.setAngle(armInputs.positionDegrees);
        }

        if (intakeInputs.appliedVolts != 0) {
            currentSimAngle -= (intakeInputs.appliedVolts / 12) * 30;
      
            int angleOffset = 0;
            for (MechanismLigament2d ligament : intakeLigament2d) {
              ligament.setAngle(angleOffset + currentSimAngle);
              angleOffset += 90;
            }
        }
    }

    @Override
    public double getArmAngle() {
        return armInputs.positionDegrees;
    }

    @Override
    public double getArmTargetAngle() {
        return targetDegrees;
    }

    @Override
    public boolean isArmAtMaxLimit() {
        if (armInputs.positionDegrees >= Algae.Constants.maxArmAngleDegrees) {
            armInputs.limitHigh = true;
          } else {
            armInputs.limitHigh = false;
          }
          return armInputs.limitHigh;
    }

    @Override
    public boolean isArmAtMinLimit() {
        if (armInputs.positionDegrees <= Algae.Constants.minArmAngleDegrees) {
            armInputs.limitLow = true;
          } else {
            armInputs.limitLow = false;
          }
          return armInputs.limitLow;
    }

    @Override
    public double getArmVelocity() {
        return armInputs.velocityDegrees;
    }

    @Override
    public void setArmAngle(double degrees) {
        degrees =
            MathUtil.clamp(degrees, Algae.Constants.minArmAngleDegrees, Algae.Constants.maxArmAngleDegrees);
        targetDegrees = degrees;

        feedforward = new ArmFeedforward(kS, kG, kV, kA);
        double ffVolts = feedforward.calculate(targetDegrees, 0);

        armIo.setPosition(targetDegrees, ffVolts);
    }

    @Override
    public boolean isArmAtSetpoint() {
        return (Math.abs(armInputs.positionDegrees - targetDegrees)
        < AlgaeSubsystem.Constants.pidAngleErrorInDegrees);
    }

    @Override
    public Command getTurnOffIntakeCommand() {
        return runOnce(() -> turnOffIntake());
    }

    @Override
    public Command getTurnRightIntakeCommand() {
        return runOnce(() -> turnOnRightIntake());
    }

    @Override
    public Command getTurnLeftIntakeCommand() {
        return runOnce(() -> turnOnLeftIntake());
    }

    @Override
    public void setLigament(MechanismLigament2d armLigament2d, ArrayList<MechanismLigament2d> intakeLigament2d) {
        this.armLigament2d = armLigament2d;
        this.intakeLigament2d = intakeLigament2d;
    }
    
    @Override
    public void runVoltageArm(double volts) {
        armIo.setVoltage(volts);
    }

    @Override
    public void runVoltageIntake(double volts) {
        intakeIo.setVoltage(volts);
    }

    @Override
    public double getCurrentVoltageIntake() {
        return intakeInputs.appliedVolts;
    }
}
