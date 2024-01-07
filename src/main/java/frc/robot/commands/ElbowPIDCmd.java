package frc.robot.commands;

//import edu.wpi.first.math.util.*;


//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElbowConstants;
import frc.robot.subsystems.ElbowSubsystem;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class ElbowPIDCmd extends CommandBase {
    private final ElbowSubsystem elbowSubsystem;
    private final SparkMaxPIDController pidController;
    private  double setpoint;
    private final double maxVel;
    private final double maxAcc;
    private double tolerance;

    public ElbowPIDCmd(ElbowSubsystem elbowSubsystem, double setpoint, double maxVel, double maxAcc, double tolerance) {
        this.elbowSubsystem = elbowSubsystem;
        this.pidController = elbowSubsystem.getIntegratedSparkPID();

        this.pidController.setP(ElbowConstants.kP); 
        this.pidController.setI(ElbowConstants.kI);
        this.pidController.setD(ElbowConstants.kD);
        this.pidController.setFF(ElbowConstants.kFF);
        this.pidController.setOutputRange(ElbowConstants.kMinOutput, ElbowConstants.kMaxOutput);
        this.setpoint = setpoint;
        this.tolerance = tolerance;
        this.maxVel = maxVel;
        this.maxAcc = maxAcc;
        
        int smartMotionSlot = 0;
       this.pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
       this.pidController.setSmartMotionMinOutputVelocity(ElbowConstants.kMinVel, smartMotionSlot);
       this.pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
       this.pidController.setSmartMotionAllowedClosedLoopError(ElbowConstants.kAllowedErr, smartMotionSlot);

        addRequirements(elbowSubsystem);
    }
    



    @Override
    public void initialize() {
 
        //       System.out.println("elbowPIDCmd started!");
 //       pidController.reset();
    }

    @Override
    public void execute() {
//        double speed = pidController.calculate(elbowSubsystem.getEncoderDegrees());
//        elbowSubsystem.setMotor(speed);
//       pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
        pidController.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
    }

    @Override
    public void end(boolean interrupted) {
        elbowSubsystem.setMotor(0);
//        System.out.println("elbowPIDCmd ended!");
    }

    @Override
    public boolean isFinished() {
        if (tolerance == 0) return false;
        else return (Math.abs((elbowSubsystem.getEncoderDegrees() - setpoint)) < tolerance);
    }
}
