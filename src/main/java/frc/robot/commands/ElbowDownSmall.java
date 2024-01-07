package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElbowConstants;
import frc.robot.subsystems.ElbowSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

public class ElbowDownSmall extends CommandBase {
private final ElbowSubsystem elbowSubsystem;
private final SparkMaxPIDController pidController;
private  double setpoint;


  public ElbowDownSmall(ElbowSubsystem elbowSubsystem) {
    this.elbowSubsystem = elbowSubsystem;
    this.pidController = elbowSubsystem.getIntegratedSparkPID();

    this.pidController.setP(ElbowConstants.kP); 
    this.pidController.setI(ElbowConstants.kI);
    this.pidController.setD(ElbowConstants.kD);
    this.pidController.setFF(ElbowConstants.kFF);
    this.pidController.setOutputRange(ElbowConstants.kMinOutput, ElbowConstants.kMaxOutput);
    
    int smartMotionSlot = 0;
    this.pidController.setSmartMotionMaxVelocity(ElbowConstants.kMaxVel, smartMotionSlot);
    this.pidController.setSmartMotionMinOutputVelocity(ElbowConstants.kMinVel, smartMotionSlot);
    this.pidController.setSmartMotionMaxAccel(ElbowConstants.kMaxAcc, smartMotionSlot);
    this.pidController.setSmartMotionAllowedClosedLoopError(ElbowConstants.kAllowedErr, smartMotionSlot);

    addRequirements(elbowSubsystem);
  }
  
  @Override
  public void initialize() {
 //   System.out.println("ElbowDownSmall started!");
    setpoint = elbowSubsystem.getEncoderDegrees() - ElbowConstants.kSmallMoveDegrees;
    if(setpoint < ElbowConstants.kStartAngle ) setpoint = ElbowConstants.kStartAngle;
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
//    System.out.println("ElbowDownSmall ended!");
  }

  @Override
  public boolean isFinished() {
    return (Math.abs((elbowSubsystem.getEncoderDegrees() - setpoint)) < ElbowConstants.kTolerance);
  }
}
