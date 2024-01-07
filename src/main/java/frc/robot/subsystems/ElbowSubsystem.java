// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import frc.robot.Constants.ElbowConstants;

public class ElbowSubsystem extends SubsystemBase {

    private CANSparkMax elbowMotor = new CANSparkMax(ElbowConstants.kElbowMotorPort, MotorType.kBrushless);
    private RelativeEncoder elbowEncoder =  elbowMotor.getEncoder();//

    public ElbowSubsystem() {
        elbowEncoder.setPositionConversionFactor(ElbowConstants.kEncoderRotation2Degrees);
        elbowEncoder.setPosition(ElbowConstants.kZeroOffset);
        elbowMotor.setSoftLimit(SoftLimitDirection.kForward, ElbowConstants.kForwardSoftLimit);
        elbowMotor.setSoftLimit(SoftLimitDirection.kReverse, ElbowConstants.kReverseSoftLimit);
    }

    @Override
    public void periodic() {
       log();
    }

    
    public SparkMaxPIDController getIntegratedSparkPID( ) {
        return elbowMotor.getPIDController();
     }
 

    public void setMotor(double speed) {
        elbowMotor.set(speed);
     }


    /** Set the elevator motor to move in the up direction. */
    public void elbowUp() {
        elbowMotor.set(ElbowConstants.kUpSpeed);
     }
 
      /** Set the elevator motor to move in the down direction. */
      public void elbowDown() {
        elbowMotor.set(ElbowConstants.kDownSpeed);
      }
 
     /** Set the elevator motor to move in the up direction. */
     public void elevatorStop() {
        elbowMotor.set(0);
     }
 

    public double getEncoderDegrees() {
        return elbowEncoder.getPosition();
    }

// Moving the arm down increases the angle value
    public double getSmallDownAngle() {
       double temp = getEncoderDegrees();
      if ( (ElbowConstants.kMaxAngle -  temp) > ElbowConstants.kSmallMoveDegrees)
         return (temp + ElbowConstants.kSmallMoveDegrees );
      else return (ElbowConstants.kMaxAngle - 1.0) ;    // dont go past Max
   }
// Moving the arm up decreases the angle value
   public double getSmallUpAngle() {
    double temp = getEncoderDegrees();
    if ( (temp - ElbowConstants.kStartAngle) > ElbowConstants.kSmallMoveDegrees)
        return (temp - ElbowConstants.kSmallMoveDegrees );
    else return (ElbowConstants.kStartAngle) ;          // dont go past min / start angle
   }
   
   
    /** Return true when the claw Opened limit switch triggers. */
    public boolean isAtMax() {
//        return m_ElbowAtMaxAngle.get();
        return  ( (elbowEncoder.getVelocity() > 0.01) && (this.getEncoderDegrees() > ElbowConstants.kForwardSoftLimit)   );

    }
    /** Return true when the claw Closed limit switch triggers. */
    public boolean isAtMin() {
  //      return m_ElbowAtMinAngle.get();
        return  ( (elbowEncoder.getVelocity() < -0.01) && (this.getEncoderDegrees() < ElbowConstants.kReverseSoftLimit)   );
    }

    public void log() {
        SmartDashboard.putNumber("Elbow encoder degrees", getEncoderDegrees());
        SmartDashboard.putNumber("Elbow Temperature", getTemp());
    }
    
  /** Get Temp. */
  public double getTemp() {
    return elbowMotor.getMotorTemperature();
  }

}




