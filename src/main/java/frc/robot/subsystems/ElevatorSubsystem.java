   // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private CANSparkMax elevatorMotor1 = new CANSparkMax(ElevatorConstants.kMotorPort1, MotorType.kBrushless);
    private RelativeEncoder elevatorEncoder =  elevatorMotor1.getEncoder();//
    private CANSparkMax elevatorMotor2 = new CANSparkMax(ElevatorConstants.kMotorPort2, MotorType.kBrushless);
 
    //private final DigitalInput m_ElevatorAtTop = new DigitalInput(ElevatorConstants.kElevatorTopLimitSwPort);
    //private final DigitalInput m_ElevatorAtBot = new DigitalInput(ElevatorConstants.kElevatorBotLimitSwPort);
  
    public ElevatorSubsystem() {
        elevatorMotor1.setInverted(true);
        elevatorMotor2.setInverted(true);
        elevatorMotor2.follow(elevatorMotor1, true);
        
        elevatorEncoder.setPositionConversionFactor(ElevatorConstants.kEncoderRotation2Inches);
        elevatorMotor1.setSoftLimit(SoftLimitDirection.kForward, ElevatorConstants.kForwardSoftLimit);
        elevatorMotor1.setSoftLimit(SoftLimitDirection.kReverse, ElevatorConstants.kReverseSoftLimit);

      }

    @Override
    public void periodic() {
       log();
    }

    public SparkMaxPIDController getIntegratedSparkPID( ) {
        return elevatorMotor1.getPIDController();
     }
    
    
    public void setMotor(double speed) {
       elevatorMotor1.set(speed);
    }


    /** Set the elevator motor to move in the up direction. */
    public void elevatorUp() {
       elevatorMotor1.set(ElevatorConstants.kUpSpeed);
    }

     /** Set the elevator motor to move in the down direction. */
     public void elevatorDown() {
        elevatorMotor1.set(ElevatorConstants.kDownSpeed);
     }

    /** Set the elevator motor to move in the up direction. */
    public void elevatorStop() {
        elevatorMotor1.set(0);
    }


    public double getEncoderInches() {
      return elevatorEncoder.getPosition() ;
  }

  public double getSmallUpLocation() {
   double temp = getEncoderInches();
   if ( (ElevatorConstants.kForwardSoftLimit -  temp) > ElevatorConstants.kSmallMoveInches)
      return (temp + ElevatorConstants.kSmallMoveInches );
   else return (ElevatorConstants.kForwardSoftLimit - 1.0) ;
}

public double getSmallDownLocation() {
   double temp = getEncoderInches();
   if ( (temp) > ElevatorConstants.kSmallMoveInches)
      return (temp - ElevatorConstants.kSmallMoveInches );
   else return (ElevatorConstants.kHomePosition) ;
}

    /** Return true when the claw Opened limit switch triggers. */
    public boolean isAtTop() {
 //       return m_ElevatorAtTop.get();
        return  ( (elevatorEncoder.getVelocity() > 0.01) && (this.getEncoderInches() > ElevatorConstants.kForwardSoftLimit)   );
    }

    /** Return true when the claw Closed limit switch triggers. */
    public boolean isAtBot() {
 //       return m_ElevatorAtBot.get();
        return  ( (elevatorEncoder.getVelocity() < -0.01) && (this.getEncoderInches() < ElevatorConstants.kReverseSoftLimit)   );
    }




    public void log() {
        SmartDashboard.putNumber("Elevator encoder inches", getEncoderInches());
        SmartDashboard.putNumber("Elevator motor1 temperature", getTemp1());
        SmartDashboard.putNumber("Elevator motor2 temperature", getTemp2());

      }
     /** Get Temp. */
      public double getTemp1() {
      return elevatorMotor1.getMotorTemperature();
   }
      public double getTemp2() {
         return elevatorMotor2.getMotorTemperature();
      }

}






