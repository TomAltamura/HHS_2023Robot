// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import edu.wpi.first.wpilibj.Solenoid;

import frc.robot.Constants.ClawConstants;

public class ClawArmsSubsystem extends SubsystemBase {

  // ******* Default CAN ID of Pneumatic Hub is 1!!! if using another CAN ID its set/used via 1st parameter in the parameter list.
  // like this - private final DoubleSolenoid m_clawSolenoid1 = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, ClawConstants.kOpenPort1, ClawConstants.kClosePort1);
  //private final DoubleSolenoid m_clawSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClawConstants.kOpenPort1, ClawConstants.kClosePort1);
  private final DoubleSolenoid m_clawSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClawConstants.kOpenPort2, ClawConstants.kClosePort2);

  private final Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);


  /** Create a new claw subsystem. */
  public ClawArmsSubsystem() {
        
 }

 public void log() {
 //   SmartDashboard.putData("Retrieved LimSw", m_objectRetrieved);
//  SmartDashboard.putData("ClawArm1 state", m_clawSolenoid1);
  SmartDashboard.putNumber("Air Pressure", phCompressor.getPressure());
 //    SmartDashboard.putData("ClawArm2 state", m_clawSolenoid2);
  }


  /** Set the claw motor to move in the open direction. */
  public void open() {
    m_clawSolenoid1.set(DoubleSolenoid.Value.kForward);
//    m_clawSolenoid2.set(DoubleSolenoid.Value.kForward);
  }

  /** Set the claw motor to move in the close direction. */
  public void close() {
    m_clawSolenoid1.set(DoubleSolenoid.Value.kReverse);
//    m_clawSolenoid2.set(DoubleSolenoid.Value.kReverse);
  }


 /** Return true when the robot is grabbing an object hard enough to trigger the limit switch. */
  //public boolean isGrabbing() {
 //   return m_objectRetrieved.get();
 //   return getDebouncedLeft()  || getDebouncedRight();
 // }

  /** Return true when the claw Opened limit switch triggers. */
  public boolean isOpened() {
    return (m_clawSolenoid1.get() == DoubleSolenoid.Value.kForward);  

  }
  /** Return true when the claw Closed limit switch triggers. */
  public boolean isClosed() {
    return (m_clawSolenoid1.get() == DoubleSolenoid.Value.kReverse);  
  }

  /** Call log method every loop. */
  @Override
  public void periodic() {
    log();
  }
}
