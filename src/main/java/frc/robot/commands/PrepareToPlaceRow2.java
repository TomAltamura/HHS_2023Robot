// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// Moves arm to appropriate location to place an object (cone or cube)
// Note that the actual release of the object happens in the corresponding PlaceRow2 Command
// This command is activated by an "onTrue" trigger. The release is activated by an "onFalse" of the same trigger
public class PrepareToPlaceRow2 extends SequentialCommandGroup {
 
  public PrepareToPlaceRow2(ElevatorSubsystem elevator, ElbowSubsystem elbow, ClawArmsSubsystem clawArms ) {
    addCommands(

      // Move Elevator and Elbow to Row2 positions 
      Commands.parallel(
        new ElevatorPIDCmd(elevator, ElevatorConstants.kRow2Height), 
        new ElbowPIDCmd(elbow, ElbowConstants.kPlaceOnRow2Angle,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc,  0)  // No Tolerance - keep PID active to try to keep elbowangle from moving     
      ) 
    );  
  }
}
