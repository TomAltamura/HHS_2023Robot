// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// Moves arm to appropriate location to place an object (cone or cube)
// Note that the actual release of the object happens in the corresponding PlaceRow3 Command
// This command is activated by an "onTrue" trigger. The release is activated by an "onFalse" of the same trigger
public class PrepareToPlaceRow3Auto extends SequentialCommandGroup {
 
  public PrepareToPlaceRow3Auto(ElevatorSubsystem elevator, ElbowSubsystem elbow, ClawArmsSubsystem clawArms ) {
    addCommands(

        //    do a multistep move to not go above the required height rule, first step move to Node 2 position
//        Commands.parallel(
//          new ElevatorPIDCmd(elevator, ElevatorConstants.kRow2Height), 
//          new ElbowPIDCmd(elbow, ElbowConstants.kPlaceOnRow2Angle,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance)    
//        ),

        //    do a multistep move to not go above the required height rule, next step move to Node 3 position
        Commands.parallel(
          new ElbowPIDCmd(elbow, ElbowConstants.kPlaceOnRow3Angle,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance),  // No Tolerance - keep PID active to try to keep elbowangle from moving     

          Commands.sequence(
            new WaitCommand(0.25),    // Wait for elbow to move to stay under max hright
            new ElevatorPIDCmd(elevator, ElevatorConstants.kRow3Height) 
          )
        )
    );  
  }
}
