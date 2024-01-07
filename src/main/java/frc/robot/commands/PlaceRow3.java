// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// Releases an object (cone or cube) and then moves arm back home
// Note that the actual positioning of the arm happens in the corresponding PlaceRow3 Command
// This command is activated by an "onFalse" trigger. The arm positioning is activated first by an "onTrue" of the same trigger
public class PlaceRow3 extends SequentialCommandGroup {
 
  public PlaceRow3(ElevatorSubsystem elevator, ElbowSubsystem elbow, ClawArmsSubsystem clawArms ) {
    addCommands(

        // Release Cube/cone
        new OpenClawWithWait(clawArms),

      //    do a multistep back to home to not go above the required height rule, but also not hit row2 pole on the way back home
//      Commands.parallel(
//        new ElevatorPIDCmd(elevator, (ElevatorConstants.kRow2Height+5.0)), 
//       new ElbowPIDCmd(elbow, ElbowConstants.kPlaceOnRow2Angle,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance)
//     ), 

      //    do a multistep back to home to not go above the required height rule, but also not hit row2 pole on the way back home
      Commands.parallel(
        new ElbowPIDCmd(elbow, ElbowConstants.kStartAngle,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance),  

        Commands.sequence(
          new WaitCommand(0.25),    // Wait for charge station to be static / reset
          new ElevatorPIDCmd(elevator, ElevatorConstants.kHomePosition) 

        )

      )
    );  
  }
}
