// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// Releases an object (cone or cube) and then moves arm back home
// Note that the actual positioning of the arm happens in the corresponding PlaceRow2 Command
// This command is activated by an "onFalse" trigger. The arm positioning is activated first by an "onTrue" of the same trigger
public class PlaceRow2 extends SequentialCommandGroup {
 
  public PlaceRow2(ElevatorSubsystem elevator, ElbowSubsystem elbow, ClawArmsSubsystem clawArms ) {
    addCommands(
      
      // Release Cube/cone
      new OpenClawWithWait(clawArms),

      // Move Elevator and Elbow back to starting (home) positions
      Commands.parallel(
        new ElevatorPIDCmd(elevator, ElevatorConstants.kHomePosition), 
        new ElbowPIDCmd(elbow, ElbowConstants.kStartAngle,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance)  
      ) 
    );  
  }
}
