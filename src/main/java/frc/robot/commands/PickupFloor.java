// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.Constants.*;
import frc.robot.subsystems.ClawArmsSubsystem;
//import frc.robot.subsystems.Wrist;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


// Pickup an object (if one is between the open claws) and get it in a safe state (home) to drive around.
// Note that the actual positioning of the arm happens in the corresponding PrepareToPickUpFloor Command
// This command is activated by an "onFalse" trigger. The arm positioning is activated first by an "onTrue" of the same trigger

public class PickupFloor extends SequentialCommandGroup {
 
  public PickupFloor(ElevatorSubsystem elevator, ElbowSubsystem elbow, ClawArmsSubsystem clawArms ) {
    addCommands(
      Commands.sequence(

        Commands.parallel(                                                // Close CLaw while sweeping elbow to pickup piece
          new CloseClawWithWait(clawArms)
 //         new ElbowPIDCmd(elbow, ElbowConstants.kPlaceOnRow3Angle,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance)
        ),

        Commands.parallel(                                                // Return elbow and elev to home positions
          new ElbowPIDCmd(elbow, ElbowConstants.kStartAngle,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance),  
          new ElevatorPIDCmd(elevator, ElevatorConstants.kHomePosition)
        )

      )
    );
  }
}
