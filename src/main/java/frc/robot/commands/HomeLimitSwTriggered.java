// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.ClawArmsSubsystem;
//import frc.robot.subsystems.Wrist;

import frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

//  ********************************************************************************************************
//   Returns arm to starting configuration position "home".
//   Note that this is only in an emergency "button push" command for the Operator
//   since the current Elbow and Elevator positions are unknown!!!!
//   This may cause the claw to hit the ground or exceed height restrictions - probably not the latter since the elevator is faster than elbow
//  ********************************************************************************************************
 

public class HomeLimitSwTriggered extends SequentialCommandGroup {
 
    public HomeLimitSwTriggered(ElevatorSubsystem elevator, ElbowSubsystem elbow, ClawArmsSubsystem clawArms ) {
      addCommands(
        Commands.parallel(
          new ElbowPIDCmd(elbow, ElbowConstants.kStartAngle,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance),
          new ElevatorPIDCmd(elevator, ElevatorConstants.kHomePosition)
        )
      );
    }
  }
  
