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


// Move  Arm (ELev and Elbow) so that the arm is  in position to pickupan object (cone/Cube) from the floor
// Note that the actual grabbing/retreival of the object happens in the corresponding PickUpFloor Command
// This command is activated by an "onTrue" trigger. The retreival is activated by an "onFalse" of the same trigger

public class PrepareToPickupFloorAuto extends SequentialCommandGroup {
 
  public PrepareToPickupFloorAuto(ElevatorSubsystem elevator, ElbowSubsystem elbow, ClawArmsSubsystem clawArms ) {
    addCommands(

      // Start elbow out to not exceed max height rule
      new ElbowPIDCmd(elbow, 45.0,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance),      

        // Finish elbow movement while Elevator goes up and claw opens - ensure arm sweeps over cone/cube
        Commands.parallel(
          new ElbowPIDCmd(elbow, ElbowConstants.kRetreiveFromFloorAngle,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance),  // No Tolerance - keep PID active to try to keep elbowangle from moving    
          new ElevatorPIDCmd(elevator, ElevatorConstants.kFloorHeight+6),      
          new OpenClawWithWait(clawArms)
        ),
      // Lower elevator to pickup height
      new ElevatorPIDCmd(elevator, ElevatorConstants.kFloorHeight)      

      );
}}
