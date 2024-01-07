// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Place a held object can onto the corresponding node. */
public class DropRow1 extends SequentialCommandGroup {
 
  public DropRow1(ElevatorSubsystem elevator, ElbowSubsystem elbow, ClawArmsSubsystem clawArms ) {
    addCommands(

      new ElbowPIDCmd(elbow, ElbowConstants.kRetreiveFromShelfAngle,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance) ,

      new OpenClawWithWait(clawArms),

      Commands.parallel(
        new ElbowPIDCmd(elbow, ElbowConstants.kStartAngle,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance),
        new ElevatorPIDCmd(elevator, ElevatorConstants.kHomePosition)
      )
    );  
  }
}
