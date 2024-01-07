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
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * Pickup an object (if one is between the open claws) and get it in a safe state to drive around.
 */
public class PickupShelf extends SequentialCommandGroup {
 
  public PickupShelf(ElevatorSubsystem elevator, ElbowSubsystem elbow,  ClawArmsSubsystem clawArms ) {
    addCommands(
      Commands.sequence(
        new OpenClawWithWait(clawArms),                                         // pneumatic claw needs time to open

        Commands.parallel(
          new ElevatorPIDCmd(elevator, ElevatorConstants.kShelfHeight),         // move elev to shelf pos - note that its faster than the elbow
          new ElbowPIDCmd(elbow, ElbowConstants.kRetreiveFromShelfAngle,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance)  ),    // move elbow to shelf pos - note that its slower than elev
          
        new WaitCommand(0.1),          //TODO: Calibrate time           // settle down arm - it rocks when stopped

        new CloseClawWithWait(clawArms),                                        // pneunatic claw needs time to close

        new ElevatorPIDCmd(elevator, (ElevatorConstants.kShelfHeight)+5.0),     // raise elev to ensure cone/cube does not hit shelf when returning arm

        new ElbowPIDCmd(elbow, ElbowConstants.kStartAngle,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance),                     // bring elbow back "home" - avoiding shelf

        new ElevatorPIDCmd(elevator, ElevatorConstants.kHomePosition)           // bring elev back "home" - finally clear to lower
        ) );
  }
}
