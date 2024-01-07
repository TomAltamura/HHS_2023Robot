// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ClawArmsSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CloseClawWithWait extends SequentialCommandGroup {
 
  public CloseClawWithWait( ClawArmsSubsystem clawArms ) {
    addCommands(
        new CloseClaw(clawArms),    //TODO: TA - Optimize  timeout
        new WaitCommand(0.25)    // .5 sec too long

    );
  }
}
