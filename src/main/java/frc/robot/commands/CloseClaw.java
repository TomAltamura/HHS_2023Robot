// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ClawArmsSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Closes the claw until the limit switch is tripped. */
public class CloseClaw extends CommandBase {
  private final ClawArmsSubsystem m_claw;

  public CloseClaw(ClawArmsSubsystem claw) {
    m_claw = claw;
    addRequirements(m_claw);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    m_claw.close();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
//    return (m_claw.isGrabbing() || m_claw.isClosed());
    return ( m_claw.isClosed());
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    }
}
