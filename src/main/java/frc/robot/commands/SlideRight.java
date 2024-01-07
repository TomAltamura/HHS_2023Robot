package frc.robot.commands;

import frc.robot.ConstTraj;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SlideRight extends SequentialCommandGroup {
    public SlideRight(SwerveSubsystem s_Swerve) {
        addCommands(
                new TrajectoryDriving (s_Swerve, ConstTraj.slideRight)
        ) ;
    }
}