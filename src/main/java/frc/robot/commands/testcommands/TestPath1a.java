package frc.robot.commands.testcommands;

import frc.robot.ConstPath;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestPath1a extends SequentialCommandGroup {
    public TestPath1a(SwerveSubsystem s_Swerve) {
        addCommands(
            s_Swerve.PathDrive( ConstPath.path1a, true)
        ) ;
    }
}