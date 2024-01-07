package frc.robot.commands.testcommands;

import frc.robot.ConstPath;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestPath1 extends SequentialCommandGroup {
    public TestPath1(SwerveSubsystem s_Swerve) {
        addCommands(
                s_Swerve.PathDrive( ConstPath.path1, true)
        ) ;
    }
}