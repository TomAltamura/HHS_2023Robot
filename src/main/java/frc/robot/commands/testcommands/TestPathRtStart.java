package frc.robot.commands.testcommands;

import frc.robot.ConstPath;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestPathRtStart extends SequentialCommandGroup {
    public TestPathRtStart(SwerveSubsystem s_Swerve, ElevatorSubsystem elevator, ElbowSubsystem elbow, ClawArmsSubsystem clawArms) {
        addCommands(
            new PrepareToPlaceRow3Auto(elevator, elbow , clawArms ),
            new PlaceRow3(elevator, elbow , clawArms ),
            s_Swerve.PathDrive( ConstPath.blueLeftPathA, true),
            new PrepareToPickupFloorAuto(elevator, elbow, clawArms),
            
            Commands.parallel(
                s_Swerve.PathDrive( ConstPath.blueLeftPathB, false),
                new PickupFloor(elevator, elbow, clawArms)
            ),

            s_Swerve.PathDrive( ConstPath.blueLeftPathC, false),
            new PrepareToPlaceRow3Auto(elevator, elbow , clawArms ),
            new PlaceRow3(elevator, elbow , clawArms )
        ) ;
    }
}



