package frc.robot.commands.autonomouscommands;

import frc.robot.ConstPath;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class zzzNewLeft2Bal extends SequentialCommandGroup {

    public zzzNewLeft2Bal(SwerveSubsystem s_Swerve, ElevatorSubsystem elevator, ElbowSubsystem elbow,  ClawArmsSubsystem clawArms ) {
        addCommands(
            // Move arm to row 3and then open claws to release cone
            new PrepareToPlaceRow3Auto(elevator, elbow , clawArms ),
            new OpenClawWithWait(clawArms),
           
            Commands.parallel(
                new ElbowPIDCmd(elbow, -20,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance), 
                Commands.sequence(
                    new WaitCommand(0.25),    // Wait a little to move arm in
                    Commands.parallel(
                        new ElevatorPIDCmd(elevator, ElevatorConstants.kHomePosition), 
                        s_Swerve.PathDrive( ConstPath.LeftPathA1, true)
                    )
                )
            ),
            // spin 180 degrees
            s_Swerve.PathDrive( ConstPath.LeftPathA2, false),


            // move forward while lowering arm to pickup cube
            Commands.parallel(
                new ElbowPIDCmd(elbow, ElbowConstants.kRetreiveFromFloorAngleAuto,  4000, 4000, ElbowConstants.kTolerance), 
                s_Swerve.PathDrive( ConstPath.LeftPathA3, false)
            ),


            Commands.parallel(
                new PickupFloor(elevator, elbow, clawArms),
                Commands.sequence(
                    new WaitCommand(0.75),    // Wait 
                    // drive to cube node
                    s_Swerve.PathDrive( ConstPath.LeftPathC, false)
                    // move arm to row 3 and then release cone and bring arm back home
                )
            ),

            // if nec, stablize robot position based on pitch angle
            new ChargeStationBalanceDriveYaw180(s_Swerve)

            
        ) ;
    }
}