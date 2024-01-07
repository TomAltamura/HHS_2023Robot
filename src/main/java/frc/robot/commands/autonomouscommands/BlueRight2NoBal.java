package frc.robot.commands.autonomouscommands;

import frc.robot.ConstPath;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class BlueRight2NoBal extends SequentialCommandGroup {

    public BlueRight2NoBal(SwerveSubsystem s_Swerve, ElevatorSubsystem elevator, ElbowSubsystem elbow,  ClawArmsSubsystem clawArms ) {
        addCommands(
            // Move arm to row 3and then open claws to release cone
            new PrepareToPlaceRow3Auto(elevator, elbow , clawArms ),
            new OpenClawWithWait(clawArms),
 
            
            //    start multistep back to home to not go above the required height rule, but also not hit row2 pole on the way back home
            Commands.parallel(
                new ElevatorPIDCmd(elevator, (ElevatorConstants.kRow2Height+5.0)), 
                new ElbowPIDCmd(elbow, ElbowConstants.kPlaceOnRow2Angle,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance)
                ), 

            //    move arm back to home while driving towards 2nd object - cube
            Commands.parallel(
                new ElevatorPIDCmd(elevator, ElevatorConstants.kHomePosition), 
                new ElbowPIDCmd(elbow, ElbowConstants.kStartAngle,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance), 

                s_Swerve.PathDrive( ConstPath.blueRightPathA, true)
            ),
  
            // move arm to pickup cube
            new ElbowPIDCmd(elbow, ElbowConstants.kRetreiveFromFloorAngleAuto,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance), 
  //          new PrepareToPickupFloorAuto(elevator, elbow, clawArms),
            
            // pickup cube while moving forward a little to ensure we get it in the claw
            s_Swerve.PathDrive( ConstPath.blueRightPathB, false),
            new PickupFloor(elevator, elbow, clawArms),

            // drive to cube node
            s_Swerve.PathDrive( ConstPath.blueRightPathC, false)
            // move arm to row 3 and then release cone and bring arm back home


        ) ;
    }
}