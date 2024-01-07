package frc.robot.commands.autonomouscommands;

import frc.robot.ConstPath;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.WaitCommand;


public class Center2Bal extends SequentialCommandGroup {

    public Center2Bal(SwerveSubsystem s_Swerve, ElevatorSubsystem elevator, ElbowSubsystem elbow,  ClawArmsSubsystem clawArms ) {
        addCommands(
            // Move arm to row 2 - "stuff on" and then open claws to release cone - no need towait for arm to settle
            new PrepareToPlaceRow2Auto(elevator, elbow , clawArms ),
//            new WaitCommand(0.75),    // Needed for autonomous only
            new OpenClawWithWait(clawArms),
   
            //    move arm home while driving towards 2nd object - cube
            Commands.parallel(
                new ElevatorPIDCmd(elevator, ElevatorConstants.kHomePosition), 
                new ElbowPIDCmd(elbow, ElbowConstants.kStartAngle,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance), 

                s_Swerve.PathDrive( ConstPath.blueCenterPathA1, true)
            ),
  
            s_Swerve.PathDrive( ConstPath.blueCenterPathA2, false),
 
            // move arm to pickup cone
            new ElbowPIDCmd(elbow, ElbowConstants.kRetreiveFromFloorAngleAuto,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance), 
  //          new PrepareToPickupFloorAuto(elevator, elbow, clawArms),
            
            
            // move forward a little to ensure we get it in the claw
            s_Swerve.PathDrive( ConstPath.blueCenterPathB, false),
            new PickupFloor(elevator, elbow, clawArms),

            // drive to Charge Station
            s_Swerve.PathDrive( ConstPath.blueCenterPathC, false),
            // if nec, stablize robot position based on pitch angle
            new ChargeStationBalanceDriveYaw0(s_Swerve)


        ) ;
    }
}