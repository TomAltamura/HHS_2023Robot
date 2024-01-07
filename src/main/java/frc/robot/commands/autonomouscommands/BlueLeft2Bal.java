package frc.robot.commands.autonomouscommands;

import frc.robot.ConstPath;
//import frc.robot.Constants.ElbowConstants;
//import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.WaitCommand;


public class BlueLeft2Bal extends SequentialCommandGroup {

    public BlueLeft2Bal(SwerveSubsystem s_Swerve, ElevatorSubsystem elevator, ElbowSubsystem elbow,  ClawArmsSubsystem clawArms ) {
        addCommands(
            // Move arm to row 2 - "stuff on" and then open claws to release cone - no need towait for arm to settle
            new PrepareToPlaceRow2Auto(elevator, elbow , clawArms ),
//            new WaitCommand(0.75),    // Needed for autonomous only
            new OpenClawWithWait(clawArms),
    
            //    move arm back to home while driving towards 2nd object - cube
            Commands.parallel(
                new ElevatorPIDCmd(elevator, ElevatorConstants.kHomePosition), 
                new ElbowPIDCmd(elbow, ElbowConstants.kStartAngle,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance), 

                s_Swerve.PathDrive( ConstPath.blueLeftPathA, true)
            ),
  
            // move arm to pickup cube
            new ElbowPIDCmd(elbow, ElbowConstants.kRetreiveFromFloorAngleAuto,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance), 
  //          new PrepareToPickupFloorAuto(elevator, elbow, clawArms),
            
            // pickup cube while moving forward a little to ensure we get it in the claw
            s_Swerve.PathDrive( ConstPath.blueLeftPathB, false),
            new PickupFloor(elevator, elbow, clawArms),

            // drive onto charge station
            s_Swerve.PathDrive( ConstPath.blueLeftPathD, false),
            // if nec, stablize robot position based on pitch angle use "180" since robot is rotated 180 when going on charge station
            new ChargeStationBalanceDriveYaw0(s_Swerve)


        ) ;
    }
}