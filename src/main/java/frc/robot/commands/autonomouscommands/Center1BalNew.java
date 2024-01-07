package frc.robot.commands.autonomouscommands;

import frc.robot.ConstPath;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class Center1BalNew extends SequentialCommandGroup {

    public Center1BalNew(SwerveSubsystem s_Swerve, ElevatorSubsystem elevator, ElbowSubsystem elbow,  ClawArmsSubsystem clawArms ) {
        addCommands(
            // Move arm to row 3and then open claws to release cone
            new PrepareToPlaceRow3Auto(elevator, elbow , clawArms ),
            new OpenClawWithWait(clawArms),

            //    start multistep back to home to not go above the required height rule, but also not hit row2 pole on the way back home
//            Commands.parallel(
//                new ElevatorPIDCmd(elevator, (ElevatorConstants.kRow2Height+5.0)), 
//                new ElbowPIDCmd(elbow, ElbowConstants.kPlaceOnRow2Angle,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance)
//            ), 
    
            //    finish multistep back to home to not go above the required height rule, but also not hit row2 pole on the way back home
            //    while driving towards 2nd object - cube
            Commands.parallel(
                new ElevatorPIDCmd(elevator, ElevatorConstants.kHomePosition), 
                new ElbowPIDCmd(elbow, ElbowConstants.kStartAngle,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance), 

                Commands.sequence(
                    new WaitCommand(0.25),    // Wait for arm to come in
                    s_Swerve.PathDrive( ConstPath.blueCenterPathA1, true)
                )
            ),

            s_Swerve.PathDrive( ConstPath.blueCenterPathA2, true),

            new WaitCommand(0.25),    // Wait for charge station to be static / reset

            s_Swerve.PathDrive( ConstPath.blueCenterPathC, false),
            new WaitCommand(0.05),    // Slight pause to wait for charge station angle change to calm ROBOT rocking in balance drive
            new ChargeStationBalanceDriveYaw0(s_Swerve)


        ) ;
    }
}