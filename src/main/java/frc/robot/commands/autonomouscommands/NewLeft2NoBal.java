package frc.robot.commands.autonomouscommands;

import frc.robot.ConstPath;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class NewLeft2NoBal extends SequentialCommandGroup {

    public NewLeft2NoBal(SwerveSubsystem s_Swerve, ElevatorSubsystem elevator, ElbowSubsystem elbow,  ClawArmsSubsystem clawArms ) {
        addCommands(
            // Move arm to row 3and then open claws to release cone
            new PrepareToPlaceRow3Auto(elevator, elbow , clawArms ),
            new OpenClawWithWait(clawArms),
           
            //    start multistep back to home to not go above the required height rule, but also not hit row2 pole on the way back home
 //           Commands.parallel(
 //               new ElevatorPIDCmd(elevator, (ElevatorConstants.kRow2Height+5.0)), 
 //               new ElbowPIDCmd(elbow, ElbowConstants.kPlaceOnRow2Angle,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance)
 //               ), 

            //    finish multistep back to home to not go above the required height rule, but also not hit row2 pole on the way back home
            //    while driving towards 2nd object - cone
            Commands.parallel(
                new ElbowPIDCmd(elbow, -10,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance), 
                Commands.sequence(
                    new WaitCommand(0.25),    // Wait a little to move arm in
                    Commands.parallel(
                        new ElevatorPIDCmd(elevator, ElevatorConstants.kHomePosition), 
                        Commands.sequence(
                            new WaitCommand(0.55),    // Wait a little to move arm in -  was .25
                            s_Swerve.PathDrive( ConstPath.newLeftPathA, true)
                        )
                    )
                )
            ),
  
            // move arm to pickup cube
 //           new ElbowPIDCmd(elbow, ElbowConstants.kRetreiveFromFloorAngleAuto,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance), 
            new ElbowPIDCmd(elbow, ElbowConstants.kRetreiveFromFloorAngleAuto,  4000, 4000, ElbowConstants.kTolerance), 
  //          new PrepareToPickupFloorAuto(elevator, elbow, clawArms),
            
            // pickup cube while moving forward a little to ensure we get it in the claw
            s_Swerve.PathDrive( ConstPath.newLeftPathB, false),


            Commands.parallel(
                new PickupFloor(elevator, elbow, clawArms),
                Commands.sequence(
                    new WaitCommand(1.05),    // Wait - was .75
                    // drive to cube node
                    s_Swerve.PathDrive( ConstPath.newLeftPathC, false)
                    // move arm to row 3 and then release cone and bring arm back home
                )
            ),

            // Move arm to row 3and then open claws to release cone
            new PrepareToPlaceRow2Auto(elevator, elbow , clawArms ),
            new OpenClawWithWait(clawArms),
            
            new Home(elevator, elbow , clawArms)
            
        ) ;
    }
}