package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

//import edu.wpi.first.math.filter.Debouncer;
//import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.commands.*;
import frc.robot.commands.autonomouscommands.*;
//import frc.robot.commands.testcommands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;
//import frc.robot.ConstTrajCubic;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Controllers */
//    private final Joystick driverPS4Cont = new Joystick(0);
//    private final Joystick autoOprJoystick = new Joystick(1);
//    private final Joystick manualOprJoystick = new Joystick(2);
    private final Joystick driverLeftJoystick = new Joystick(0);
    private final Joystick driverRightJoystick = new Joystick(1);
    private final Joystick autoOprJoystick = new Joystick(2);

//    private final Joystick testOprJoystick = new Joystick(3);  // this was for testing purposes only

// Only using 1 limit switch
//    private final DigitalInput m_objectRetrievedLeft = new DigitalInput(1);
//    private final DigitalInput m_objectRetrievedRight = new DigitalInput(2);
 //   private final Trigger leftLimSwTrigger =  new Trigger(m_objectRetrievedLeft::get);
//    private final Trigger rightLimSwTrigger =  new Trigger(m_objectRetrievedRight::get);
 

    /* Subsystems */
    private final SwerveSubsystem s_Swerve = new SwerveSubsystem();
    private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    private final ElbowSubsystem m_elbow = new ElbowSubsystem();
    private final ClawArmsSubsystem m_clawArms = new ClawArmsSubsystem();
 

    // get instance of all auto and test commands

    // get instance of all test commands

    private final Command there1 = s_Swerve.PathDrive( ConstPath.newLeftPathA, true);
    private final Command there2 = s_Swerve.PathDrive( ConstPath.newLeftPathB, false);
    private final Command back = s_Swerve.PathDrive( ConstPath.newLeftPathC, false);


    private final Command center1BalNew = new Center1BalNew(s_Swerve, m_elevator, m_elbow, m_clawArms);
    private final Command newLeft2Bal = new NewLeft2Bal(s_Swerve, m_elevator, m_elbow, m_clawArms);
    private final Command newLeft2NoBal = new NewLeft2NoBal(s_Swerve, m_elevator, m_elbow, m_clawArms);
    private final Command newRight2Bal = new NewRight2Bal(s_Swerve, m_elevator, m_elbow, m_clawArms);
    private final Command newRight2NoBal = new NewRight2NoBal(s_Swerve, m_elevator, m_elbow, m_clawArms);
    private final Command zzznewLeft2Bal = new zzzNewLeft2Bal(s_Swerve, m_elevator, m_elbow, m_clawArms);
    private final Command zzznewLeft2NoBal = new zzzNewLeft2NoBal(s_Swerve, m_elevator, m_elbow, m_clawArms);
    private final Command zzznewRight2Bal = new zzzNewRight2Bal(s_Swerve, m_elevator, m_elbow, m_clawArms);
    private final Command zzznewRight2NoBal = new zzzNewRight2NoBal(s_Swerve, m_elevator, m_elbow, m_clawArms);



    private final Command blueLeft1Bal = new BlueLeft1Bal(s_Swerve, m_elevator, m_elbow, m_clawArms);
    private final Command blueLeft2Bal = new BlueLeft2Bal(s_Swerve, m_elevator, m_elbow, m_clawArms);
 //   private final Command blueLeft1NoBal = new BlueLeft1NoBal(s_Swerve, m_elevator, m_elbow, m_clawArms);
    private final Command blueLeft2NoBal = new BlueLeft2NoBal(s_Swerve, m_elevator, m_elbow, m_clawArms);

    private final Command center1Bal = new Center1Bal(s_Swerve, m_elevator, m_elbow, m_clawArms);
    private final Command center1BalOnly = new Center1BalOnly(s_Swerve, m_elevator, m_elbow, m_clawArms);
    private final Command center2Bal = new Center2Bal(s_Swerve, m_elevator, m_elbow, m_clawArms);
    private final Command center1NoBal = new Center1NoBal(s_Swerve, m_elevator, m_elbow, m_clawArms);
    private final Command center2NoBal = new Center2NoBal(s_Swerve, m_elevator, m_elbow, m_clawArms);
    private final Command sc1LeaveComm = new Sc1LeaveComm(s_Swerve, m_elevator, m_elbow, m_clawArms);

    private final Command blueRight1Bal = new BlueRight1Bal(s_Swerve, m_elevator, m_elbow, m_clawArms);
    private final Command blueRight2Bal = new BlueRight2Bal(s_Swerve, m_elevator, m_elbow, m_clawArms);
    private final Command blueRight2NoBal = new BlueRight2NoBal(s_Swerve, m_elevator, m_elbow, m_clawArms);



//    private final Command testPath1 =  s_Swerve.PathDrive (ConstPath.path1, true);
//    private final Command testPath2 = s_Swerve.PathDrive (ConstPath.path2, true);
//    private final Command testPath3 = s_Swerve.PathDrive (ConstPath.path3, true);
//    private final Command testPath1a = s_Swerve.PathDrive ( ConstPath.path1a, true);
//    private final Command testPath2a = s_Swerve.PathDrive ( ConstPath.path2a, true);
//    private final Command blueLeftPathA = s_Swerve.PathDrive ( ConstPath.blueLeftPathA, true);
//    private final Command blueLeftPathB = s_Swerve.PathDrive ( ConstPath.blueLeftPathB, true);
//    private final Command blueLeftPathC = s_Swerve.PathDrive ( ConstPath.blueLeftPathC, true);
//    private final Command blueLeftPathD = s_Swerve.PathDrive ( ConstPath.blueLeftPathD, true);
 

    // A chooser for autonomous commands
    SendableChooser<Command> autonomousChooser = new SendableChooser<>();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // Add commands to the autonomous command chooser to put on SmartDashboard
//        autonomousChooser.addOption("there1", there1);
//        autonomousChooser.addOption("there2", there2);
//        autonomousChooser.addOption("back", back);
        autonomousChooser.setDefaultOption("Center-1BalNew", center1BalNew);
 //       autonomousChooser.addOption("Center-1Bal", center1Bal);
        autonomousChooser.addOption("Center-1BalOnly", center1BalOnly);
 //       autonomousChooser.addOption("Center-2Bal", center2Bal);
 //       autonomousChooser.addOption("Center-2NoBal", center2NoBal);
 //       autonomousChooser.addOption("NEW Left -2Bal", newLeft2Bal);
        autonomousChooser.addOption("NEW Left -2NoBal", newLeft2NoBal);
 //       autonomousChooser.addOption("Left -1Bal", blueLeft1Bal);
        //       autonomousChooser.addOption("Left-1NoBal", blueLeft1NoBal);
        //       autonomousChooser.addOption("Left -2Bal", blueLeft2Bal);
        //       autonomousChooser.addOption("Left -2NoBal", blueLeft2NoBal);
 //       autonomousChooser.addOption("NEW Right -2Bal", newRight2Bal);
        autonomousChooser.addOption("NEW Right -2NoBal", newRight2NoBal);

        autonomousChooser.setDefaultOption("Score1-NoMove", center1NoBal);
        autonomousChooser.setDefaultOption("Score1-LeaveComm", sc1LeaveComm);


 //       autonomousChooser.addOption("Right-1Bal", blueRight1Bal);
 //       autonomousChooser.addOption("BluLt-1NoBal", blueLeft1NoBal);
//        autonomousChooser.addOption("Right-2Bal", blueRight2Bal);
//        autonomousChooser.addOption("Right-2NoBal", blueRight2NoBal);
//        autonomousChooser.addOption("zzz Left -2Bal", zzznewLeft2Bal);
//        autonomousChooser.addOption("zzz Left -2NoBal", zzznewLeft2NoBal);
//        autonomousChooser.addOption("zzz Right -2Bal", zzznewRight2Bal);
//        autonomousChooser.addOption("zzz Right -2NoBal", zzznewRight2NoBal);


        // Put the chooser on the dashboard
        SmartDashboard.putData(autonomousChooser);   

        
  
        // Put Some buttons on the SmartDashboard
        SmartDashboard.putData("PitchDrve", new ChargeStationBalanceDriveYaw0(s_Swerve));


        // Show what command your subsystem is running on the SmartDashboard
        SmartDashboard.putData(s_Swerve);
        SmartDashboard.putData(m_elevator);
        SmartDashboard.putData(m_elbow);
        SmartDashboard.putData(m_clawArms);


        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        // Driver Using PS4
/*
        // Driver Buttons
        final int translationAxis = PS4Controller.Axis.kLeftY.value;
        final int strafeAxis = PS4Controller.Axis.kLeftX.value;
        final int rotationAxis = PS4Controller.Axis.kRightX.value;
        final JoystickButton robotCentric = new JoystickButton(driverPS4Cont, PS4Controller.Button.kL1.value);

        final JoystickButton zeroGyro = new JoystickButton(driverPS4Cont, PS4Controller.Button.kTriangle.value);
        final JoystickButton slideLeft = new JoystickButton(driverPS4Cont, PS4Controller.Button.kL1.value);
        final JoystickButton slideRight = new JoystickButton(driverPS4Cont, PS4Controller.Button.kR1.value);

        // Connect the buttons to commands
        // Assign default swerve command
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
 //               () -> -driverPS4Cont.getRawAxis(translationAxis),         //was inverted?!?
 //               () -> -driverPS4Cont.getRawAxis(strafeAxis), 
                () -> driverPS4Cont.getRawAxis(translationAxis), 
                () -> driverPS4Cont.getRawAxis(strafeAxis), 
                () -> -driverPS4Cont.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        slideLeft.debounce(.1).onTrue(new SlideLeft(s_Swerve ));
        slideRight.debounce(.1).onTrue(new SlideRight(s_Swerve ));

*/


// Driver Using Joysticks
        // Driver Buttons

        final JoystickButton driveZeroGyro = new JoystickButton(driverRightJoystick, 1);
        final JoystickButton slideLeft = new JoystickButton(driverLeftJoystick, 6);
        final JoystickButton slideRight = new JoystickButton(driverLeftJoystick, 5);
        final JoystickButton zeroGyro = new JoystickButton(driverLeftJoystick, 10);
        final JoystickButton robotCentric = new JoystickButton(driverLeftJoystick, 11);

        final JoystickButton resetPose = new JoystickButton(driverLeftJoystick, 7);

        // Connect the buttons to commands
        // Assign default swerve command
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> driverLeftJoystick.getY(), 
                () -> driverLeftJoystick.getX(), 
                () -> -driverRightJoystick.getX(), 
                () -> robotCentric.getAsBoolean()
            )
        );

        driveZeroGyro.debounce(.1).whileTrue(            
            new TeleopSwerveZeroYaw(
                s_Swerve, 
                () -> driverLeftJoystick.getY(), 
                () -> driverLeftJoystick.getX(), 
                () -> -driverRightJoystick.getX(), 
                () -> robotCentric.getAsBoolean()
            )
        );
        resetPose.onTrue(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(new Translation2d(0,0),  Rotation2d.fromDegrees(0)))));
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        slideLeft.debounce(.1).onTrue(new SlideLeft(s_Swerve ));
        slideRight.debounce(.1).onTrue(new SlideRight(s_Swerve ));







        // Operator Buttons

//        final JoystickButton a1 = new JoystickButton(autoOprJoystick, 1);
        final JoystickButton a2 = new JoystickButton(autoOprJoystick, 2);
        final JoystickButton a3 = new JoystickButton(autoOprJoystick,3);
        final JoystickButton a4 = new JoystickButton(autoOprJoystick,4);
        final JoystickButton a5 = new JoystickButton(autoOprJoystick, 5);
        final JoystickButton a6 = new JoystickButton(autoOprJoystick, 6);
        final JoystickButton a7 = new JoystickButton(autoOprJoystick, 7);
 //       final JoystickButton a8 = new JoystickButton(autoOprJoystick, 8);
        final JoystickButton a9 = new JoystickButton(autoOprJoystick, 9);
        final JoystickButton a10 = new JoystickButton(autoOprJoystick, 10);
        final JoystickButton a11 = new JoystickButton(autoOprJoystick, 11);

        // Connect the buttons to commands


        a2.debounce(0.1).onTrue(new PickupShelf(m_elevator, m_elbow , m_clawArms ));
        a6.debounce(0.1).onTrue(new PickupShelfFar(m_elevator, m_elbow , m_clawArms ));

        a3.debounce(0.1).onTrue(new PrepareToPlaceRow2(m_elevator, m_elbow , m_clawArms ));
        a3.debounce(0.1).onFalse(new PlaceRow2(m_elevator, m_elbow , m_clawArms ));

        a4.debounce(0.1).onTrue(new DropRow1(m_elevator, m_elbow , m_clawArms ));

        a5.debounce(0.1).onTrue(new PrepareToPlaceRow3(m_elevator, m_elbow , m_clawArms ));
        a5.debounce(0.1).onFalse(new PlaceRow3(m_elevator, m_elbow , m_clawArms ));

        a7.debounce(0.1).onTrue(new PrepareToPickupFloor(m_elevator, m_elbow , m_clawArms ));
        a7.debounce(0.1).onFalse(new PickupFloor(m_elevator, m_elbow , m_clawArms ));

//        a7.and(leftLimSwTrigger.debounce(0.05)).onTrue(new PickupFloor(m_elevator, m_elbow, m_clawArms));
//        a7.(0.1).or(a7.and(leftLimSwTrigger.debounce(0.05))).onTrue(new PickupFloor(m_elevator, m_elbow, m_clawArms));

        a9.debounce(0.1).onTrue(new CloseClaw(m_clawArms));                               // only use on emergency!!!!      
        a10.debounce(0.1).onTrue(new OpenClaw(m_clawArms));                                // only use on emergency!!!!

        a11.debounce(0.1).onTrue(new Home(m_elevator, m_elbow , m_clawArms ));            // only use on emergency!!!!


//
//      Close claws automatically when both limit switches get activated together (presumably only by a cone or cube!)
//      leftLimSwTrigger.debounce(0.05).and(rightLimSwTrigger.debounce(0.05).onTrue(
//        leftLimSwTrigger.debounce(0.05).onTrue(
//        new CloseClawWithWait(m_clawArms).andThen(new Home(m_elevator, m_elbow , m_clawArms))) );



        
/*  Driver PS4
//        final JoystickButton m1 = new JoystickButton(manualOprJoystick, 1);
        final JoystickButton m2 = new JoystickButton(manualOprJoystick, 2);
        final JoystickButton m3 = new JoystickButton(manualOprJoystick, 3);
        final JoystickButton m4 = new JoystickButton(manualOprJoystick, 4);
        final JoystickButton m5 = new JoystickButton(manualOprJoystick, 5);
        final JoystickButton m6 = new JoystickButton(manualOprJoystick, 6);
        final JoystickButton m7 = new JoystickButton(manualOprJoystick, 7);
        final JoystickButton m8 = new JoystickButton(manualOprJoystick, 8);
        final JoystickButton m9 = new JoystickButton(manualOprJoystick, 9);
        final JoystickButton m10 = new JoystickButton(manualOprJoystick, 10);
        final JoystickButton m11 = new JoystickButton(manualOprJoystick, 11);
*/
        // Driver Joysticks
        final JoystickButton m1 = new JoystickButton(driverRightJoystick, 1);
        final JoystickButton m2 = new JoystickButton(driverRightJoystick, 2);
        final JoystickButton m3 = new JoystickButton(driverRightJoystick, 3);
        final JoystickButton m4 = new JoystickButton(driverRightJoystick, 4);
        final JoystickButton m5 = new JoystickButton(driverRightJoystick, 5);
        final JoystickButton m6 = new JoystickButton(driverRightJoystick, 6);
        final JoystickButton m7 = new JoystickButton(driverRightJoystick, 7);
        final JoystickButton m8 = new JoystickButton(driverRightJoystick, 8);
        final JoystickButton m9 = new JoystickButton(driverRightJoystick, 9);
        final JoystickButton m10 = new JoystickButton(driverRightJoystick, 10);
        final JoystickButton m11 = new JoystickButton(driverRightJoystick, 11);


        m1.onTrue(new Home(m_elevator, m_elbow , m_clawArms ));
        m2.whileTrue(new ManualElevatorUpDownCmd(m_elevator, ElevatorConstants.kDownSpeed));
        m3.whileTrue(new ManualElevatorUpDownCmd(m_elevator, ElevatorConstants.kUpSpeed));
        m4.whileTrue(new ManualElbowUpDownCmd(m_elbow, ElbowConstants.kDownSpeed));
        m5.whileTrue(new ManualElbowUpDownCmd(m_elbow, ElbowConstants.kUpSpeed));

        m6.onTrue(new ElevatorUpSmall(m_elevator));
        m7.onTrue(new ElevatorDownSmall(m_elevator));
        m8.onTrue(new CloseClaw(m_clawArms));
        m9.onTrue(new OpenClaw(m_clawArms));
        m10.onTrue(new ElbowDownSmall(m_elbow) );
        m11.onTrue(new ElbowUpSmall(m_elbow) );
 

/* 
//Test joystick, buttons and bindings
        final Joystick testOprJoystick = new Joystick(3);  // this was for testing purposes only
        final JoystickButton t1 = new JoystickButton(testOprJoystick, 1);
        final JoystickButton t2 = new JoystickButton(testOprJoystick, 2);
        final JoystickButton t3 = new JoystickButton(testOprJoystick,3);
        final JoystickButton t4 = new JoystickButton(testOprJoystick,4);
        final JoystickButton t5 = new JoystickButton(testOprJoystick, 5);
        final JoystickButton t6 = new JoystickButton(testOprJoystick, 6);
        final JoystickButton t7 = new JoystickButton(testOprJoystick, 7);
        final JoystickButton t8 = new JoystickButton(testOprJoystick, 8);
        final JoystickButton t9 = new JoystickButton(testOprJoystick, 9);
        final JoystickButton t10 = new JoystickButton(testOprJoystick, 10);
        final JoystickButton t11 = new JoystickButton(testOprJoystick, 11);

  //      t1.onTrue(testTrajChooser.getSelected());   
        
        t2.onTrue(new ElevatorPIDCmd(m_elevator, 5.0));
        t3.onTrue(new ElevatorPIDCmd(m_elevator, 15));
        t4.onTrue(new ElevatorPIDCmd(m_elevator, 25));
        t5.onTrue(new ElevatorPIDCmd(m_elevator, 45));
        t6.onTrue(new ElbowPIDCmd(m_elbow, -20, ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance));
        t7.onTrue(new ElbowPIDCmd(m_elbow, 0,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance));
        t8.onTrue(new ElbowPIDCmd(m_elbow, 30,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance));
        t9.onTrue(new ElbowPIDCmd(m_elbow, 45,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance));
        t10.onTrue(new ElbowPIDCmd(m_elbow, 60,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance));
        t11.onTrue(new ElbowPIDCmd(m_elbow, 90,  ElbowConstants.kMaxVel, ElbowConstants.kMaxAcc, ElbowConstants.kTolerance));


// End of Test Button Logic       */



    }

    /**
     * Use this to pass the  commands to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonomousChooser.getSelected();
    }
}
