package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;       //Default for Ps4 controller - and new joystick
//    public static final double stickDeadband = 0.2;         // For HHS Joysticks which are old


public static double kPitchDriveFactor = 0.008; //TODO TA -  was 0085 - too much - this needs to be reduced - 0.01  too much
public static double kPitchDriveStopAngle = 7.5; //TODO TA -made changes, 5 too low, rocks alot was 9 ==8 too low and 10 too high??? - really?



    public static final class Swerve {
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: [DONE] L1 too far
      COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);    // TODO: TA - changed this
//      COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L1);    // TODO: TA - changed this
//        COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
//        public static final double trackWidth = Units.inchesToMeters(21.7); //TODO: [DONE] This must be tuned to specific robot
//        public static final double wheelBase = Units.inchesToMeters(21.7); //TODO: [DONE] This must be tuned to specific robot
        public static final double trackWidth = Units.inchesToMeters(21.8); //TODO: [DONE] This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(21.8); //TODO: [DONE] This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */

         /* Swerve Voltage Compensation */
         public static final double voltageComp = 12.0;

         /* Swerve Current Limiting */
         public static final int angleContinuousCurrentLimit = 20;
         public static final int driveContinuousCurrentLimit = 40; /*TODO: [DONE] Change this if needee */

        // public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 60;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        // public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
//        public static final double angleKP = 0.03; //TODO: [DONE] This must be tuned to specific robot - TA - tuned
        public static final double angleKP = 0.05; //TODO: [DONE] This must be tuned to specific robot - TA - tuned .1 too high
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
 //       public static final double driveKP = 0.05; //TODO: [DONE] This must be tuned to specific robot - TA - tuned
        public static final double driveKP = 0.8; //TODO: [DONE] This must be tuned to specific robot - TA - tuned
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE *, MOST LIEKLY NOT NEEDED FOR NEO motors*/
        public static final double driveKS = (0.12295); //TODO: [DONE] This must be tuned to specific robot - TA - works as is
        public static final double driveKV = (2.6405);
        public static final double driveKA = (0.40882);

        
        /* Drive Motor Conversion Factors */
        public static final double driveConversionPositionFactor =
        (wheelCircumference) / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        public static final double angleConversionFactor = 360.0 / angleGearRatio;


        /* Swerve Profiling Values */
        /** Meters per Second */
       public static final double maxSpeed = 4.5; //TODO: [TA-orig value] This must be tuned to specific robot
//     public static final double maxSpeed = 6.5; //TODO: [TA-try faster!] This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: [TA-Not Used] This must be tuned to specific robot

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kCoast;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: [DONE] This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(75.59);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: [DONE] This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(169.63);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: [DONE] This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(73.82);
 //           public static final Rotation2d angleOffset = Rotation2d.fromDegrees(259.28);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: [DONE] This must be tuned to specific robot
            public static final int driveMotorID = 8;       // when we rebuilt robot CAN IDs for thismodule got swapped
            public static final int angleMotorID = 7;
            public static final int canCoderID = 14;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(305.60);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        // The Charge station max Pitch angle is ~11 degrees
        // TODO:  <TA> Must calibrate this number - used for ChargeStationBalanceDrive
        public static double kPitchFactor = 0.09; // was 0.1
    }


//////////////////////////    AUTO     /////////////////////////////////////////

    public static final class AutoConstants { 

    }


//////////////////////////    ELEVATOR     /////////////////////////////////////////
    
    public static final class ElevatorConstants {
        public static final int kMotorPort1 = 22;
        public static final int kMotorPort2 = 21;
        
        public static final int kElevatorTopLimitSwPort = 1;
        public static final int kElevatorBotLimitSwPort = 2;

        public static final double kUpSpeed = 0.15;      // 0.5 too fast for now
        public static final double kDownSpeed = -0.05;

        // CanSparkMax Encoder native units is rotations. We convert to inches for the elevator
        // rotations * circumference of wheel (pi*d) / the gear ratio: 
        // d= sprocket diameter * pulley diameter, 1.757*1.041" and gear down is 5:1 
        // THere is a factor of 2 that I dont understand?!?
        public static final double kEncoderRotation2Inches = 1.0 * Math.PI * (1.757 * 1.041) / 5  * 1.925061;
 //   ****    Below are the PID values for Position Mode PID - DO not remove;
/*        public static final double kP = .2; 
//        public static final double kI = 0;
//        public static final double kD = 0;
//        public static final double kFF = 0.0;
//        public static final double kMinOutput = -0.15;
//        public static final double kMaxOutput = +0.25;
//        public static final double kMaxVel = 3000;
//        public static final double kMminVel = 0;
//        public static final double kMaxAcc = 1000;
//        public static final double kAllowedErr = 1;
*/  // ****    Above are the PID values for Position Mode PID - DO not remove;


// ****    Below are the PID values for Smart Motion Mode PID - DO not remove;
// ****    This is a Position based PID That uses a Trapazoidal Profile for movement
        public static final double kP = 0.00020; 
        public static final double kI = 0.0;
        public static final double kD = 0; 
        public static final double kIz = 0; 
 //       public static final double kFF = 0.000156; //TODO: I think this should be zero - must test
        public static final double kFF = 0.0; 
        public static final double kMaxOutput = 1; 
        public static final double kMinOutput = -1;
 //       public static final double maxRPM = 5700;
    // Smart Motion Coefficients
        public static final int smartMotionSlot = 0;
        public static final double kMinVel = 0;      // rpm
        public static final double kMaxVel = 4000;   // rpm
        public static final double kMaxAcc = 4000;
        public static final double kAllowedErr = 0;
        
// ****    Above are the PID values for Smart Motion Mode PID - DO not remove;


        public static final double kHomePosition = 0.25 ;       // Keep home very slightly above hard stop
 //       public static final double kRow1Height = 1 ;
        public static final double kRow2Height = 4.5 ;      //was 7
        public static final double kRow3Height = 45 ;       // was 45, but wanted higeer since elbow drifts down quickly
        public static final double kFloorHeight = 25 ;
        public static final double kShelfHeight = 4.55 ;       //was 5
        public static final double kShelfFarHeight = 5.0 ;       // was 5.5

//        public static final double kTolerance = 0.75 ;      // was 0.25 - to small
        public static final double kTolerance = 1.25 ;      // was 0.75 - need bigger now that we have wear and ear on elevator
        
        public static float kReverseSoftLimit = -1;
        public static float kForwardSoftLimit = 48;
 
        public static double kSmallMoveInches = 3;

        

 //       public static final double kRaisedPosition = 18/ kEncoderTick2Inches;
        

        public static final double kJoystickMaxSpeed = 0.5 ;
    }


    //////////////////////////    ELBOW     /////////////////////////////////////////

    public static final class ElbowConstants {
        public static final int kElbowMotorPort = 24;
        
        public static final int kElbowUpLimitSwPort = 3;
        public static final int kElbowDownLimitSwPort = 4;

        // there are 4096 ticks per revolution - this is the denominator -  CanSParkMax uses rotations not ticks
        // numerator is conversion to degrees taking into account the gear ratio (125:1 - was 100:1) and sprocket ratio (52:28)
//        public static final double kEncoderTick2Degrees = 1.0 / 4096.0 *  360 * 100 * 52/28;
//      public static final double kEncoderTick2Degrees = 1.0 / 4096.0 *  360 / 100 * 28/52;
        public static final double kEncoderRotation2Degrees = 1.0  *  360 / 125 * 28/52  * 1.3444444;
// The below Constansts are for the CanSparkMax Position PID - Not using now however
//       public static final double kP = .1; //was .2
//        public static final double kI = 0;
//        public static final double kD = 0;
//        public static final double kFF = 0.0;
//        public static final double kMinOutput = -0.50;
//        public static final double kMaxOutput = +0.50;

        // ****    Below are the PID values for Smart Motion Mode PID - DO not remove;
        // ****    This is a Position based PID That uses a Trapazoidal Profile for movement
        public static final double kP = 5e-5; 
        public static final double kI = 1e-6;   //TODO: I think this is correct
        public static final double kD = 0; 
        public static final double kIz = 0; 
        public static final double kFF = 0.000156; 
        public static final double kMaxOutput = 1; 
        public static final double kMinOutput = -1;
        //       public static final double maxRPM = 5700;
        // Smart Motion Coefficients
        public static final int smartMotionSlot = 0;
        public static final double kMinVel = 0;      // rpm
        public static final double kMaxVel = 3000;   // rpm
        public static final double kMaxAcc = 2000;      // TODO: TA - Optimize Max Vel and Acc (Acc was 3000 - seemed to fast)
        public static final double kAllowedErr = 0;

// ****    Above are the PID values for Smart Motion Mode PID - DO not remove;



        public static final double kZeroOffset = -27;
        public static final double kStartAngle = -22;
        public static final double kRetreiveFromFloorAngle = 175 ;
        public static final double kRetreiveFromFloorAngleAuto = 97 ;      //105 too low
        public static final double kRetreiveFromShelfAngle = -2.5; // 0 was too far out , -10 was too far in      
        public static final double kRetreiveFromShelfFarAngle = +5.5; //9.5 was too far out    
//        public static final double kPlaceOnRow1Angle = 165;
        public static final double kPlaceOnRow2Angle = 27.5 ;      // was 25, tried 20, but not out far enough
        public static final double kPlaceOnRow2AngleAuto = 31 ;      // was  33 try to stuff cone on in auto.
        public static final double kPlaceOnRow3Angle = 90 ;    

        public static float kReverseSoftLimit = -22;
        public static float kForwardSoftLimit = 180;

        public static float kSmallMoveDegrees = 5;
        public static final double kMaxAngle = (kForwardSoftLimit-3);


        public static final double kUpSpeed = 0.2;
        public static final double kDownSpeed = -0.2;

        public static final double kTolerance = 1.50;           // was 0.5 - to small


    }

    
//////////////////////////    CLAW     /////////////////////////////////////////

public static final class ClawConstants {

    public static int kOpenPort1 = 6;
    public static int kClosePort1 = 7;

    public static int kOpenPort2 = 10;
    public static int kClosePort2 = 11;


    public static final int kClawOpenedLimitSwPort = 5;
    public static final int kClawClosedLimitSwPort = 6;
    public static final int kClawObjectRetrievedLimitSwPort = 7;

    // standard encoder has 4096 ticks per revolution - we are using CANSparkMax however whose units are revolutions!
    // therefore conversion factor is just 1 revolution = 360 degrees / 100:1 gear ratio
    public static final double kEncoderRotation2Degrees = 1.0  *  360 / 100;
//       public static final double kP = 3;
//       public static final double kI = 0;
//       public static final double kD = 0.8;
    public static final double kP = .05;
    public static final double kI = 0;
    public static final double kD = 0.0;
    public static final double kFF = 0.0;
    public static final double kMinOutput = -0.23;
    public static final double kMaxOutput = +0.23;

    public static final double kOpenedAngle = -40;
    public static final double kClosedAngle = +2;
    public static final double kPIDClosedAngle = -10;
    public static double kReleaseTimeOut = 0.5;

    public static float kReverseSoftLimit = ((float)kOpenedAngle -5);
    public static float kForwardSoftLimit = ((float)kClosedAngle + 5);

    // arms moving speed
    public static final double kOpenSpeed = -0.23;  //0.1234 toofast
    public static final double kCloseSpeed = 0.23;

    // Wheel spinning speeds
    public static final double kRetrieveSpeed = 0.65;
    public static final double kPlaceSpeed = -0.25;
}


//////////////////////////    OI     /////////////////////////////////////////
    public static final class OIConstants {
        public static final int kDriverJoystickPort = 0;

        public static final int kArcadeDriveSpeedAxis = 1;
        public static final int kArcadeDriveTurnAxis = 3;

        public static final int kElevatorPIDRaiseButtonIdx = 1;
        public static final int kElevatorPIDLowerButtonIdx = 2;
        public static final int kElevatorJoystickRaiseButtonIdx = 3;
        public static final int kElevatorJoystickLowerButtonIdx = 4;
        public static final int kIntakeCloseButtonIdx = 5;
    }


















}






