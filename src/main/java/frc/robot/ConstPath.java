package frc.robot;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

public final class ConstPath {

    //TODO: <TA - need to check if still used>The below constants are used in the example auto, and must be tuned to specific robot
//    public static final double kMaxSpeedMetersPerSecond =  1.750*Math.PI;                     // .5 too slow
//    public static final double kMaxAccelerationMetersPerSecondSquared =  0.75*Math.PI;      //.25 too slow
    public static final double kMaxSpeedMetersPerSecond = 2.0*Math.PI;                     // .5 too slow
    public static final double kMaxAccelerationMetersPerSecondSquared =  1.0*Math.PI;      //.25 too slow
//          public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
//          public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 13;           // TA - 0.75 seems to give slight overshoot 
    public static final double kPYController = 10.0;           // TA - 0.75 seems to work 
    public static final double kPThetaController = 4.0;         // TA - 1.0 was too low  
//    public static final double kPXController = 14.7500;           // TA -  13.75 - may be a hair low
//    public static final double kPYController = 10.0;           // TA - not sure - had 10 - 12 too high
//    public static final double kPThetaController = 1.5;         // TA - 1.5 was too low  1.7 too high - not sure 


    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final         TrajectoryConfig kconfig =
            new TrajectoryConfig(
                    kMaxSpeedMetersPerSecond,
                    kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);
        
    public static final double kIn2M = 0.0254;

    public static final double kDeg2Rads = Math.PI / 180;



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                          another New     Left/Right Paths
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //   Autonomous path - Starting at Left Node going out to the Cube
    public static final PathPlannerTrajectory LeftPathA1 = PathPlanner.generatePath(
        new PathConstraints(4, 2.0), //was 5,3 &5,2.5 werent repeatable
        //                                 position,                heading(direction of travel     holonomic rotation ),       
        new PathPoint(new Translation2d(0 * kIn2M, 0 * kIn2M),      Rotation2d.fromDegrees(-10),    Rotation2d.fromDegrees(-180)), 
        new PathPoint(new Translation2d(60 * kIn2M, -16 * kIn2M),   Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(-180))
    );
    //   Autonomous path - Starting at Left Node going out to the Cube
    public static final PathPlannerTrajectory LeftPathA2 = PathPlanner.generatePath(
        new PathConstraints(4, 2.0), //was 5,3 &5,2.5 werent repeatable
        //                                 position,                heading(direction of travel     holonomic rotation ),       
        new PathPoint(new Translation2d(60 * kIn2M, -16 * kIn2M),   Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(-180)),
        new PathPoint(new Translation2d(60 * kIn2M, -16 * kIn2M),   Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(-1))
    );
    //   Autonomous path - Starting at Left Node going out to the Cube
    public static final PathPlannerTrajectory LeftPathA3 = PathPlanner.generatePath(
        new PathConstraints(4, 2.0), //was 5,3 &5,2.5 werent repeatable
        //                                 position,                heading(direction of travel     holonomic rotation ),       
        new PathPoint(new Translation2d(60 * kIn2M, -16 * kIn2M),   Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(-1)),
        new PathPoint(new Translation2d(160 * kIn2M, -16 * kIn2M),   Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(0))
    );

    //   Autonomous path - back to cube node
    public static final PathPlannerTrajectory LeftPathB1 = PathPlanner.generatePath(
        new PathConstraints(4, 2.0), 
        //                                 position,                heading(direction of travel),                holonomic rotation      
        new PathPoint(new Translation2d(160 * kIn2M, -16 * kIn2M), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0) ), 
        new PathPoint(new Translation2d(110 * kIn2M, -16 * kIn2M), Rotation2d.fromDegrees(180),  Rotation2d.fromDegrees(0) ) 
    );

    //   Autonomous path - back to cube node
    public static final PathPlannerTrajectory LeftPathB2 = PathPlanner.generatePath(
        new PathConstraints(4, 2.0), 
        //                                 position,                heading(direction of travel),                holonomic rotation      
        new PathPoint(new Translation2d(110 * kIn2M, -16 * kIn2M), Rotation2d.fromDegrees(180),  Rotation2d.fromDegrees(0) ), 
        new PathPoint(new Translation2d(110 * kIn2M, -16 * kIn2M), Rotation2d.fromDegrees(180),  Rotation2d.fromDegrees(-179) ) 
    );

    //   Autonomous path - back to cube node
    public static final PathPlannerTrajectory LeftPathB3 = PathPlanner.generatePath(
        new PathConstraints(4, 2.0), 
        //                                 position,                heading(direction of travel),                holonomic rotation      
        new PathPoint(new Translation2d(110 * kIn2M, -16 * kIn2M), Rotation2d.fromDegrees(180),  Rotation2d.fromDegrees(-179) ), 
        new PathPoint(new Translation2d(60 * kIn2M, -16 * kIn2M), Rotation2d.fromDegrees(180),  Rotation2d.fromDegrees(-180) ), 
        new PathPoint(new Translation2d(0 * kIn2M, 0 * kIn2M), Rotation2d.fromDegrees(180),  Rotation2d.fromDegrees(-180) ) 
    );

    //   Autonomous path - to the charge station center
    public static final PathPlannerTrajectory LeftPathC = PathPlanner.generatePath(
        new PathConstraints(3, 2.0), //maxVel was 1.15
        //                                 position,                heading(direction of travel),                holonomic rotation      
        new PathPoint(new Translation2d(160 * kIn2M, -16 * kIn2M), Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(0) ), 
        new PathPoint(new Translation2d(150 * kIn2M, -72 * kIn2M), Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(0) ), 
        new PathPoint(new Translation2d(76 * kIn2M, -80 * kIn2M),  Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0)) 
    );



    
    //   Autonomous path - Starting at Left Node going out to the Cube
    public static final PathPlannerTrajectory RightPathA1 = PathPlanner.generatePath(
        new PathConstraints(4, 2.0), //was 5,3 &5,2.5 werent repeatable
        //                                 position,                heading(direction of travel     holonomic rotation ),       
        new PathPoint(new Translation2d(0 * kIn2M, 0 * kIn2M),      Rotation2d.fromDegrees(10),    Rotation2d.fromDegrees(180)), 
        new PathPoint(new Translation2d(60 * kIn2M, 16 * kIn2M),   Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(180))
    );
    //   Autonomous path - Starting at Left Node going out to the Cube
    public static final PathPlannerTrajectory RightPathA2 = PathPlanner.generatePath(
        new PathConstraints(4, 2.0), //was 5,3 &5,2.5 werent repeatable
        //                                 position,                heading(direction of travel     holonomic rotation ),       
        new PathPoint(new Translation2d(60 * kIn2M, 16 * kIn2M),   Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(180)),
        new PathPoint(new Translation2d(60 * kIn2M, 16 * kIn2M),   Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(1))
    );
    //   Autonomous path - Starting at Left Node going out to the Cube
    public static final PathPlannerTrajectory RightPathA3 = PathPlanner.generatePath(
        new PathConstraints(4, 2.0), //was 5,3 &5,2.5 werent repeatable
        //                                 position,                heading(direction of travel     holonomic rotation ),       
        new PathPoint(new Translation2d(60 * kIn2M, 16 * kIn2M),   Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(1)),
        new PathPoint(new Translation2d(160 * kIn2M, 16 * kIn2M),   Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(0))
    );

    //   Autonomous path - back to cube node
    public static final PathPlannerTrajectory RightPathB1 = PathPlanner.generatePath(
        new PathConstraints(4, 2.0), 
        //                                 position,                heading(direction of travel),                holonomic rotation      
        new PathPoint(new Translation2d(160 * kIn2M, 16 * kIn2M), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0) ), 
        new PathPoint(new Translation2d(110 * kIn2M, 16 * kIn2M), Rotation2d.fromDegrees(180),  Rotation2d.fromDegrees(0) ) 
    );

    //   Autonomous path - back to cube node
    public static final PathPlannerTrajectory RightPathB2 = PathPlanner.generatePath(
        new PathConstraints(4, 2.0), 
        //                                 position,                heading(direction of travel),                holonomic rotation      
        new PathPoint(new Translation2d(110 * kIn2M, 16 * kIn2M), Rotation2d.fromDegrees(180),  Rotation2d.fromDegrees(0) ), 
        new PathPoint(new Translation2d(110 * kIn2M, 16 * kIn2M), Rotation2d.fromDegrees(180),  Rotation2d.fromDegrees(179) ) 
    );

    //   Autonomous path - back to cube node
    public static final PathPlannerTrajectory RightPathB3 = PathPlanner.generatePath(
        new PathConstraints(4, 2.0), 
        //                                 position,                heading(direction of travel),                holonomic rotation      
        new PathPoint(new Translation2d(110 * kIn2M, 16 * kIn2M), Rotation2d.fromDegrees(180),  Rotation2d.fromDegrees(179) ), 
        new PathPoint(new Translation2d(60 * kIn2M, 16 * kIn2M), Rotation2d.fromDegrees(180),  Rotation2d.fromDegrees(180) ), 
        new PathPoint(new Translation2d(0 * kIn2M, 0 * kIn2M), Rotation2d.fromDegrees(180),  Rotation2d.fromDegrees(180) ) 
    );

    //   Autonomous path - to the charge station center
    public static final PathPlannerTrajectory RightPathC = PathPlanner.generatePath(
        new PathConstraints(3, 2.0), //maxVel was 1.15
        //                                 position,                heading(direction of travel),                holonomic rotation      
        new PathPoint(new Translation2d(160 * kIn2M, 16 * kIn2M), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(0) ), 
        new PathPoint(new Translation2d(150 * kIn2M, 72 * kIn2M), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(0) ), 
        new PathPoint(new Translation2d(76 * kIn2M, 80 * kIn2M),  Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0)) 
    );




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                           New New      Left Paths
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //   Autonomous path - Starting at Left Node going out to the Cube
    public static final PathPlannerTrajectory newLeftPathA = PathPlanner.generatePath(
        new PathConstraints(4, 2.0), //was 5,3 &5,2.5 werent repeatable
        //                                 position,                heading(direction of travel     holonomic rotation ),       
        new PathPoint(new Translation2d(0 * kIn2M, 0 * kIn2M),      Rotation2d.fromDegrees(-10),    Rotation2d.fromDegrees(-180)), 
        new PathPoint(new Translation2d(60 * kIn2M, -10 * kIn2M),   Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(-1)),
        new PathPoint(new Translation2d(110 * kIn2M, -11 * kIn2M),   Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(-0)),
        new PathPoint(new Translation2d(145 * kIn2M, -11 * kIn2M),   Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(0))
    );

    //   Autonomous path - Pickup Cube moving a little forward
    public static final PathPlannerTrajectory newLeftPathB = PathPlanner.generatePath(
        new PathConstraints(1.5, 2.0), //was 2,2
        //                                 position,                heading(direction of travel     holonomic rotation ),       
        new PathPoint(new Translation2d(145 * kIn2M, -11 * kIn2M),  Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(0)),
        new PathPoint(new Translation2d(163 * kIn2M, -11 * kIn2M),   Rotation2d.fromDegrees(0),    Rotation2d.fromDegrees(0))
    );

    //   Autonomous path - back to cube node
    public static final PathPlannerTrajectory newLeftPathC = PathPlanner.generatePath(
        new PathConstraints(4, 2.0), 
        //                                 position,                heading(direction of travel),                holonomic rotation      
        new PathPoint(new Translation2d(163 * kIn2M, -11 * kIn2M), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0) ), 
        new PathPoint(new Translation2d(110 * kIn2M, -10 * kIn2M), Rotation2d.fromDegrees(180),  Rotation2d.fromDegrees(-179) ), 
        new PathPoint(new Translation2d(60 * kIn2M, -9 * kIn2M), Rotation2d.fromDegrees(180),  Rotation2d.fromDegrees(-180) ), 
        new PathPoint(new Translation2d(10 * kIn2M, -8 * kIn2M), Rotation2d.fromDegrees(180),  Rotation2d.fromDegrees(-180) ), 
        new PathPoint(new Translation2d(-4 * kIn2M, -7 * kIn2M), Rotation2d.fromDegrees(180),  Rotation2d.fromDegrees(-180) ) 
    );

    //   Autonomous path - to the charge station center
    public static final PathPlannerTrajectory newLeftPathD = PathPlanner.generatePath(
        new PathConstraints(3, 2.0), //maxVel was 1.15
        //                                 position,                heading(direction of travel),                holonomic rotation      
        new PathPoint(new Translation2d(163 * kIn2M, -9 * kIn2M), Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(0) ), 
        new PathPoint(new Translation2d(150 * kIn2M, -72 * kIn2M), Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(0) ), 
        new PathPoint(new Translation2d(76 * kIn2M, -80 * kIn2M),  Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0)) 
    );



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                           New New      Right Paths
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    //   Autonomous path - Starting at Blue Left Node going out to the Cube
    public static final PathPlannerTrajectory newRightPathA = PathPlanner.generatePath(
        new PathConstraints(4, 2.0), //was 5,3 &5,2.5 werent repeatable
        //                                 position,                heading(direction of travel     holonomic rotation ),       
        new PathPoint(new Translation2d(0 * kIn2M, 0 * kIn2M),      Rotation2d.fromDegrees(+10),    Rotation2d.fromDegrees(180)), 
        new PathPoint(new Translation2d(60 * kIn2M, 20 * kIn2M),   Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(1)),
        new PathPoint(new Translation2d(110 * kIn2M, 30 * kIn2M),   Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(0)),
        new PathPoint(new Translation2d(145 * kIn2M, 30 * kIn2M),   Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(0))
    );

    //   Autonomous path - Pickup Cube moving a little forward
    public static final PathPlannerTrajectory newRightPathB = PathPlanner.generatePath(
        new PathConstraints(1.5, 2.0), //was 2,2
        //                                 position,                heading(direction of travel     holonomic rotation ),       
        new PathPoint(new Translation2d(145 * kIn2M, 30 * kIn2M),  Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(0)),
        new PathPoint(new Translation2d(163 * kIn2M, 30 * kIn2M),   Rotation2d.fromDegrees(0),    Rotation2d.fromDegrees(0))
    );

    //   Autonomous path - back to cube node
    public static final PathPlannerTrajectory newRightPathC = PathPlanner.generatePath(
        new PathConstraints(4, 2.0), 
        //                                 position,                heading(direction of travel),                holonomic rotation      
        new PathPoint(new Translation2d(163 * kIn2M, 30 * kIn2M), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0) ), 
        new PathPoint(new Translation2d(110 * kIn2M, 18 * kIn2M), Rotation2d.fromDegrees(180),  Rotation2d.fromDegrees(179) ), 
        new PathPoint(new Translation2d(60 * kIn2M, 16 * kIn2M), Rotation2d.fromDegrees(180),  Rotation2d.fromDegrees(180) ), 
        new PathPoint(new Translation2d(10 * kIn2M, 12 * kIn2M), Rotation2d.fromDegrees(180),  Rotation2d.fromDegrees(180) ), 
        new PathPoint(new Translation2d(-4 * kIn2M, 8 * kIn2M), Rotation2d.fromDegrees(180),  Rotation2d.fromDegrees(180) ) 
    );

    //   Autonomous path - to the charge station center
    public static final PathPlannerTrajectory newRightPathD = PathPlanner.generatePath(
        new PathConstraints(3, 2.0), //maxVel was 1.15
        //                                 position,                heading(direction of travel),                holonomic rotation      
        new PathPoint(new Translation2d(163 * kIn2M, 30 * kIn2M), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(0) ), 
        new PathPoint(new Translation2d(150 * kIn2M, 72 * kIn2M), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(0) ), 
        new PathPoint(new Translation2d(76 * kIn2M, 80 * kIn2M),  Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0)) 
    );



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//          NEW  NEW               Center Path - 1 (score 1 in row 3) then move out of community come back to balance
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Use this path only to go over bridge out of community (use it as first path if want to go back and balance)
    public static final PathPlannerTrajectory Center1NewBalPathA = PathPlanner.generatePath(
//        new PathConstraints(1.15, 2.0), 
        new PathConstraints(2.0, 1.0), 
        //                                 position,                heading(direction of travel     holonomic rotation ),       
        new PathPoint(new Translation2d(0 * kIn2M, 0 * kIn2M),      Rotation2d.fromDegrees(0),  Rotation2d.fromDegrees(180)), 
        new PathPoint(new Translation2d(190 * kIn2M, 0 * kIn2M),  Rotation2d.fromDegrees(0),    Rotation2d.fromDegrees(180))
    );

                // Must use Center1NoBalPath first, then return with this path
    public static final PathPlannerTrajectory Center1NewBalPathB = PathPlanner.generatePath(
        new PathConstraints(2.0, 1.0), 
        //                                 position,                heading(direction of travel     holonomic rotation ),       
        new PathPoint(new Translation2d(190 * kIn2M, 0 * kIn2M),  Rotation2d.fromDegrees(180),    Rotation2d.fromDegrees(180)),
 // was 76 but with new batteries it seemed to always go too far.
        new PathPoint(new Translation2d(86 * kIn2M, 0 * kIn2M),   Rotation2d.fromDegrees(180),    Rotation2d.fromDegrees(180))
    );




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                           OLD      Left Paths
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //  Real Autonomous path - Starting at Blue Left Node going out to the Cube
    public static final PathPlannerTrajectory blueLeftPathA = PathPlanner.generatePath(
        new PathConstraints(kMaxSpeedMetersPerSecond/2, kMaxAccelerationMetersPerSecondSquared), 
        //                                 position,                heading(direction of travel     holonomic rotation ),       
        new PathPoint(new Translation2d(0 * kIn2M, 0 * kIn2M),      Rotation2d.fromDegrees(-10),    Rotation2d.fromDegrees(180)), 
        new PathPoint(new Translation2d(20 * kIn2M, -19 * kIn2M),  Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(179.99)),  // force clockwise turn
        new PathPoint(new Translation2d(110 * kIn2M, -19 * kIn2M),  Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(179.99)),  // force clockwise turn
        new PathPoint(new Translation2d(150 * kIn2M, -19 * kIn2M),   Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(0))
    );

    //  Real Autonomous path - Pickup Cube moving a little forward
    public static final PathPlannerTrajectory blueLeftPathB = PathPlanner.generatePath(
        new PathConstraints(kMaxSpeedMetersPerSecond/2, kMaxAccelerationMetersPerSecondSquared), 
        //                                 position,                heading(direction of travel     holonomic rotation ),       
        new PathPoint(new Translation2d(150 * kIn2M, -19 * kIn2M),  Rotation2d.fromDegrees(0),             Rotation2d.fromDegrees(0)),
        new PathPoint(new Translation2d(173 * kIn2M, -19 * kIn2M),   Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(0))
    );

    //  Real Autonomous path - back to cube node
    public static final PathPlannerTrajectory blueLeftPathC = PathPlanner.generatePath(
        new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared), 
        //                                 position,                heading(direction of travel),                holonomic rotation      
        new PathPoint(new Translation2d(173 * kIn2M, -19 * kIn2M), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0) ), 
        new PathPoint(new Translation2d(140 * kIn2M, -12 * kIn2M), Rotation2d.fromDegrees(175), Rotation2d.fromDegrees(0) ), 
        new PathPoint(new Translation2d(120 * kIn2M, -12 * kIn2M), Rotation2d.fromDegrees(180),  Rotation2d.fromDegrees(0) ) 
 //       new PathPoint(new Translation2d(-0 * kIn2M, -34 * kIn2M), Rotation2d.fromDegrees(-140),  Rotation2d.fromDegrees(0) ) 
    );

    //  Real Autonomous path - to the charge station center
    public static final PathPlannerTrajectory blueLeftPathD = PathPlanner.generatePath(
        new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared), 
        //                                 position,                heading(direction of travel),                holonomic rotation      
        new PathPoint(new Translation2d(173 * kIn2M, -19 * kIn2M), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0) ), 
        new PathPoint(new Translation2d(81 * kIn2M, -80 * kIn2M),  Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0)) 
    );


    

    //   Autonomous path - Left Side to the charge station center
    public static final PathPlannerTrajectory blueLeft1Bal = PathPlanner.generatePath(
        new PathConstraints(3, 3), 
        //                                 position,                heading(direction of travel),                holonomic rotation      
        new PathPoint(new Translation2d(0 * kIn2M, 0 * kIn2M),      Rotation2d.fromDegrees(+0),    Rotation2d.fromDegrees(180)), 
//        new PathPoint(new Translation2d(20 * kIn2M, -16 * kIn2M),  Rotation2d.fromDegrees(+0),     Rotation2d.fromDegrees(180)),  
        new PathPoint(new Translation2d(170 * kIn2M, -20 * kIn2M),  Rotation2d.fromDegrees(+0),     Rotation2d.fromDegrees(180)),  
        new PathPoint(new Translation2d(170 * kIn2M, -80 * kIn2M), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180)), 
        new PathPoint(new Translation2d(76 * kIn2M, -80 * kIn2M),  Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180)) 
    );




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                              OLD   Right Paths
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    //  Real Autonomous path - Starting at Blue Left Node going out to the Cube
    public static final PathPlannerTrajectory blueRightPathA = PathPlanner.generatePath(
        new PathConstraints(kMaxSpeedMetersPerSecond/2, kMaxAccelerationMetersPerSecondSquared), 
        //                                 position,                heading(direction of travel     holonomic rotation ),       
        new PathPoint(new Translation2d(0 * kIn2M, 0 * kIn2M),      Rotation2d.fromDegrees(10),    Rotation2d.fromDegrees(180)), 
        new PathPoint(new Translation2d(20 * kIn2M, 19 * kIn2M),  Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(179.99)),  // force clockwise turn
        new PathPoint(new Translation2d(110 * kIn2M, 19 * kIn2M),  Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(179.99)),  // force clockwise turn
        new PathPoint(new Translation2d(150 * kIn2M, 19 * kIn2M),   Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(0))
    );

    //  Real Autonomous path - Pickup Cube moving a little forward
    public static final PathPlannerTrajectory blueRightPathB = PathPlanner.generatePath(
        new PathConstraints(kMaxSpeedMetersPerSecond/2, kMaxAccelerationMetersPerSecondSquared), 
        //                                 position,                heading(direction of travel     holonomic rotation ),       
        new PathPoint(new Translation2d(150 * kIn2M, 19 * kIn2M),  Rotation2d.fromDegrees(0),             Rotation2d.fromDegrees(0)),
        new PathPoint(new Translation2d(173 * kIn2M, 19 * kIn2M),   Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(0))
    );

    //  Real Autonomous path - back to cube node
    public static final PathPlannerTrajectory blueRightPathC = PathPlanner.generatePath(
        new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared), 
        //                                 position,                heading(direction of travel),                holonomic rotation      
        new PathPoint(new Translation2d(173 * kIn2M, 19 * kIn2M), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0) ), 
        new PathPoint(new Translation2d(140 * kIn2M, 12 * kIn2M), Rotation2d.fromDegrees(-175), Rotation2d.fromDegrees(0) ), 
        new PathPoint(new Translation2d(120 * kIn2M, 12 * kIn2M), Rotation2d.fromDegrees(180),  Rotation2d.fromDegrees(0) ) 
 //       new PathPoint(new Translation2d(-0 * kIn2M, 34 * kIn2M), Rotation2d.fromDegrees(+140),  Rotation2d.fromDegrees(0) ) 
    );

    //  Real Autonomous path - to the charge station center
    public static final PathPlannerTrajectory blueRightPathD = PathPlanner.generatePath(
        new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared), 
        //                                 position,                heading(direction of travel),                holonomic rotation      
        new PathPoint(new Translation2d(173 * kIn2M, 16 * kIn2M), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0) ), 
        new PathPoint(new Translation2d(81 * kIn2M, 88 * kIn2M),  Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0)) 
    );


    

    //   Autonomous path - Left Side to the charge station center
    public static final PathPlannerTrajectory blueRight1Bal = PathPlanner.generatePath(
        new PathConstraints(3, 3), 
        //                                 position,                heading(direction of travel),                holonomic rotation      
        new PathPoint(new Translation2d(0 * kIn2M, 0 * kIn2M),      Rotation2d.fromDegrees(+0),    Rotation2d.fromDegrees(180)), 
//        new PathPoint(new Translation2d(20 * kIn2M, 16 * kIn2M),  Rotation2d.fromDegrees(+0),     Rotation2d.fromDegrees(180)),  
        new PathPoint(new Translation2d(170 * kIn2M, 20 * kIn2M),  Rotation2d.fromDegrees(+0),     Rotation2d.fromDegrees(180)),  
        new PathPoint(new Translation2d(170 * kIn2M, 80 * kIn2M), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180)), 
        new PathPoint(new Translation2d(78 * kIn2M, 80 * kIn2M),  Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180)) 
    );


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                         Center Paths - Pickup 2nd piece / option to balance (score first piece row 2 due to time constraints)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //  Real Autonomous path - Starting at Center Right Node going out to the Cube
    public static final PathPlannerTrajectory blueCenterPathA1 = PathPlanner.generatePath(
        new PathConstraints(2, 1), 
        //                                 position,                heading(direction of travel     holonomic rotation ),       
        new PathPoint(new Translation2d(0 * kIn2M, 0 * kIn2M),      Rotation2d.fromDegrees(-10),    Rotation2d.fromDegrees(180)), 
        new PathPoint(new Translation2d(88 * kIn2M, 0 * kIn2M),   Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(180))
    );

    //  Real Autonomous path - Starting at Center Right Node going out to the Cube
    public static final PathPlannerTrajectory blueCenterPathA2 = PathPlanner.generatePath(
        new PathConstraints(2, 1), 
        //                                 position,                heading(direction of travel     holonomic rotation ),       
        new PathPoint(new Translation2d(61 * kIn2M, 0 * kIn2M),      Rotation2d.fromDegrees(0),    Rotation2d.fromDegrees(180)), 
        new PathPoint(new Translation2d(150 * kIn2M, 0 * kIn2M),   Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(180))
    );

    //  Real Autonomous path - Starting at Center Right Node going out to the Cube
    public static final PathPlannerTrajectory blueCenterPathB = PathPlanner.generatePath(
        new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared), 
        //                                 position,                heading(direction of travel     holonomic rotation ),       
        new PathPoint(new Translation2d(150 * kIn2M, -2 * kIn2M),  Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
        new PathPoint(new Translation2d(173 * kIn2M, -2 * kIn2M),   Rotation2d.fromDegrees(0),     Rotation2d.fromDegrees(0))
    );

    //  Real Autonomous path - back to cube node
    public static final PathPlannerTrajectory blueCenterPathC = PathPlanner.generatePath(
        new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared), 
        //                                 position,                heading(direction of travel),                holonomic rotation      
        new PathPoint(new Translation2d(150 * kIn2M, 0 * kIn2M), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180) ), 
        new PathPoint(new Translation2d(86 * kIn2M, 0 * kIn2M), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180) ) 
    );

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                 Center Paths - 1 (score 1 in row 3) then move out of community / option to come back to balance
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Use this path only to go over bridge out of community (use it as first path if want to go back and balance)
    public static final PathPlannerTrajectory Center1NoBalPathA1 = PathPlanner.generatePath(
        new PathConstraints(kMaxSpeedMetersPerSecond*.5, kMaxAccelerationMetersPerSecondSquared), 
        //                                 position,                heading(direction of travel     holonomic rotation ),       
        new PathPoint(new Translation2d(0 * kIn2M, 0 * kIn2M),      Rotation2d.fromDegrees(0),  Rotation2d.fromDegrees(180)), 
        new PathPoint(new Translation2d(85 * kIn2M, 0 * kIn2M),  Rotation2d.fromDegrees(0),    Rotation2d.fromDegrees(180))
    );

    public static final PathPlannerTrajectory Center1NoBalPathA2 = PathPlanner.generatePath(
        new PathConstraints(kMaxSpeedMetersPerSecond*.5, kMaxAccelerationMetersPerSecondSquared/2), 
        //                                 position,                heading(direction of travel     holonomic rotation ),       
        new PathPoint(new Translation2d(85 * kIn2M, 0 * kIn2M),      Rotation2d.fromDegrees(0),  Rotation2d.fromDegrees(180)), 
        new PathPoint(new Translation2d(180 * kIn2M, 0 * kIn2M),  Rotation2d.fromDegrees(180),    Rotation2d.fromDegrees(180))
    );
    
                // Must use Center1NoBalPath first, then return with this path
    public static final PathPlannerTrajectory Center1BalPath = PathPlanner.generatePath(
        new PathConstraints(kMaxSpeedMetersPerSecond*.5, kMaxAccelerationMetersPerSecondSquared), 
        //                                 position,                heading(direction of travel     holonomic rotation ),       
        new PathPoint(new Translation2d(180 * kIn2M, 0 * kIn2M),  Rotation2d.fromDegrees(180),    Rotation2d.fromDegrees(180)),
        new PathPoint(new Translation2d(81 * kIn2M, 0 * kIn2M),   Rotation2d.fromDegrees(180),    Rotation2d.fromDegrees(180))
    );




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                               Old Blue Left Paths
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* 
    //  Real Autonomous path - Starting at Blue Left Node going out to along side of the Cube
    public static final PathPlannerTrajectory oldblueLeftPathA = PathPlanner.generatePath(
        new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared), 
        //                                 position,                heading(direction of travel     holonomic rotation ),       
        new PathPoint(new Translation2d(0 * kIn2M, 0 * kIn2M),      Rotation2d.fromDegrees(5),    Rotation2d.fromDegrees(180)), 
        new PathPoint(new Translation2d(106 * kIn2M, -10 * kIn2M),  Rotation2d.fromDegrees(5), Rotation2d.fromDegrees(180)),
        new PathPoint(new Translation2d(207 * kIn2M, 28 * kIn2M),   Rotation2d.fromDegrees(-2),     Rotation2d.fromDegrees(270))
    );

    //  Real Autonomous path - forward to get cube (with PickupFloor in parallel)
    public static final PathPlannerTrajectory oldblueLeftPathB = PathPlanner.generatePath(
        new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared), 
        //                                 position,                 heading(direction of travel),                 holonomic rotation    
        new PathPoint(new Translation2d(207 * kIn2M, 28 * kIn2M), Rotation2d.fromDegrees(-2), Rotation2d.fromDegrees(270)), 
        new PathPoint(new Translation2d(207 * kIn2M, 0 * kIn2M), Rotation2d.fromDegrees(-106), Rotation2d.fromDegrees(270) ) 
    );

    //  Real Autonomous path - back to cube node
    public static final PathPlannerTrajectory oldblueLeftPathC = PathPlanner.generatePath(
        new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared), 
        //                                 position,                heading(direction of travel),                holonomic rotation      
        new PathPoint(new Translation2d(207 * kIn2M, 0 * kIn2M), Rotation2d.fromDegrees(-106), Rotation2d.fromDegrees(270) ), 
        new PathPoint(new Translation2d(106 * kIn2M, -10 * kIn2M), Rotation2d.fromDegrees(175), Rotation2d.fromDegrees(180) ), 
        new PathPoint(new Translation2d(0 * kIn2M, -22 * kIn2M), Rotation2d.fromDegrees(-80), Rotation2d.fromDegrees(180) ) 
    );

    //  Real Autonomous path - to the charge station center
    public static final PathPlannerTrajectory oldblueLeftPathD = PathPlanner.generatePath(
        new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared), 
        //                                 position,                heading(direction of travel),                holonomic rotation      
        new PathPoint(new Translation2d(0 * kIn2M, -22 * kIn2M), Rotation2d.fromDegrees(-80), Rotation2d.fromDegrees(180)), 
        new PathPoint(new Translation2d(82 * kIn2M, -88 * kIn2M),  Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)) 
    );

*/


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                               Old Blue Right Paths
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*/
    //  Real Autonomous path - Starting at Blue Right Node going out to along side of the Cube
    public static final PathPlannerTrajectory oldblueRightPathA = PathPlanner.generatePath(
        new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared), 
        //                                 position,                heading(direction of travel),                holonomic rotation      
        new PathPoint(new Translation2d(0 * kIn2M, 0 * kIn2M), Rotation2d.fromDegrees(5), Rotation2d.fromDegrees(180)), 
        new PathPoint(new Translation2d(106 * kIn2M, 10 * kIn2M), Rotation2d.fromDegrees(5), Rotation2d.fromDegrees(180)),
        new PathPoint(new Translation2d(207 * kIn2M, 65 * kIn2M), Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(270))
    );

    //  Real Autonomous path - forward to get cube (with PickupFloor in parallel)
    public static final PathPlannerTrajectory oldblueRightPathB = PathPlanner.generatePath(
        new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared), 
        //                                 position,                heading(direction of travel),                holonomic rotation      
        new PathPoint(new Translation2d(207 * kIn2M, 65 * kIn2M), Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(270)), 
        new PathPoint(new Translation2d(207 * kIn2M, 35 * kIn2M), Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(270)) 
    );

    //  Real Autonomous path - back to cube node
    public static final PathPlannerTrajectory oldblueRightPathC = PathPlanner.generatePath(
        new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared), 
        //                                 position,                heading(direction of travel),                holonomic rotation      
        new PathPoint(new Translation2d(207 * kIn2M, 35 * kIn2M), Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(270)), 
        new PathPoint(new Translation2d(106 * kIn2M, 12 * kIn2M), Rotation2d.fromDegrees(175), Rotation2d.fromDegrees(180)), 
        new PathPoint(new Translation2d(0 * kIn2M, 22 * kIn2M),   Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(180)) 
    );

    //  Real Autonomous path - to the charge station center
    public static final PathPlannerTrajectory oldblueRightPathD = PathPlanner.generatePath(
        new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared), 
        //                                 position,                heading(direction of travel),                holonomic rotation      
        new PathPoint(new Translation2d(0 * kIn2M, 22 * kIn2M), Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(180)), 
        new PathPoint(new Translation2d(82 * kIn2M, 70 * kIn2M), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)) 
    );
*/







//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                Test Paths
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        
    // Simple Test path with no holonomic rotation. Stationary start/end. Max velocity and max accel of 2Pi 
    public static final PathPlannerTrajectory path1 = PathPlanner.generatePath(
        new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared), 
        //                                 position,              heading(direction of travel),       holonomic rotation
        new PathPoint(new Translation2d(0 * kIn2M, 0 * kIn2M), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)), 
        new PathPoint(new Translation2d(100 * kIn2M, 0 * kIn2M), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)) 
    );
    public static final PathPlannerTrajectory path1a = PathPlanner.generatePath(
        new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared), 
        //                                 position,              heading(direction of travel),       holonomic rotation
        new PathPoint(new Translation2d(0 * kIn2M, 0 * kIn2M), Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(180)), 
        new PathPoint(new Translation2d(0 * kIn2M, -100 * kIn2M), Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(180)) 
    );
    public static final PathPlannerTrajectory path3 = PathPlanner.generatePath(
        new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared), 
        //                                 position,              heading(direction of travel),       holonomic rotation
        new PathPoint(new Translation2d(0 * kIn2M, 0 * kIn2M), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)), 
        new PathPoint(new Translation2d(0 * kIn2M, 0 * kIn2M), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)) 
    );
        
    // Simple Test path with no holonomic rotation. Stationary start/end. Max velocity and max accel of 2Pi 
    public static final PathPlannerTrajectory path1b = PathPlanner.generatePath(
        new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared), 
        //                                 position,              heading(direction of travel),       holonomic rotation
        new PathPoint(new Translation2d(0 * kIn2M, 0 * kIn2M), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)), 
        new PathPoint(new Translation2d(140 * kIn2M, 0 * kIn2M), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)), 
        new PathPoint(new Translation2d(180 * kIn2M, 24 * kIn2M), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)) 
    );
     
        
    // Simple Test path with no holonomic rotation. Stationary start/end. Max velocity and max accel of 2Pi 
    public static final PathPlannerTrajectory path2 = PathPlanner.generatePath(
        new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared), 
        //                                 position,              heading(direction of travel),       holonomic rotation
        new PathPoint(new Translation2d(0 * kIn2M, 0 * kIn2M), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)), 
        new PathPoint(new Translation2d(140 * kIn2M, 0 * kIn2M), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)), 
        new PathPoint(new Translation2d(180 * kIn2M, 0 * kIn2M), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)) 
    );
        
    // Simple Test path with no holonomic rotation. Stationary start/end. Max velocity and max accel of 2Pi 
    public static final PathPlannerTrajectory path2a = PathPlanner.generatePath(
        new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared), 
        //                                 position,              heading(direction of travel),       holonomic rotation
        new PathPoint(new Translation2d(0 * kIn2M, -0 * kIn2M), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)), 
        new PathPoint(new Translation2d(140 * kIn2M, -0 * kIn2M), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)), 
        new PathPoint(new Translation2d(180 * kIn2M, -24 * kIn2M), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)) 
    );




    
}
