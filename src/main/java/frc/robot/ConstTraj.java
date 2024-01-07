package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public final class ConstTraj {

          //TODO: <TA - need to check if still used>The below constants are used in the example auto, and must be tuned to specific robot
          public static final double kMaxSpeedMetersPerSecond = 3;
          public static final double kMaxAccelerationMetersPerSecondSquared = 3;
//          public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
//          public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
          public static final double kMaxAngularSpeedRadiansPerSecond = 2*Math.PI;
          public static final double kMaxAngularSpeedRadiansPerSecondSquared = 2*Math.PI;
          
          public static final double kPXController = 1;
          public static final double kPYController = 1;
          public static final double kPThetaController = 1;
  
  
    
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




// was a 4.5 inch slide, but the team felt this was too big of a move - changed to 3 inch
            public static final Trajectory slideLeft = TrajectoryGenerator.generateTrajectory ( List.of( 
                new Pose2d(0 * kIn2M, 0 * kIn2M, new Rotation2d(0)),
//                new Pose2d(0 * kIn2M, -2 * kIn2M, new Rotation2d(0)),
                new Pose2d(0 * kIn2M, 3.0 * kIn2M, new Rotation2d(0)) ),
            kconfig );

            public static final Trajectory slideRight = TrajectoryGenerator.generateTrajectory ( List.of( 
                new Pose2d(0, 0, new Rotation2d(0)),
//                new Pose2d(0 * kIn2M, -2 * kIn2M, new Rotation2d(0)),
                new Pose2d(0, -3.0 * kIn2M, new Rotation2d(0)) ),
            kconfig );







    
}
