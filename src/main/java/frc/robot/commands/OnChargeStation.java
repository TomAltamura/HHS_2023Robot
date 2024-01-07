package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class OnChargeStation extends CommandBase {    
    private SwerveSubsystem s_Swerve;    

 //   private double doneDrivingCtr = 0;
    private double translationVal;

    public OnChargeStation(SwerveSubsystem s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        // if yaw is near 0 we need to leave polarity of pitch drive!!!!
        translationVal = s_Swerve.pitchDrive() * Constants.kPitchDriveFactor;
 //       if (translationVal == 0.0) doneDrivingCtr++;
//        if (doneDrivingCtr >= 3) translationVal = 0.0;
        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, 0.0).times(Constants.Swerve.maxSpeed), 
            0.0, 
            true, 
            true
        );
    }


    @Override
    public void initialize() {
//        doneDrivingCtr = 0;
//        System.out.println("ChargeStationBalanceDrive started!");
//      pidController.reset();
    }


    @Override
    public void end(boolean interrupted) {
 //       System.out.println("ChargeStationBalanceDrive ended!");
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(s_Swerve.getPitch() ) < 4.0);
 //       return false;
    }




}