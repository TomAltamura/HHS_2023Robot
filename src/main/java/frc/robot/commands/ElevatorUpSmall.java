package frc.robot.commands;

//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class ElevatorUpSmall extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;
//    private final PIDController pidController;
    private final SparkMaxPIDController pidController;
    private double setpoint;

    public ElevatorUpSmall(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;

        this.pidController = elevatorSubsystem.getIntegratedSparkPID();

        this.pidController.setP(ElevatorConstants.kP); 
        this.pidController.setI(ElevatorConstants.kI);
        this.pidController.setD(ElevatorConstants.kD);
        this.pidController.setFF(ElevatorConstants.kFF);
        this.pidController.setOutputRange(ElevatorConstants.kMinOutput, ElevatorConstants.kMaxOutput);

        int smartMotionSlot = 0;
        this.pidController.setSmartMotionMaxVelocity(ElevatorConstants.kMaxVel, smartMotionSlot);
        this.pidController.setSmartMotionMinOutputVelocity(ElevatorConstants.kMinVel, smartMotionSlot);
        this.pidController.setSmartMotionMaxAccel(ElevatorConstants.kMaxAcc, smartMotionSlot);
        this.pidController.setSmartMotionAllowedClosedLoopError(ElevatorConstants.kAllowedErr, smartMotionSlot);


        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
//        System.out.println("ElevatorUpSmall started!");
        double startPoint = elevatorSubsystem.getEncoderInches(); 
        setpoint = startPoint + ElevatorConstants.kSmallMoveInches  ;
        if (setpoint > ElevatorConstants.kForwardSoftLimit)  setpoint = ElevatorConstants.kForwardSoftLimit;
      }

    @Override
    public void execute() {
//       pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
        pidController.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
    }

    @Override
    public void end(boolean interrupted) {
 // TA 2/23/23 I think we need to comment this line out so elevator wont just "fall down"
        //       elevatorSubsystem.setMotor(0);
//        System.out.println("ElevatorUpSmall ended!");
    }

    @Override
    public boolean isFinished() {
        return (Math.abs((elevatorSubsystem.getEncoderInches() - setpoint)) <ElevatorConstants.kTolerance);
    }
}
