package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ManualElevatorUpDownCmd extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;
    private final double speed;

    public ManualElevatorUpDownCmd(ElevatorSubsystem elevatorSubsystem, double speed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ElevatorUpDownCmd started!");
    }

    @Override
    public void execute() {
        elevatorSubsystem.setMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setMotor(0);
        System.out.println("ElevatorUpDownCmd ended!");
    }

    @Override
    public boolean isFinished() {
//        return false;
        return (elevatorSubsystem.isAtTop() || elevatorSubsystem.isAtBot());

    }
}
