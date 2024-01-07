package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElbowSubsystem;

public class ManualElbowUpDownCmd extends CommandBase {
    private final ElbowSubsystem elbowSubsystem;
    private final double speed;

    public ManualElbowUpDownCmd(ElbowSubsystem elbowSubsystem, double speed) {
        this.elbowSubsystem = elbowSubsystem;
        this.speed = speed;
        addRequirements(elbowSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ElbowUpDownCmd started!");
    }

    @Override
    public void execute() {
        elbowSubsystem.setMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        elbowSubsystem.setMotor(0);
        System.out.println("ElbowUpDownCmd ended!");
    }

    @Override
    public boolean isFinished() {
 //       return false;
     return (elbowSubsystem.isAtMax() || elbowSubsystem.isAtMin());
    }
}
