package frc.robot.commands;

import frc.robot.subsystems.LiftSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.XBOX_CONTROLLER;

public class LiftCommand extends CommandBase {
    
    private static LiftSubsystem subsystem;

    public LiftCommand(LiftSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        subsystem.moveByVelocity(XBOX_CONTROLLER.getRightY());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subsystem.moveByVelocity(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}

