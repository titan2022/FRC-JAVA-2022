package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends CommandBase {
    private ClimbSubsystem climb;
    private double rotations;
    private double desiredPosition;
    private double seconds;
    private static final double deadBand = 10;

    //The gear ratio is for the lift 190.5:1 and the max RPM of the Falcon 500 is 6380 so the max rpm of the lift per second is 0.54516 rps or 196.2576 degrees per second
    //2048 ticks per rotations

    /**
     * Creates climb command for the autoclimb
     * 
     * @param climb The ClimbSubsystem
     * @param rotations Desired rotations(1 is 360 degrees positive and -1 is 360 negative)
     * @param seconds How much time to perform those rotations
     */
    public ClimbCommand(ClimbSubsystem climb, double rotations, double seconds) {
        this.climb = climb;
        this.rotations = rotations;
        this.seconds = seconds;
        addRequirements(climb);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        desiredPosition = climb.getTicks() + (rotations * 2048 * 190.5);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        climb.runClimbVelocity(rotations * 2048 * 190.5 / (10 * seconds));
        if (isFinished())
            end(false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (climb.getTicks() < desiredPosition + deadBand && climb.getTicks() > desiredPosition - deadBand) 
            return true;
        else
            return false;
    }
}   

