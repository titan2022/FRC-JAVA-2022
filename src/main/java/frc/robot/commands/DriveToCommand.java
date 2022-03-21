package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.TranslationalDrivebase;

public class DriveToCommand extends CommandBase {
    // TODO: make a class that includes all these components (hmm RobotContainer?)
    private final LocalizationSubsystem nav;
    private final TranslationalDrivebase drivebase;
    private final double vel, tolerance, acceleration;
    private final Translation2d target;
    
    /**
     * Creates a new DriveToCommand.
     * 
     * @param drivebase  The drivebase to control.
     * @param nav  The localizer to use.
     * @param target  The target position to drive to.
     * @param vel  The target cruise velocity.
     * @param tolerance  The tolerance to allow when determining if the
     *  current position matches the target position.
     * @param acceleration  The deceleration to use while approaching the
     *  target. Note that smooth deceleration cannot be guarunteed within the
     *  tolerance radius of the target.
     */
    public DriveToCommand(TranslationalDrivebase drivebase, LocalizationSubsystem nav, Translation2d target, double vel, double tolerance, double acceleration) {
        this.drivebase = drivebase;
        this.nav = nav;
        this.target = target;
        this.vel = vel;
        this.tolerance = tolerance;
        this.acceleration = acceleration;
    }
    /**
     * Creates a new DriveToCommand.
     * 
     * Acceleration is set such that the robot does not decelerate outside of
     * the tolerance radius around the target position.
     * 
     * @param drivebase  The drivebase to control.
     * @param nav  The localizer to use.
     * @param target  The target position to drive to.
     * @param vel  The target cruise velocity.
     * @param tolerance  The tolerance to allow when determining if the
     *  current position matches the target position.
     */
    public DriveToCommand(TranslationalDrivebase drivebase, LocalizationSubsystem nav, Translation2d target, double vel, double tolerance) {
        this(drivebase, nav, target, vel, tolerance, vel*vel/tolerance/2);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Translation2d offset = target.minus(nav.getPosition());
        double dist = offset.getNorm();
        double v = Math.min(vel, Math.sqrt(2 * acceleration * dist));
        drivebase.setVelocity(offset.times(v / dist));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivebase.setVelocity(new Translation2d(0, 0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return target.minus(nav.getPosition()).getNorm() < tolerance;
    }
}
