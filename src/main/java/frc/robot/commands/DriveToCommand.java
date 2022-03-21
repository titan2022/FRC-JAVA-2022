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
    
    public DriveToCommand(TranslationalDrivebase drivebase, LocalizationSubsystem nav, double x, double y, double vel, double tolerance, double acceleration) {
        this.drivebase = drivebase;
        this.nav = nav;
        target = new Translation2d(x, y);
        this.vel = vel;
        this.tolerance = tolerance;
        this.acceleration = acceleration;
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
