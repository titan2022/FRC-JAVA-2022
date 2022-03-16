package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.TranslationalDrivebase;

public class DriveToCommand extends WaitCommand {
    // TODO: make a class that includes all these components (hmm RobotContainer?)
    private final LocalizationSubsystem odometry;
    private final TranslationalDrivebase drivebase;
    private final double x, y, sec;
    
    public DriveToCommand(TranslationalDrivebase drivebase, LocalizationSubsystem odometry, double x, double y, double sec) {
        super(sec);
        this.drivebase = drivebase;
        this.odometry = odometry;
        this.x = x;
        this.y = y;
        this.sec = sec;
    }

    /**
     * 
     * @param x desired x position (meters)
     * @param y desired y position (meters)
     * @param rot desired absolute position (radians)
     * @param sec amount of seconds for the robot to make the move
     */
    private void driveTo(double x, double y, double sec) {
        Translation2d position = odometry.getPosition();
        drivebase.setVelocity(new Translation2d((x - position.getX()) / sec, (y - position.getY()) / sec));
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        driveTo(x, y, sec);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (isFinished()) {
            end(false);
        } else {
            driveTo(x, y, sec);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivebase.setVelocity(new Translation2d(0, 0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (odometry.getPosition().getX() == x && odometry.getPosition().getY() == y) {
            return true;
        } else {
            return false;
        }
    }
}
