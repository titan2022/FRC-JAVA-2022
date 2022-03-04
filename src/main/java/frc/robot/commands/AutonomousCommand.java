package frc.robot.commands;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OdometrySubsystemWrapper;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonomousCommand extends CommandBase {
    // Subsystems TODO: make a class that includes all these components (hmm RobotContainer?)
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final DriveSubsystem drivebase;
    private final OdometrySubsystemWrapper odemetry;
    
    public AutonomousCommand(ShooterSubsystem shooter, IntakeSubsystem intake, DriveSubsystem drivebase, OdometrySubsystemWrapper odometry) {
        this.shooter = shooter;
        this.intake = intake;
        this.drivebase = drivebase;
        this.odemetry = odometry;
    }

    /**
     * 
     * @param x desired x position (meters)
     * @param y desired y position (meters)
     * @param rot desired absolute position (radians)
     * @param sec amount of seconds for the robot to make the move
     */
    private void driveTo(int x, int y, int rot, int sec) {
        int[] position = odemetry.getPos();
        drivebase.setVelocities(new ChassisSpeeds((x - position[0]) / sec, (y - position[1]) / sec, (rot - position[3]) / sec));
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // TODO: does this have a time limit?
        intake.spinIntake(Math.PI / 4);  // Arbitrary speed
        intake.spinHopper(Math.PI / 4);
        driveTo(x, y, rot, sec);
        // TODO: aim with Limelight then shoot
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
