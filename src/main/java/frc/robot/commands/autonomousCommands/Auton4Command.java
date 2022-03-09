package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class Auton4Command extends SequentialCommandGroup{
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final DriveSubsystem driveBase;

    public Auton4Command(ShooterSubsystem shooter, IntakeSubsystem intake, DriveSubsystem driveBase) {
        this.shooter = shooter;
        this.intake = intake;
        this.driveBase = driveBase;
    }

    @Override
    public void initialize() {

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
        return true;
    }
}
