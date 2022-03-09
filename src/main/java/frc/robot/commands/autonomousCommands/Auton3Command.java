package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.DriveToCommand;
import frc.robot.commands.SpinIntake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class Auton3Command extends ParallelCommandGroup{
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final DriveSubsystem driveBase;
    private final LocalizationSubsystem nav;
    
    public Auton3Command(ShooterSubsystem shooter, IntakeSubsystem intake, DriveSubsystem driveBase, LocalizationSubsystem nav) {
        this.shooter = shooter;
        this.intake = intake;
        this.driveBase = driveBase;
        this.nav = nav;

        addCommands(
            new SpinIntake(intake, 10),
            new DriveToCommand(driveBase, nav, 1, 1, 1)
        );


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
