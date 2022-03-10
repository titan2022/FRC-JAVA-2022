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
}
