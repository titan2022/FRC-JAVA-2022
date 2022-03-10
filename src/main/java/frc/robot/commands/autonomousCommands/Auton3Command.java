package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.DriveToCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.intakeCommands.MasterIntakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class Auton3Command extends SequentialCommandGroup {
    
    public Auton3Command(ShooterSubsystem shooter, IntakeSubsystem intake, DriveSubsystem driveBase, LocalizationSubsystem nav) {
        //Needs tuning
        addCommands(
            //Collect our ball and shoots
            new ParallelCommandGroup(new MasterIntakeCommand(intake), new DriveToCommand(driveBase, nav, 1, 1, 1)),
            new ShooterCommand(shooter, driveBase.getRotational(), intake, nav, 0, 0, 0, 0, 50, 0.1, 0.02),
            new ShooterCommand(shooter, driveBase.getRotational(), intake, nav, 0, 0, 0, 0, 50, 0.1, 0.02),

            //Collects two balls on the edges of field and shoot
            new ParallelCommandGroup(new MasterIntakeCommand(intake), new DriveToCommand(driveBase, nav, 1, 1, 1)),
            new ParallelCommandGroup(new MasterIntakeCommand(intake), new DriveToCommand(driveBase, nav, 1, 1, 1)),
            new ShooterCommand(shooter, driveBase.getRotational(), intake, nav, 0, 0, 0, 0, 50, 0.1, 0.02),
            new ShooterCommand(shooter, driveBase.getRotational(), intake, nav, 0, 0, 0, 0, 50, 0.1, 0.02)
        );
    }
}
