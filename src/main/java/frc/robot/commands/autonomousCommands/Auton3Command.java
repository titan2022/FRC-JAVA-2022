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
            //Collect team ball and shoots both starter and team ball
            new ParallelCommandGroup(new MasterIntakeCommand(intake), new DriveToCommand(driveBase, nav, 1, 1, 1)),
            new ShooterCommand(shooter, driveBase.getRotational(), intake, nav, 0, 0, 0, 0, 50, 0.1, 0.02),
            new ShooterCommand(shooter, driveBase.getRotational(), intake, nav, 0, 0, 0, 0, 50, 0.1, 0.02),

            //Collects two teams balls on the edges of field
            new ParallelCommandGroup(new MasterIntakeCommand(intake), new DriveToCommand(driveBase, nav, 1, 1, 1)),
            new ParallelCommandGroup(new MasterIntakeCommand(intake), new DriveToCommand(driveBase, nav, 1, 1, 1)),

            //Shoots two team balls
            new ShooterCommand(shooter, driveBase.getRotational(), intake, nav, 0, 0, 0, 0, 50, 0.1, 0.02),
            new ShooterCommand(shooter, driveBase.getRotational(), intake, nav, 0, 0, 0, 0, 50, 0.1, 0.02),

            //While shooting, terminal inputs ball which is shot into goal again
            new MasterIntakeCommand(intake),
            new ShooterCommand(shooter, driveBase.getRotational(), intake, nav, 0, 0, 0, 0, 50, 0.1, 0.02),

            //Gets to the line and prepares for player control period
            new DriveToCommand(driveBase, nav, 1, 1, 1)
        );
    }
}
