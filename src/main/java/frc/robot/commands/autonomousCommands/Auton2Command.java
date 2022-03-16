package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.Unit;
import frc.robot.commands.DriveToCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.intakeCommands.MasterIntakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class Auton2Command extends SequentialCommandGroup {
    
    public Auton2Command(ShooterSubsystem shooter, IntakeSubsystem intake, DriveSubsystem driveBase, LocalizationSubsystem nav) {
        //Needs tuning
        addCommands(
            //Collect team ball and shoots both starter and team ball
            new ParallelCommandGroup(new MasterIntakeCommand(intake), new DriveToCommand(driveBase.getTranslational(), nav, 88.303 * Unit.IN, -121.095 * Unit.IN, 1)),
            new ShooterCommand(shooter, driveBase.getRotational(), intake, nav, 0, 0, 0, 0, 50, 0.1, 0.02),
            new ShooterCommand(shooter, driveBase.getRotational(), intake, nav, 0, 0, 0, 0, 50, 0.1, 0.02),

            //Collects teams ball on the edge of field and shoots it
            new ParallelCommandGroup(new MasterIntakeCommand(intake), new DriveToCommand(driveBase.getTranslational(), nav, 117.725 * Unit.IN, -282.080 * Unit.IN, 1)),
            new ShooterCommand(shooter, driveBase.getRotational(), intake, nav, 0, 0, 0, 0, 50, 0.1, 0.02),

            //Terminal player inputs one ball which is shot
            new MasterIntakeCommand(intake),
            new ShooterCommand(shooter, driveBase.getRotational(), intake, nav, 0, 0, 0, 0, 50, 0.1, 0.02),

            //Collects enemy ball and shoots it away
            new ParallelCommandGroup(new MasterIntakeCommand(intake), new DriveToCommand(driveBase.getTranslational(), nav, 33.767 * Unit.IN, -149.227 * Unit.IN, 1)),
            new ShooterCommand(shooter, driveBase.getRotational(), intake, nav, 0, 0, 0, 0, 50, 0.1, 0.02),

            //Gets to the line and prepares for player control period
            new DriveToCommand(driveBase.getTranslational(), nav, 1, 1, 1)
        );
    }
}
