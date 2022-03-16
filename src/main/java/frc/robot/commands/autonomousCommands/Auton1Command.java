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

public class Auton1Command extends SequentialCommandGroup {

    public Auton1Command(ShooterSubsystem shooter, IntakeSubsystem intake, DriveSubsystem driveBase,
        LocalizationSubsystem nav) {
        // Needs to be tuned
        addCommands(
            // Collect team ball and shoots starter and team balls into goal
            new ParallelCommandGroup(new MasterIntakeCommand(intake), new DriveToCommand(driveBase.getTranslational(), nav, -81.643 * Unit.IN, 129.396 * Unit.IN, 1)),
            new ShooterCommand(shooter, driveBase.getRotational(), intake, nav, 0, 0, 0, 0, 50, 0.1, 0.02),
            new ShooterCommand(shooter, driveBase.getRotational(), intake, nav, 0, 0, 0, 0, 50, 0.1, 0.02),

            //Gets terminal balls team's balls
            new ParallelCommandGroup(new MasterIntakeCommand(intake), new DriveToCommand(driveBase.getTranslational(), nav, 117.725 * Unit.IN, -282.080 * Unit.IN, 1)),
            new ShooterCommand(shooter, driveBase.getRotational(), intake, nav, 0, 0, 0, 0, 50, 0.1, 0.02),
            new ShooterCommand(shooter, driveBase.getRotational(), intake, nav, 0, 0, 0, 0, 50, 0.1, 0.02)
        );
    }
}
