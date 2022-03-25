package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intakeCommands.IntakeCargo;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TranslationalDrivebase;

import static frc.robot.Constants.Unit.*;

public class SimpleAuton extends SequentialCommandGroup {
    public SimpleAuton(DriveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter, LocalizationSubsystem nav) {
        addCommands(
            new InstantCommand(() -> {nav.resetHeading(); nav.translateTo(new Translation2d(0, 0));}),
            new ParallelCommandGroup(
                new DriveToCommand(drive.getTranslational(), nav, new Translation2d(0, 50*IN), 5, 5*IN, 2),
                new IntakeCargo(intake, shooter)
            ),
            new DriveToCommand(drive.getTranslational(), nav, new Translation2d(0, -85*IN), 1.5, 5*IN, 1).withTimeout(5),
            new InstantCommand(() -> {nav.resetHeading(); nav.translateTo(new Translation2d(0, 2*FT + 7.5*IN));}),
            new ShooterCommand(shooter, drive.getRotational(), nav, 1, 2, 0, 0, 1, 0.1, 0.02),
            new ShooterCommand(shooter, drive.getRotational(), nav, 1, 2, 0, 0, 1, 0.1, 0.02),
            new DriveToCommand(drive.getTranslational(), nav, new Translation2d(0, 120*IN), 5, 2*IN, 2)
        );
    }
}
