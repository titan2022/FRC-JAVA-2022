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

import static frc.robot.Constants.Unit.*;

public class AutonRoute extends SequentialCommandGroup {
    public AutonRoute(DriveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter, LocalizationSubsystem nav, Translation2d start, Translation2d... cargo) {
        addCommands(new InstantCommand(() -> nav.translateTo(start)));
        boolean hasCargo = true;
        for(Translation2d pos : cargo){
            addCommands(new DriveToCommand(drive.getTranslational(), nav, pos, 10, 1));
            addCommands(new ParallelCommandGroup(
                new DriveToCommand(drive.getTranslational(), nav, pos, 10, 2.5*IN),
                new IntakeCargo(intake, shooter)
            ));
            if(hasCargo){
                addCommands(new ShooterCommand(shooter, drive.getRotational(), intake, nav, 2, 2, 0, 0, 1, 6*IN, 0.02));
                addCommands(new ShooterCommand(shooter, drive.getRotational(), intake, nav, 2, 2, 0, 0, 1, 6*IN, 0.02));
                hasCargo = false;
            }
            else{
                hasCargo = true;
            }
        }
        if(hasCargo)
            addCommands(new ShooterCommand(shooter, drive.getRotational(), intake, nav, 2, 2, 0, 0, 1, 6*IN, 0.02));
    }
}
