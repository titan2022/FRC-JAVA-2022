package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.Unit;
import frc.robot.commands.DriveToCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.intakeCommands.MasterIntakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class Auton3Command extends SequentialCommandGroup {
    
    public Auton3Command(ShooterSubsystem shooter, IntakeSubsystem intake, DriveSubsystem driveBase, LocalizationSubsystem nav) {
        nav.translateTo(new Translation2d(80.505, -23.997));
        //Needs tuning
        addCommands(
            //Collect team ball and shoots both starter and team ball
            new ParallelCommandGroup(new MasterIntakeCommand(intake), new DriveToCommand(driveBase.getTranslational(), nav, 150.790 * Unit.IN, -25.910 * Unit.IN, 1)),
            new ShooterCommand(shooter, driveBase.getRotational(), intake, nav, -2, 2, 0, 0, 1, 0.1, 0.02),
            new ShooterCommand(shooter, driveBase.getRotational(), intake, nav, -2, 2, 0, 0, 1, 0.1, 0.02),

            //Collects two teams balls on the edges of field
            new ParallelCommandGroup(new MasterIntakeCommand(intake), new DriveToCommand(driveBase.getTranslational(), nav, 88.303 * Unit.IN, -121.095 * Unit.IN, 1)),
            new ParallelCommandGroup(new MasterIntakeCommand(intake), new DriveToCommand(driveBase.getTranslational(), nav, 117.725 * Unit.IN, -282.080 * Unit.IN, 1)),

            //Shoots two team balls
            new ShooterCommand(shooter, driveBase.getRotational(), intake, nav, -2, 2, 0, 0, 1, 0.1, 0.02),
            new ShooterCommand(shooter, driveBase.getRotational(), intake, nav, -2, 2, 0, 0, 1, 0.1, 0.02),

            //While shooting, terminal inputs ball which is shot into goal again
            new MasterIntakeCommand(intake),
            new ShooterCommand(shooter, driveBase.getRotational(), intake, nav, -2, 2, 0, 0, 1, 0.1, 0.02)
        );
    }
}
