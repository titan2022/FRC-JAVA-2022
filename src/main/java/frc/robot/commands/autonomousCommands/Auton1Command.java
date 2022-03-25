package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Unit;
import frc.robot.commands.AutonRoute;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Auton1Command extends SequentialCommandGroup {

    public Auton1Command(ShooterSubsystem shooter, IntakeSubsystem intake, DriveSubsystem driveBase,
        LocalizationSubsystem nav) {
        nav.translateTo(new Translation2d(-23.526 * Unit.IN,-90.975 * Unit.IN));
        nav.setOrientation(new Rotation2d(1.54461638801));
        addCommands(
            new AutonRoute(driveBase, intake, shooter, nav, new Translation2d(-23.526 * Unit.IN ,-90.975 -23.526 * Unit.IN), new Translation2d(-81.643 * Unit.IN, 129.396 * Unit.IN), new Translation2d(117.725 * Unit.IN, -282.080 * Unit.IN))
        );
    
    }
}
