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


public class Auton3Command extends SequentialCommandGroup {
    
    public Auton3Command(ShooterSubsystem shooter, IntakeSubsystem intake, DriveSubsystem driveBase, LocalizationSubsystem nav) {
        nav.translateTo(new Translation2d(80.505 * Unit.IN, -23.997 * Unit.IN));
        nav.setOrientation(new Rotation2d(-0.02617993877));
        //Needs tuning
        addCommands(
            new AutonRoute(driveBase, intake, shooter, nav, new Translation2d(80.505, -23.997), new Translation2d(150.790 * Unit.IN, -25.910 * Unit.IN), new Translation2d(88.303 * Unit.IN, -121.095 * Unit.IN), new Translation2d(117.725 * Unit.IN, -282.080 * Unit.IN))
        );
    }
}
