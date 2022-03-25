package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class EnqueueCargo extends SequentialCommandGroup {
    public EnqueueCargo(IntakeSubsystem intake, ShooterSubsystem shooter) {
        addCommands(
            new WaitUntilCommand(() -> !shooter.hasQueue()),
            new StartEndCommand(
                () -> intake.spinHopper(1.0),
                () -> intake.spinHopper(0.0),
                intake
            ).until(shooter::hasQueue)
        );
    }
}
