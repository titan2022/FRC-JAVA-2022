package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCargo extends SequentialCommandGroup {
    public IntakeCargo(IntakeSubsystem intake, ShooterSubsystem shooter) {
        addCommands(
            new StartEndCommand(
                () -> {intake.spinIntake(1.0); intake.spinHopper(1.0);},
                () -> {intake.spinIntake(0.0); intake.spinHopper(0.0);},
                intake).until(intake::bottomHopperBall),
            new ScheduleCommand(new EnqueueCargo(intake, shooter))
        );
    }
}
