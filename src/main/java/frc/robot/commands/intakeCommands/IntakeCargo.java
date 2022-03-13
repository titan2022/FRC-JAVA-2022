package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCargo extends CommandBase {
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;

    public IntakeCargo(IntakeSubsystem intake, ShooterSubsystem shooter) {
        this.intake = intake;
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        intake.spinIntake(1.0);
        intake.spinHopper(1.0);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        intake.spinHopper(0.0);
        if(interrupted)
            new StartEndCommand(() -> intake.spinIntake(-1.0), () -> intake.spinIntake(0.0), intake)
                .until(() -> !intake.intakeBall()).schedule();
        else{
            intake.spinIntake(0.0);
            new SequentialCommandGroup(
                new StartEndCommand(() -> intake.spinHopper(1.0), () -> intake.spinHopper(0.0), intake)
                    .until(intake::topHopperBall),
                new WaitUntilCommand(() -> !shooter.hasQueue() && shooter.getCurrentCommand() == null),
                new ScheduleCommand(new StartEndCommand(() -> {
                        intake.spinHopper(1.0);
                        shooter.runQueue(0.5);
                    }, () -> {
                        intake.spinHopper(0.0);
                        shooter.runQueue(0.5);
                    }, intake, shooter
                ).until(shooter::hasQueue).withTimeout(2.0))
            ).schedule();
        }
    }
}
