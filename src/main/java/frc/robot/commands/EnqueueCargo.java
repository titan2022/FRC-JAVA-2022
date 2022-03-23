package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class EnqueueCargo extends CommandBase {
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;

    public EnqueueCargo(IntakeSubsystem intake, ShooterSubsystem shooter) {
        this.intake = intake;
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        intake.spinHopper(1.0);
        shooter.runQueue(0.5);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return shooter.hasQueue();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.runQueue(0.0);
        intake.spinHopper(0.0);
    }
}
