package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Xinmotek;
import frc.robot.subsystems.ShooterSubsystem;

public class ManualShooterCommand extends CommandBase {
    private ShooterSubsystem shooter;
    private Xinmotek joy;
    private double lastVel = 0;
    private double step;

    public ManualShooterCommand(ShooterSubsystem shooter, Xinmotek joy, double step) {
        addRequirements(shooter);
        this.shooter = shooter;
        this.joy = joy;
        this.step = step;
    }
    public ManualShooterCommand(ShooterSubsystem shooter, Xinmotek joy) {
        this(shooter, joy, 0.2);
    }

    @Override
    public void initialize() {
        lastVel = shooter.getVelocity();
    }

    @Override
    public void execute() {
        lastVel += joy.getRightY() * step;
        if(lastVel < 0) lastVel = 0;
        shooter.run(lastVel);
        shooter.sendDebug();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.runPercent(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
