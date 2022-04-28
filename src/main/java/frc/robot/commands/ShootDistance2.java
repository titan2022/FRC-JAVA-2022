package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.Unit.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootDistance2 extends CommandBase {
    private ShooterSubsystem shooter;
    private final double targetVel, targetAngle;
    private int running = 0;

    public ShootDistance2(ShooterSubsystem shooter, double distance) {
        this.shooter = shooter;
        if(distance < 100 * IN){
            targetVel = 7000;
            targetAngle = (0.1705 * (distance / IN) + 11.0776) * DEG;
        }
        else if(distance < 200 * IN){
            targetVel = 61.35 * (distance / IN - 176) + 12000;
            targetAngle = (0.127166 * (distance / IN - 176) + 43.812) * DEG; 
        }
        else{
            targetVel = 32.25 * (distance / IN - 208);
            targetAngle = 43.77 * DEG;
        }
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setAngle(targetAngle);
        shooter.runTicks(targetVel);
        running = 0;
    }

    @Override
    public void execute() {
        if(Math.abs(shooter.getAngle() - targetAngle) < 1.5 * DEG
            && Math.abs(shooter.getRawVelocity() - targetVel) < 50){
            shooter.runQueue(1.0);
            running++;
        }
        else
            shooter.runQueue(0.0);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.coast();
    }

    @Override
    public boolean isFinished() {
        return running > 75;
    }
}
