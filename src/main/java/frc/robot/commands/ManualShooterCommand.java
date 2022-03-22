package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.XBOX_CONTROLLER;

public class ManualShooterCommand extends CommandBase {
    private ShooterSubsystem shooter;

    public ManualShooterCommand(ShooterSubsystem shooter) {
        addRequirements(shooter);
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        shooter.runPercent(0);
    }

    @Override
    public void execute() {
        shooter.runPercent(XBOX_CONTROLLER.getRightTriggerAxis());
        shooter.setAngle(XBOX_CONTROLLER.getLeftTriggerAxis() * Math.PI / 18 + Math.PI / 6);
        SmartDashboard.putNumber("shooter", XBOX_CONTROLLER.getRightTriggerAxis());
        SmartDashboard.putNumber("hood (deg)", (XBOX_CONTROLLER.getLeftTriggerAxis() * Math.PI / 18 + Math.PI / 6) * (180 / Math.PI));
    }

    @Override
    public void end(boolean interrupted) {
        shooter.runPercent(0);
        SmartDashboard.putNumber("shooter", -2);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
