package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class SimpleRotationCommand extends CommandBase {
    private SwerveDriveSubsystem drive;
    private double radians, targetPlus, targetMinus;
    private static double TICKS_PER_RADIAN = 39182.09;

    public SimpleRotationCommand(SwerveDriveSubsystem drive, double radians) {
        this.drive = drive;
        this.radians = radians;
    }

    @Override
    public void initialize() {
        targetPlus = drive.getEncoderCount(0) + TICKS_PER_RADIAN * radians;
        targetMinus = drive.getEncoderCount(0) - TICKS_PER_RADIAN * radians;
        drive.getRotational().setRotation(Math.PI);
    }

    @Override
    public void end(boolean interrupted) {
        drive.getRotational().setRotation(0);
    }

    @Override
    public boolean isFinished(){
        return targetMinus < drive.getEncoderCount(0) && drive.getEncoderCount(0) < targetPlus;
    }
}
