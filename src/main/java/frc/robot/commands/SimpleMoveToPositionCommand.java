package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

import static frc.robot.Constants.Unit.*;

public class SimpleMoveToPositionCommand extends CommandBase {

    private SwerveDriveSubsystem drivebase;
    private double dist;
    private double initEncoderCount;
    private double encoderCount;

    public static final double ROBOT_TRACK_WIDTH = 23.5 * IN; // 0.672; // meters (30 in)
    public static final double ROBOT_LENGTH = 26 * IN; // 0.672; // meter
    public static final double WHEEL_RADIUS = 2 * IN; // 0.0508; // meters (2 in)
    public static final double GEAR_RATIO = 6.86;
    public static final double METERS_PER_TICKS = WHEEL_RADIUS * 2 * Math.PI / FALCON_CPR / GEAR_RATIO;

    public SimpleMoveToPositionCommand(SwerveDriveSubsystem drivebase, double dist) {
        this.drivebase = drivebase;
        this.dist = dist;
    }

    @Override
    public void initialize() {
        drivebase.getTranlational().setVelocity(new Translation2d(0, 5));
        encoderCount = drivebase.getEncoderCount(0);
        initEncoderCount = encoderCount;
    }

    @Override
    public void execute() {
        encoderCount = drivebase.getEncoderCount(0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivebase.getTranlational().setVelocity(new Translation2d(0, 0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (encoderCount - initEncoderCount) * METERS_PER_TICKS >= dist;
    }

}
