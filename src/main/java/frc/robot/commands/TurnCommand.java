package frc.robot.commands;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RotationalDrivebase;

public class TurnCommand extends CommandBase {
    private RotationalDrivebase drive;
    private Rotation2d start, target, last;
    private double velocity, acceleration, step, lastVel;
    private WPI_Pigeon2 imu = new WPI_Pigeon2(40);

    public TurnCommand(RotationalDrivebase drive, Rotation2d offset, double velocity, double acceleration, double step) {
        this.drive = drive;
        target = offset;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.step = step;
    }
    public TurnCommand(RotationalDrivebase drive, Rotation2d offset, double velocity, double acceleration) {
        this(drive, offset, velocity, acceleration, 0.02);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Rotation Task", String.format("Rel %.2f", target.getDegrees()));
        start = Rotation2d.fromDegrees(imu.getAngle());
        target = start.plus(target);
        last = start;
    }

    @Override
    public void execute() {
        last = last.plus(new Rotation2d(lastVel).times(step));
        double delta = target.minus(last).getRadians();
        lastVel = Math.min(velocity, Math.sqrt(2 * acceleration * Math.abs(delta))) * Math.signum(delta);
        drive.setRotation(lastVel);
        SmartDashboard.putNumber("Rotate Target Distance", Math.toDegrees(delta));
        SmartDashboard.putNumber("Rotate Velocity", lastVel);
    }

    @Override
    public void end(boolean interrupted) {
        drive.setRotation(0);
        SmartDashboard.putNumber("Rotate Velocity", 0);
        SmartDashboard.putString("Rotation Task", interrupted ? "Interrupted" : "Finished");
    }

    @Override
    public boolean isFinished() {
        double rem = target.minus(last).getRadians();
        return rem / (2 * lastVel * step) < 1;
    }
}
