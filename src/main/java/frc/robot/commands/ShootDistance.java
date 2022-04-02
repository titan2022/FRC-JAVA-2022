package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.RotationalDrivebase;
import frc.robot.subsystems.ShooterSubsystem;
import static frc.robot.Constants.Unit.*;

public class ShootDistance extends CommandBase {
    private ShooterSubsystem shooter;
    private double threshold, dist;
    private static final double TARGET_HEIGHT = 8 * FT + 8 * IN;
    private static final double g = 9.8 * (M / S);
    private int state = 0;

    private class Trajectory {
        public final double vel;
        public final Rotation2d theta;
        private final double r = (4.5 + 2) * IN;
        private final double R = 2 * FT;

        Trajectory(double vel, Rotation2d theta){
            this.vel = vel;
            this.theta = theta;
        }
        Trajectory(double d, double h) {
            double den = d*R*(d-R);
            double a = -(d*r + h*R) / den;
            double b = (d*d*r + 2*d*h*R - h*R*R) / den;
            SmartDashboard.putNumber("a", a);
            SmartDashboard.putNumber("b", b);
            double vx = Math.sqrt(-g / (2*a));
            double vy = b*vx;
            theta = new Rotation2d(vx, vy);
            vel = Math.hypot(vx, vy);
        }

        double getError(double r, double h) {
            double vy = vel * theta.getSin();
            double t = (Math.sqrt(vy*vy - 2*g*h) - vy) / g;
            Translation2d offset = new Translation2d(0, vel * theta.getCos()).times(t);
            return Math.hypot(offset.getX(), offset.getY() - r);
        }
    }

    public ShootDistance(ShooterSubsystem shooter, double dist, double threshold) {
        this.shooter = shooter;
        this.dist = dist;
        this.threshold = threshold;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        state = 0;
        SmartDashboard.putBoolean("Shooter running", true);
    }

    private void aim() {
        double h = TARGET_HEIGHT - shooter.getHeight();
        Rotation2d theta = new Rotation2d(Math.PI - shooter.getAngle());
        Trajectory current = new Trajectory(shooter.getVelocity(), theta);
        SmartDashboard.putNumber("Error", current.getError(dist, h));
        shooter.runQueue(current.getError(dist, h) < threshold ? 1.0 : 0.0);
        Trajectory target = new Trajectory(dist, h);
        double hoodAngle = Math.PI - target.theta.getRadians();
        shooter.run(target.vel);
        shooter.setAngle(Math.PI/2 - target.theta.getRadians());
        SmartDashboard.putNumber("Tgt Angle", hoodAngle / DEG);
        SmartDashboard.putNumber("Tgt Velocity", target.vel);
        //shooter.sendDebug();
    }

    @Override
    public void execute() {
        aim();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.coast();
        SmartDashboard.putBoolean("Shooter running", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
