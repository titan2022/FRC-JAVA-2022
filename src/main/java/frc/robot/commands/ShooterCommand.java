package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.RotationalDrivebase;
import frc.robot.subsystems.ShooterSubsystem;
import static frc.robot.Constants.Unit.*;

public class ShooterCommand extends CommandBase {
    private ShooterSubsystem shooter;
    private RotationalDrivebase base;
    private LocalizationSubsystem nav;
    private double m;
    private double kP, kI, kD, kF;
    private double threshold;
    private double step;
    private double sum = 0.0;
    private double prev = 0.0;
    private static final double TARGET_HEIGHT = 8 * FT + 8 * IN;
    private static final double g = 9.8 * (M / S);
    private int state = 0;

    private class Trajectory {
        public final double vel;
        public final Rotation2d theta, phi;

        Trajectory(double vel, Rotation2d theta, Rotation2d phi){
            this.vel = vel;
            this.theta = theta;
            this.phi = phi;
        }
        Trajectory(double r, double h, Translation2d drift) {
            double vx = Math.sqrt(g*r*r / (h - m*r) / 2) - drift.getY();
            double vy = g*r / vx + m*vx;
            double v = Math.hypot(vx, vy);
            theta = new Rotation2d(vx, vy);
            phi = new Rotation2d(v, -drift.getX());
            vel = Math.hypot(v, drift.getX());
        }
        Trajectory(double r, double h, Translation2d drift, Rotation2d theta) {
            this.theta = theta;
            double tmp = 2 * theta.getCos() * (h*theta.getCos() - r*theta.getSin());
            double s = drift.getY() * theta.getSin();
            double v = (-r*Math.sqrt(S*S - g*tmp) + r*s - 2*drift.getY()*h*theta.getCos()) / tmp;
            vel = Math.hypot(v, drift.getX());
            phi = new Rotation2d(v, -drift.getX());
        }

        double getError(double r, double h, Translation2d drift) {
            double vy = vel * theta.getSin();
            double t = (Math.sqrt(vy*vy - 2*g*h) - vy) / g;
            Translation2d offset = new Translation2d(vel * theta.getCos(), phi).plus(drift).times(t);
            return Math.hypot(offset.getX(), offset.getY() - r);
        }
    }

    public ShooterCommand(ShooterSubsystem shooter, RotationalDrivebase base, LocalizationSubsystem nav, double targetSlope, double P, double I, double D, double F, double threshold, double step) {
        this.shooter = shooter;
        this.base = base;
        this.nav = nav;
        m = targetSlope;
        kP = P;
        kI = I;
        kD = D;
        kF = F;
        this.threshold = threshold;
        this.step = step;
        addRequirements(shooter, base);
    }

    @Override
    public void initialize() {
        state = 0;
        sum = 0;
        prev = new Rotation2d(Math.PI).minus(nav.getOrientation()).plus(nav.getTheta()).getRadians();
    }

    private double updateRotationPID(double r, Rotation2d theta, Translation2d vel, double deltaPhi) {
        Translation2d predPos = new Translation2d(r, theta).plus(vel.times(step));
        Rotation2d nextTheta = new Rotation2d(predPos.getX(), predPos.getY());
        double omega = kP*deltaPhi + kI*sum + kD*(deltaPhi-prev) + kF*(theta.minus(nextTheta).getRadians());
        sum += deltaPhi;
        prev = deltaPhi;
        return omega;
    }

    @Override
    public void execute() {
        Translation2d fieldVel = nav.getVelocity();
        Rotation2d deltaPhi = nav.getDeltaPhi();
        Translation2d drift = fieldVel.rotateBy(deltaPhi);
        double h = TARGET_HEIGHT - shooter.getHeight();
        double r = nav.getDistance();
        Rotation2d theta = new Rotation2d(Math.PI - shooter.getAngle());
        Trajectory current = new Trajectory(shooter.getVelocity(), theta, deltaPhi);
        if(shooter.queueEnabled)
            shooter.runQueue(current.getError(r, h, drift) < threshold ? 1.0 : 0.0);
        Trajectory target = shooter.hoodEnabled ? new Trajectory(r, h, drift) : new Trajectory(r, h, drift, theta);
        double hoodAngle = Math.PI - target.theta.getRadians();
        if(shooter.hoodEnabled && (hoodAngle > shooter.getMaxAngle() || hoodAngle < shooter.getMinAngle()))
            target = new Trajectory(r, h, drift, theta);
        shooter.run(shooter.getQueueColor() == shooter.robotColor ? target.vel : 1.5);
        if(shooter.hoodEnabled)
            shooter.setAngle(Math.PI - target.theta.getRadians());
        base.setRotation(updateRotationPID(r, nav.getTheta(), fieldVel, deltaPhi.getRadians()));
        if(shooter.hasCargo())
            state = 1;
        else if(state == 1)
            state = 2;
        SmartDashboard.putNumber("Tgt Angle", hoodAngle / DEG);
        SmartDashboard.putNumber("Tgt Velocity", target.vel);
        SmartDashboard.putNumber("Tgt Phi", target.phi.getDegrees());
        shooter.sendDebug();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.coast();
    }

    @Override
    public boolean isFinished() {
        return state > 1;
    }
}
