package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.RotationalDrivebase;
import frc.robot.subsystems.ShooterSubsystem;
import static frc.robot.Constants.Unit.*;

public class ShooterCommand extends CommandBase {
    private ShooterSubsystem shooter;
    private RotationalDrivebase base;
    private IntakeSubsystem intake;
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

    public ShooterCommand(ShooterSubsystem shooter, RotationalDrivebase base, IntakeSubsystem intake, LocalizationSubsystem nav, double targetSlope, double P, double I, double D, double F, double threshold, double step) {
        this.shooter = shooter;
        this.base = base;
        this.intake = intake;
        this.nav = nav;
        m = targetSlope;
        kP = P;
        kI = I;
        kD = D;
        kF = F;
        this.threshold = threshold;
        this.step = step;
        addRequirements(shooter, base, intake);
    }

    @Override
    public void initialize() {
        state = 0;
        sum = 0;
        prev = new Rotation2d(Math.PI).minus(nav.getOrientation()).plus(nav.getTheta()).getRadians();
    }

    private double calcError(Translation2d vel, Rotation2d phi, double h) {
        Translation2d pos = nav.getPosition();
        Rotation2d incl = new Rotation2d(Math.PI - shooter.getAngle());
        double v = shooter.getVelocity();
        double vy = v * incl.getSin();
        double v_ground = v * incl.getCos();
        double t = (Math.sqrt(vy*vy - 2*g*h) - vy) / g;
        double error = new Translation2d(v_ground, phi).plus(vel).times(t).plus(pos).getNorm();
        SmartDashboard.putNumber("Shooter error", error);
        return error;
    }

    private double updateRotationPID(double r, Rotation2d theta, Translation2d vel, Rotation2d phi) {
        double deltaPhi = new Rotation2d(Math.PI).minus(phi).plus(theta).getRadians();
        Translation2d predPos = new Translation2d(r, theta).plus(vel.times(step));
        Rotation2d nextTheta = new Rotation2d(predPos.getX(), predPos.getY());
        double omega = kP*deltaPhi + kI*sum + kD*(deltaPhi-prev) + kF*(theta.minus(nextTheta).getRadians());
        sum += deltaPhi;
        prev = deltaPhi;
        return omega;
    }

    private void setVTheta(double r, double h) {
        double vx = Math.sqrt(g*r*r / (h - m*r) / 2);
        double vy = g*r / vx + m*vx;
        double theta = Math.PI - Math.atan2(vy, vx);
        shooter.setAngle(theta);
        if(theta < shooter.getMinAngle()){
            setV(r, shooter.getMinAngle(), h);
        }
        else if(theta > shooter.getMaxAngle()){
            setV(r, shooter.getMaxAngle(), h);
        }
        else{
            double vel = Math.hypot(vx, vy);
            shooter.run(vel);
            SmartDashboard.putNumber("Tgt Shooter Vel", vel);
            SmartDashboard.putNumber("Tgt Hood Angle", theta);
        }
    }

    private void setV(double r, double theta, double h) {
        double drop = Math.tan(theta) * r - h;
        double vel = r * Math.sqrt(g / drop / 2) / Math.cos(theta);
        shooter.run(vel);
        SmartDashboard.putNumber("Tgt Shooter Vel", vel);
        SmartDashboard.putNumber("Tgt Hood Angle", theta);
    }

    @Override
    public void execute() {
        Translation2d vel = nav.getVelocity();
        Rotation2d phi = nav.getOrientation();
        double h = TARGET_HEIGHT - shooter.getHeight();
        if(calcError(vel, phi, h) < threshold)
            intake.spinHopper(1.0);
        else
            intake.spinIntake(0);
        double r = nav.getDistance();
        if(shooter.hoodEnabled)
            setVTheta(r, h);
        else
            setV(r, Math.PI - shooter.getAngle(), h);
        base.setRotation(updateRotationPID(r, nav.getTheta(), vel, phi));
        if(shooter.hasCargo())
            state = 1;
        else if(state == 1)
            state = 2;
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
