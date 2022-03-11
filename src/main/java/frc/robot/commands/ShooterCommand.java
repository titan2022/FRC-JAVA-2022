package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
        return new Translation2d(v_ground, phi).plus(vel).times(t).plus(pos).getNorm();
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
        double vx = Math.sqrt(g*r*r / (h - m*r) / 2);
        Translation2d targetVel = new Translation2d(vx, g*r / vx + m*vx);
        shooter.run(targetVel.getNorm());
        shooter.setAngle(Math.PI - Math.atan2(targetVel.getY(), targetVel.getX()));
        base.setRotation(updateRotationPID(r, nav.getTheta(), vel, phi));
        if(shooter.hasCargo())
            state = 1;
        else if(state == 1)
            state = 2;
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
