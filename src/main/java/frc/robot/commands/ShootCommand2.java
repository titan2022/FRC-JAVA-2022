package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RotationalDrivebase;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.Unit.*;

public class ShootCommand2 extends CommandBase {
    private final ShooterSubsystem shooter;
    private final RotationalDrivebase drive;
    private static final NetworkTableEntry tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
    private static final NetworkTableEntry ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty");
    private static final NetworkTableEntry tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv");
    private static final double CAMERA_HEIGHT = 26.5 * IN;
    private static final double TARGET_HEIGHT = 8 * FT + 8 * IN;
    private static final double CAMERA_ANGLE = 32 * DEG;
    private static final double CAMERA_OFFSET = 14 * IN;
    private int running = 0;
    private boolean queueRunning = false;

    public ShootCommand2(ShooterSubsystem shooter, RotationalDrivebase drive) {
        this.shooter = shooter;
        this.drive = drive;
        addRequirements(shooter, drive);
    }

    @Override
    public void initialize() {
        running = 0;
        queueRunning = false;
    }

    @Override
    public void execute() {
        double v = tv.getDouble(0), x = tx.getDouble(180) * DEG, y = ty.getDouble(0) * DEG + CAMERA_ANGLE;
        if(v == 0){
            drive.setRotation(Math.PI / 4);
            shooter.runQueue(0);
        }
        else if(Math.abs(x) > 5){
            drive.setRotation(x);
            shooter.runQueue(0);
        }
        else{
            drive.setRotation(x);
            double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(y / RAD) - CAMERA_OFFSET;
            double targetVel, targetAngle;
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
            shooter.runTicks(targetVel);
            shooter.setAngle(targetAngle);
            SmartDashboard.putNumber("shoot dist (ft)", distance / FT);
            double vel = shooter.getRawVelocity();
            double angle = shooter.getAngle();
            SmartDashboard.putNumber("vel err", targetVel - vel);
            SmartDashboard.putNumber("angle err", (targetAngle - angle) / DEG);
            if(Math.abs(vel - targetVel) < 50 && Math.abs(angle - targetAngle) < 1 * DEG){
                if(!queueRunning) shooter.runQueue(1.0);
                queueRunning = true;
                running++;
            }
            else if(queueRunning){
                shooter.runQueue(0.0);
                queueRunning = false;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.coast();
        shooter.runQueue(0.0);
    }

    @Override
    public boolean isFinished() {
        return running > 75;
    }
}
