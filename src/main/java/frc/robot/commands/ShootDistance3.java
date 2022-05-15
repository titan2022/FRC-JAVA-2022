package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RotationalDrivebase;
import frc.robot.subsystems.ShooterSubsystem;
import static frc.robot.Constants.Unit.*;

public class ShootDistance3 extends CommandBase {
    private ShooterSubsystem shooter;
    private RotationalDrivebase drive;
    private double vel = 0;
    private double angle = 0;
    private double setPoint = 0;
    private double distance = 0;
    private NetworkTableEntry tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
    private NetworkTableEntry tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv");

    public ShootDistance3(ShooterSubsystem shooter, RotationalDrivebase drive, double distance) {
        this.shooter = shooter;
        this.drive = drive;
        setPoint = distance;
        addRequirements(shooter, drive);
    }
    public ShootDistance3(ShooterSubsystem shooter, RotationalDrivebase drive) {
        this(shooter, drive, 0);
    }

    @Override
    public void initialize() {
        distance = setPoint == 0 ? SmartDashboard.getNumber("Hub Distance", 10.0) * FT : setPoint;
        if(distance / FT < 11.25){
            vel = 8000;
            angle = 0.0372 * distance / FT - 0.136 + shooter.getMinAngle();
        }
        else{
            angle = 27.5461 * DEG;
            vel = 762.2257 * distance / FT - 546.3353;
        }
        SmartDashboard.putNumber("Hub Distance", distance / FT);
        shooter.setAngle(angle);
        shooter.runTicks(vel);
    }

    @Override
    public void execute() {
        double x = -(tv.getDouble(0) != 0 ? tx.getDouble(0) : 80);
        double angleErr = shooter.getAngle() - angle;
        double velErr = shooter.getRawVelocity() - vel;
        if(Math.tan(Math.toRadians(x)) * distance < 1*FT
           && Math.abs(angleErr) < 1*DEG
           && Math.abs(velErr) < 200){
            drive.setRotation(0);
            shooter.runQueue(1.0);
        }
        else{
            shooter.runQueue(0);
            if(Math.abs(x) > 3)
                drive.setRotation(Math.copySign(Math.PI / 8, x));
            else if(Math.abs(x) > 1.5)
                drive.setRotation(Math.PI / 8 * (x / 3));
            else
                drive.setRotation(0);
        }
        SmartDashboard.putNumber("Hub Angle", -x);
        SmartDashboard.putNumber("Angle Err", angleErr / DEG);
        SmartDashboard.putNumber("Vel Err", velErr);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.coast();
        drive.setRotation(0);
        shooter.runQueue(0);
    }
}
