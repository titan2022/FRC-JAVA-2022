package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface DriveSubsystem extends Subsystem {
    public void setVelocities(ChassisSpeeds velocities);
    public void setVelocities(Translation2d velocities);
    public void setRotation(double omega);
    public ChassisSpeeds getVelocities();
}
