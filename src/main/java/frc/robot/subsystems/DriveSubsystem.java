package frc.robot.subsystems;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface DriveSubsystem extends Subsystem {
    public void setVelocities(ChassisSpeeds velocities);
    public ChassisSpeeds getVelocities();
}
