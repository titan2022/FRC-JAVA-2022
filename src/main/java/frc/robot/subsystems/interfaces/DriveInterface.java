package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface DriveInterface extends Subsystem {
    
    void drive(double velocity, double angleDeg);

    int metersToTicks();
}
