package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftSubsystem extends SubsystemBase {

    private static double MAX_POSITION = 0;
    private static double MIN_POSITION = 0;
    private static double MAX_VELOCITY = 0;

    private final WPI_TalonFX leftMotor;
    private final WPI_TalonFX rightMotor;
    private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    private double position;
    
    public LiftSubsystem() {
        leftMotor = new WPI_TalonFX(0);
        rightMotor = new WPI_TalonFX(1);
        rightMotor.set();
        leftEncoder = new Encoder(4, 5);
        rightEncoder = new Encoder(6, 7);
        position = 0;
    }

    public void move(double position, double velocity) {
        this.position = Math.max(Math.min(position, MAX_POSITION), MIN_POSITION);
        this.velocity = Math.min(velocity, MAX_VELOCITY);
        rightMotor // counterclockwise
    }

    public double getPosition() {
        return position;
    }

}