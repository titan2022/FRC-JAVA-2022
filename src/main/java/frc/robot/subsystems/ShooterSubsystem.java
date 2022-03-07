package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import static frc.robot.Constants.Unit.*;


public class ShooterSubsystem extends SubsystemBase {
    /** Raw ticks measurement at 0 degrees */
    private static final double HOOD_OFFSET = 0;
    /** Gear ratio between the hood motor and hood rack */
    private static final double HOOD_RATIO = 200;
    private static final double HOOD_MIN_ANGLE = 0 * DEG;
    private static final double HOOD_MAX_ANGLE = 20 * DEG;
    private static final double FLYWHEEL_RATIO = 1;
    private static final double FLYWHEEL_RADIUS = 2 * IN;

    private static final int RIGHT_MOTOR_PORT = 20;
    private static final int LEFT_MOTOR_PORT = 21;
    private static final int HOOD_MOTOR_ID = 19;
    
    private static final WPI_TalonFX rightMotor = new WPI_TalonFX(RIGHT_MOTOR_PORT);
    private static final WPI_TalonFX leftMotor = new WPI_TalonFX(LEFT_MOTOR_PORT);
    private static final WPI_TalonFX hoodMotor = new WPI_TalonFX(HOOD_MOTOR_ID);

    public ShooterSubsystem(){
        rightMotor.follow(leftMotor);
        rightMotor.setSensorPhase(false);
        leftMotor.setSensorPhase(false);
        leftMotor.setInverted(false);
        rightMotor.setInverted(true);

        rightMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
        leftMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);

        leftMotor.config_kP(0, 100);
        leftMotor.config_kI(0, 0);
        leftMotor.config_kD(0, 0);
        rightMotor.config_kP(0, 100);
        rightMotor.config_kI(0, 0);
        rightMotor.config_kD(0, 0);

        hoodMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        hoodMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        hoodMotor.setInverted(false);
        hoodMotor.config_kP(0, 100);
        hoodMotor.config_kI(0, 0);
        hoodMotor.config_kD(0, 0);
    }

    /**
     * Runs the shooter at a specified velocity.
     * 
     * @param speed  The target velocity of the shooter in meters per second.
     */
    public void shootPrecise(double speed) {
        leftMotor.set(ControlMode.Velocity, FLYWHEEL_RATIO * speed * (M / S) / FLYWHEEL_RADIUS * RAD / (FALCON_TICKS / (100 * MS)));
    }

    /**
     * Runs the shooter at by prcent output velocity.
     * 
     * @param percent  The target velocity of the shooter, as a proportion of the
     *  maximum output, [-1,1].
     */
    public void shootPercent(double percent) {
        leftMotor.set(ControlMode.PercentOutput, percent);
    }

    /**
     * Sets the angle of the hood.
     * 
     * This method does nothing if the specified angle is out of bounds.
     * 
     * @param radians  The angle of inclination of the hood in radians. This is
     *  the complement of the angle of inclination of the initial velocity of a
     *  projectile launched from the shooter.
     */
    public void setAngle(double radians) {
        if(HOOD_MIN_ANGLE < radians && radians < HOOD_MAX_ANGLE)
            hoodMotor.set(ControlMode.Position, HOOD_RATIO * (radians * RAD) / FALCON_TICKS + HOOD_OFFSET);
    }

    /**
     * Returns the current angle of the hood.
     * 
     * @return  The current measure angle of inclination of the hood in radians.
     *  This is the complement of the angle of inclination of the initial
     *  velocity of a projectile launched from the shooter.
     */
    public double getAngle() {
        return (hoodMotor.getSelectedSensorPosition() - HOOD_OFFSET) * FALCON_TICKS;
    }

    /**
     * Returns the current velocity of the shooter.
     * 
     * @return  The current measured velocity of the shooter in meters per second.
     */
    public double getVelocity() {
        double rawVel = (leftMotor.getSelectedSensorVelocity() + rightMotor.getSelectedSensorVelocity()) / 2;
        return rawVel * (FALCON_TICKS / (100 * MS)) / RAD * FLYWHEEL_RADIUS;
    }
}
