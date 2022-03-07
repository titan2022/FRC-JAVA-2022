package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import static frc.robot.Constants.Unit.*;


public class ShooterSubsystem extends SubsystemBase {

    private static final double MAX_VELOCITY = 10000000 * RAD / S;
    private static final double DEFAULT_VELOCITY = 0 * RAD / S;
    private static final double HOOD_OFFSET = 0;
    private static final double HOOD_RATIO = 200;
    private static final double HOOD_MIN_ANGLE = 0 * DEG;
    private static final double HOOD_MAX_ANGLE = 20 * DEG;

    private static final int RIGHT_MOTOR_PORT = 20;
    private static final int LEFT_MOTOR_PORT = 21;
    private static final int HOOD_MOTOR_ID = 19;
    
    //private static final int RIGHT_ENCODER_PORT = 9;
    //private static final int LEFT_ENCODER_PORT = 10;
    
    private static final WPI_TalonFX rightMotor = new WPI_TalonFX(RIGHT_MOTOR_PORT);
    private static final WPI_TalonFX leftMotor = new WPI_TalonFX(LEFT_MOTOR_PORT);
    private static final WPI_TalonFX hoodMotor = new WPI_TalonFX(HOOD_MOTOR_ID);

    //private static final CANCoder rightEncoder = new CANCoder(RIGHT_ENCODER_PORT);    
    //private static final CANCoder leftEncoder = new CANCoder(LEFT_ENCODER_PORT);   

    public ShooterSubsystem(){
        //rightMotor.setInverted();
        rightMotor.follow(leftMotor);
        rightMotor.setSensorPhase(false);
        leftMotor.setSensorPhase(false);
        leftMotor.setInverted(false);
        rightMotor.setInverted(true);

        //rightMotor.configRemoteFeedbackFilter(rightEncoder, 0);
        //leftMotor.configRemoteFeedbackFilter(leftEncoder, 0);
        rightMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
        leftMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
        
        //rightEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        //leftEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        rightMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        leftMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        //rightEncoder.configMagnetOffset(0);
        //leftEncoder.configMagnetOffset(0);

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

    public void shoot() {
        leftMotor.set(ControlMode.Velocity, DEFAULT_VELOCITY * (RAD / S) / (FALCON_TICKS / (100 * MS)));
    }

    /**
     * Rotates falcon for shooter
     * 
     * @param radians = Radians per sec
     */
    public void shootPrecise(double radians) {
        leftMotor.set(ControlMode.Velocity, Math.min(radians * (RAD / S), MAX_VELOCITY) / (FALCON_TICKS / (100 * MS)));
    }

    /**
     * Rotates falcon in terms of fractions of max speed
     * 
     * @param percent = Percentage from -1 to 1
     */
    public void shootPercent(double percent) {
        leftMotor.set(ControlMode.PercentOutput, percent);
    }

    public void setAngle(double radians) {
        if(HOOD_MIN_ANGLE < radians && radians < HOOD_MAX_ANGLE)
            hoodMotor.set(ControlMode.Position, HOOD_RATIO * (radians * RAD) / FALCON_TICKS + HOOD_OFFSET);
    }

    public double getAngle() {
        return (hoodMotor.getSelectedSensorPosition() - HOOD_OFFSET) * FALCON_TICKS;
    }

    public double getVelocity() {
        double rawVel = (leftMotor.getSelectedSensorVelocity() + rightMotor.getSelectedSensorVelocity()) / 2;
        return rawVel * (FALCON_TICKS / (100 * MS));
    }
}
