package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;


public class ShooterSubsystem extends SubsystemBase {

    private static final double MAX_VELOCITY = 10000000;
    private static final double DEFAULT_VELOCITY = 0;
    private static final int TICKS_PER_REVOLUTION = 2048;

    private static final int RIGHT_MOTOR_PORT = 0;
    private static final int LEFT_MOTOR_PORT = 1;
    
    private static final int RIGHT_ENCODER_PORT = 9;
    private static final int LEFT_ENCODER_PORT = 10;
    
    private static final WPI_TalonFX rightMotor = new WPI_TalonFX(RIGHT_MOTOR_PORT);
    private static final WPI_TalonFX leftMotor = new WPI_TalonFX(LEFT_MOTOR_PORT);

    private static final CANCoder rightEncoder = new CANCoder(RIGHT_ENCODER_PORT);    
    private static final CANCoder leftEncoder = new CANCoder(LEFT_ENCODER_PORT);   

    public ShooterSubsystem(){
        //rightMotor.setInverted();
        rightMotor.follow(leftMotor);
        rightMotor.setSensorPhase(false);
        leftMotor.setSensorPhase(false);
        leftMotor.setInverted(false);
        rightMotor.setInverted(true);

        rightMotor.configRemoteFeedbackFilter(rightEncoder, 0);
        leftMotor.configRemoteFeedbackFilter(leftEncoder, 0);
        rightMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0,0,0);
        leftMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0,0,0);
        
        rightEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        leftEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        rightEncoder.configMagnetOffset(0);
        leftEncoder.configMagnetOffset(0);

        leftMotor.config_kP(0, 100);
        leftMotor.config_kI(0, 0);
        leftMotor.config_kD(0, 0);
        rightMotor.config_kP(0, 100);
        rightMotor.config_kI(0, 0);
        rightMotor.config_kD(0, 0);

    }

    public void shoot() {
        leftMotor.set(ControlMode.Velocity, DEFAULT_VELOCITY * TICKS_PER_REVOLUTION);
    }

    /**
     * Rotates falcon for shooter
     * 
     * @param radians = Radians per sec
     */
    public void shootPrecise(double radians) {
        leftMotor.set(ControlMode.Velocity, Math.min((radians / (20 * Math.PI)), MAX_VELOCITY) * TICKS_PER_REVOLUTION);
    }

    /**
     * Rotates falcon in terms of fractions of max speed
     * 
     * @param percent = Percentage from -1 to 1
     */
    public void shootPercent(double percent) {
        leftMotor.set(ControlMode.PercentOutput, percent);
    }

}
