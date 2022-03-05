package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftSubsystem extends SubsystemBase {

    private static final double MAX_POSITION = 0;
    private static final double MIN_POSITION = 0;
    private static final double MAX_VELOCITY = 0;
    private static final double MPS_TO_TPHMS = 0;

    private static final WPI_TalonFX leftMotor = new WPI_TalonFX(0);
    private static final WPI_TalonFX rightMotor = new WPI_TalonFX(1);
    private static final Encoder leftEncoder = new Encoder(4, 5);
    private static final Encoder rightEncoder = new Encoder(6, 7);
    private static final DigitalInput topLimitSwitch = new DigitalInput(8);
    private static final DigitalInput bottomLimitSwitch = new DigitalInput(8);
    
    public LiftSubsystem() {
        
        rightMotor.setInverted();
        rightMotor.follow(leftMotor);
        rightMotor.setSensorPhase(false);
        leftMotor.setSensorPhase(false);

        leftMotor.configRemoteFeedbackSensor(leftEncoder, 0);
        rightMotor.configRemoteFeedbackSensor(rightEncoder, 0);
        leftMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0);
        rightMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0);

        leftEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        rightEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        leftEncoder.configMagnetOffset(0);
        rightEncoder.configMagnetOffset(0);
    
    }
    
    /**
     * Moves lift based on a given position value.
     * @param position Position value to move to
     */
    public void moveByPosition(double position) {
        leftMotor.set(ControlMode.Position, Math.max(Math.min(position, MAX_POSITION), MIN_POSITION));
    }
    
    /**
     * Moves lift based on a given velocity value.
     * @param velocity Velocity value to follow for move.
     */
    public void moveByVelocity(double velocity) {
        leftMotor.set(ControlMode.Velocity, Math.min(velocity, MAX_VELOCITY) * MPS_TO_TPHMS);
    }

    /**
     * Periodically checks whether limit switches are activated.
     */
    @Override
    public void periodic() {
        if (topLimitSwitch.get() || bottomLimitSwitch.get()) {
            leftMotor.set(ControlMode.PercentOutput, 0);
        }
    }

}