package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SequencerSubsystem extends SubsystemBase {
    private static final int LEFT_MOTOR_PORT = 0;
    private static final int RIGHT_MOTOR_PORT = 0;
    private static final int MAIN_SENSOR_PORT = 0;

    private static final WPI_TalonFX leftMotor = new WPI_TalonFX(LEFT_MOTOR_PORT);
    private static final WPI_TalonFX rightMotor = new WPI_TalonFX(RIGHT_MOTOR_PORT);
    private static final DigitalInput sequencerBeamSensor = new DigitalInput(MAIN_SENSOR_PORT);
    private static final SupplyCurrentLimitConfiguration MAX_AMPS = new SupplyCurrentLimitConfiguration(true, 10, 0, 0);
    private static final StatorCurrentLimitConfiguration MAX_AMPS_OUT = new StatorCurrentLimitConfiguration(true, 10, 0, 0);

    public SequencerSubsystem(){
        leftMotor.setNeutralMode(NeutralMode.Brake);
        leftMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        leftMotor.setInverted(false);
        leftMotor.configSupplyCurrentLimit(MAX_AMPS);
        leftMotor.configStatorCurrentLimit(MAX_AMPS_OUT);
        leftMotor.configNeutralDeadband(0.01);
        leftMotor.selectProfileSlot(0, 0);

        rightMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        rightMotor.setInverted(true);
        rightMotor.configSupplyCurrentLimit(MAX_AMPS);
        rightMotor.configStatorCurrentLimit(MAX_AMPS_OUT);
        rightMotor.configNeutralDeadband(0.01);
        rightMotor.selectProfileSlot(0, 0);
    }

    public boolean hasBall(){
        return sequencerBeamSensor.get();
    }

    public void spinIntake(int direction) {
        // (rev/min) * (tick/1rev) * (60 sec/min) * (1000 ms/sec) * 100 ms
        intakeMotor.set(ControlMode.Velocity, 1000 * COUNTS_PER_REVOLUTION * 6 * direction);
    }
}
