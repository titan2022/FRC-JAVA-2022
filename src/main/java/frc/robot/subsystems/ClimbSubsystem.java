package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    private static int LEFT_MOTOR_ID = 9;
    private static int RIGHT_MOTOR_ID = 17;

    private WPI_TalonFX leftMotor = new WPI_TalonFX(LEFT_MOTOR_ID);
    private WPI_TalonFX rightMotor = new WPI_TalonFX(RIGHT_MOTOR_ID);

    public ClimbSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.neutralDeadband = 0;
        config.statorCurrLimit.enable = false;
        leftMotor.configAllSettings(config);
        rightMotor.configAllSettings(config);
        leftMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        rightMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        leftMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        rightMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        leftMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);
        rightMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);
        leftMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.setNeutralMode(NeutralMode.Brake);
        leftMotor.setInverted(true);
    }

    public double getTicks() {
        return (getLeftTicks() + getRightTicks()) / 2;
    }
    public double getLeftTicks() {
        return leftMotor.getSelectedSensorPosition();
    }

    public double getRightTicks() {
        return rightMotor.getSelectedSensorPosition();
    }

    /**
     * Runs climb by ticks/100ms
     * @param velocity Ticks per 100/ms
     */
    public void runClimbVelocity(double velocity) {
        leftMotor.set(ControlMode.Velocity, velocity);
        rightMotor.set(ControlMode.Velocity, velocity);
    }

    public void runClimbPercent(double pct) {
        //SmartDashboard.putNumber("climb left", pct);
        //SmartDashboard.putNumber("climb right", pct);
        leftMotor.set(ControlMode.PercentOutput, pct);
        rightMotor.set(ControlMode.PercentOutput, pct);
    }

    public void spinLeft(double pct) {
        //SmartDashboard.putNumber("climb left", pct);
        leftMotor.set(ControlMode.PercentOutput, pct);
    }
    public void spinRight(double pct) {
        //SmartDashboard.putNumber("climb right", pct);
        rightMotor.set(ControlMode.PercentOutput, pct);
    }

    public void coast() {
        leftMotor.setNeutralMode(NeutralMode.Coast);
        rightMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void brake() {
        leftMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.setNeutralMode(NeutralMode.Brake);
    }
    
}
