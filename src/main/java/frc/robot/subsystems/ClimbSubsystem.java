package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        leftMotor.configFactoryDefault();
        rightMotor.configFactoryDefault();
        leftMotor.configAllSettings(config);
        rightMotor.configAllSettings(config);
        leftMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.setNeutralMode(NeutralMode.Brake);
        leftMotor.setInverted(true);
    }

    public void runClimb(double pct) {
        SmartDashboard.putNumber("climb left", pct);
        SmartDashboard.putNumber("climb right", pct);
        leftMotor.set(ControlMode.PercentOutput, pct);
        rightMotor.set(ControlMode.PercentOutput, pct);
    }

    public void spinLeft(double pct) {
        SmartDashboard.putNumber("climb left", pct);
        leftMotor.set(ControlMode.PercentOutput, pct);
    }
    public void spinRight(double pct) {
        SmartDashboard.putNumber("climb right", pct);
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
