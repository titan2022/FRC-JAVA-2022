package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    private static int LEFT_MOTOR_ID = 0;
    private static int RIGHT_MOTOR_ID = 0;

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
        rightMotor.follow(rightMotor);
        rightMotor.setInverted(true);
    }

    public void runClimb(double pct) {
        leftMotor.set(ControlMode.PercentOutput, pct);
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
