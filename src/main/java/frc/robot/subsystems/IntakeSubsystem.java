package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {

    private static final int MOTOR_PORT = 0;

    private static final WPI_TalonFX motor = new WPI_TalonFX(MOTOR_PORT);
    private static final SupplyCurrentLimitConfiguration MAX_AMPS = new SupplyCurrentLimitConfiguration(true, 10, 0, 0);
    private static final StatorCurrentLimitConfiguration MAX_AMPS_OUT = new StatorCurrentLimitConfiguration(true, 10, 0, 0);

    public IntakeSubsystem() {
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        motor.setInverted(false);
        motor.configSupplyCurrentLimit(MAX_AMPS);
        motor.configStatorCurrentLimit(MAX_AMPS_OUT);
        motor.configNeutralDeadband(0.01);
        motor.selectProfileSlot(0, 0);
    }

    public static IntakeSubsystem test() {
        return new IntakeSubsystem();
    }

    /** Run the intake motor(s) at 1000 rpm
     * @param direction 1 = inward, -1 = outward, 0 = stop
     * @see https://motors.vex.com/vexpro-motors/falcon?q=&locale.name=English
     */
    public void spin(int direction) {
        motor.set(ControlMode.Velocity, (1000 * COUNTS_PER_REVOLUTION * 0.1) / 100);
    }
}
