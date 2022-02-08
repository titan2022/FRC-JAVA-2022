package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {

    private static final int INTAKE_MOTOR_PORT = 0;
    private static final int HOPPER_MOTOR_PORT = 0;
    private static final int INTAKE_SENSOR_PORT = 0;
    private static final int HOPPER_SENSOR_PORT = 0;

    private static final WPI_TalonFX intakeMotor = new WPI_TalonFX(INTAKE_MOTOR_PORT);
    private static final WPI_TalonFX hopperMotor = new WPI_TalonFX(HOPPER_MOTOR_PORT);
    private static final DigitalInput intakeBeamSensor = new DigitalInput(INTAKE_SENSOR_PORT);
    private static final DigitalInput hopperBeamSensor = new DigitalInput(HOPPER_SENSOR_PORT);
    private static final Solenoid claw = new Solenoid(1);
    private static final SupplyCurrentLimitConfiguration MAX_AMPS = new SupplyCurrentLimitConfiguration(true, 10, 0, 0);
    private static final StatorCurrentLimitConfiguration MAX_AMPS_OUT = new StatorCurrentLimitConfiguration(true, 10, 0, 0);

    public IntakeSubsystem() {
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        intakeMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        intakeMotor.setInverted(false);
        intakeMotor.configSupplyCurrentLimit(MAX_AMPS);
        intakeMotor.configStatorCurrentLimit(MAX_AMPS_OUT);
        intakeMotor.configNeutralDeadband(0.01);
        intakeMotor.selectProfileSlot(0, 0);

        hopperMotor.setNeutralMode(NeutralMode.Brake);
        hopperMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        hopperMotor.setInverted(false);
        hopperMotor.configSupplyCurrentLimit(MAX_AMPS);
        hopperMotor.configStatorCurrentLimit(MAX_AMPS_OUT);
        hopperMotor.configNeutralDeadband(0.01);
        hopperMotor.selectProfileSlot(0, 0);
    }

    public boolean intakeBall() {
        return intakeBeamSensor.get();
    }

    public boolean hopperBall(){
        return hopperBeamSensor.get();
    }

    /** Run the intake motor(s) at 1000 rpm
     * @param direction 1 = inward, -1 = outward, 0 = stop
     * @see https://motors.vex.com/vexpro-motors/falcon?q=&locale.name=English
     */
    public void spinIntake(int direction) {
        // (rev/min) * (tick/1rev) * (60 sec/min) * (1000 ms/sec) * 100 ms
        intakeMotor.set(ControlMode.Velocity, 1000 * COUNTS_PER_REVOLUTION * 6 * direction);
    }
    public void spinHopper(int direction) {
        hopperMotor.set(ControlMode.Velocity, 1000 * COUNTS_PER_REVOLUTION * 6 * direction);
    }
    public void setClaw(boolean open) {
        claw.set(open);
    }
}
