package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {

    private static final int INTAKE_MOTOR_PORT = 12;
    private static final int HOPPER_MOTOR_PORT = 13;
    private static final int HOPPER_MOTOR_PORT2 = 14;
    //private static final int HOPPER_MOTOR_PORT3 = 2;
    private static final int INTAKE_SENSOR_PORT = 0;
    private static final int HOPPER_SENSOR_PORT = 1;

    private static final WPI_TalonFX intakeMotor = new WPI_TalonFX(INTAKE_MOTOR_PORT);
    private static final WPI_TalonFX hopperMotor = new WPI_TalonFX(HOPPER_MOTOR_PORT);
    private static final WPI_TalonFX hopperMotor2 = new WPI_TalonFX(HOPPER_MOTOR_PORT2);
    //private static final WPI_TalonFX hopperMotor3 = new WPI_TalonFX(HOPPER_MOTOR_PORT3);
    private static final DigitalInput intakeBeamSensor = new DigitalInput(INTAKE_SENSOR_PORT);
    private static final DigitalInput hopperBeamSensor = new DigitalInput(HOPPER_SENSOR_PORT);
    //private static final Solenoid claw = new Solenoid(1);
    private static final SupplyCurrentLimitConfiguration MAX_AMPS = new SupplyCurrentLimitConfiguration(true, 10, 0, 0);
    private static final StatorCurrentLimitConfiguration MAX_AMPS_OUT = new StatorCurrentLimitConfiguration(true, 10, 0, 0);

    public IntakeSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.supplyCurrLimit = MAX_AMPS;
        config.statorCurrLimit = MAX_AMPS_OUT;
        config.neutralDeadband = 0.1;
        config.slot0.kP = 3;
        config.slot0.kI = 0;
        config.slot0.kD = 0;
        config.slot0.kF = 0;

        intakeMotor.configAllSettings(config);
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        intakeMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        intakeMotor.setInverted(true);
        intakeMotor.selectProfileSlot(0, 0);

        hopperMotor.configAllSettings(config);
        hopperMotor.setNeutralMode(NeutralMode.Brake);
        hopperMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        hopperMotor.setInverted(true);
        hopperMotor.selectProfileSlot(0, 0);

        hopperMotor2.configAllSettings(config);
        hopperMotor2.setNeutralMode(NeutralMode.Brake);
        hopperMotor2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        hopperMotor2.setInverted(true);
        hopperMotor2.selectProfileSlot(0, 0);
        hopperMotor2.follow(hopperMotor);

        /*hopperMotor3.configAllSettings(config);
        hopperMotor3.setNeutralMode(NeutralMode.Brake);
        hopperMotor3.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        hopperMotor3.setInverted(true);
        hopperMotor3.selectProfileSlot(0, 0);*/
    }

    public boolean intakeBall() {
        return intakeBeamSensor.get();
    }

    public boolean hopperBall(){
        return hopperBeamSensor.get();
    }

    /** Run the intake motor(s) at 1000 rpm
     * @param intakeSpeed = Radians per sec
     * @see https://motors.vex.com/vexpro-motors/falcon?q=&locale.name=English
     */
    public void spinIntake(double intakeSpeed) {
        
        intakeMotor.set(ControlMode.Velocity, (intakeSpeed / 20 * Math.PI) * COUNTS_PER_REVOLUTION);
    }

    /** Run the intake motor(s) at 1000 rpm
     * @param intakeSpeed = Radians per sec
     * @see https://motors.vex.com/vexpro-motors/falcon?q=&locale.name=English
     */
    public void spinHopper(double hopperSpeed) {
        hopperMotor.set(ControlMode.Velocity, (hopperSpeed / 20 * Math.PI) * COUNTS_PER_REVOLUTION);
    }
    
    public void setClaw(boolean open) {
        //claw.set(open);
    }
}
