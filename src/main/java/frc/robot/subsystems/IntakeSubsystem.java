package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {

    private static final int INTAKE_MOTOR_PORT = 19;
    private static final int HOPPER_MOTOR_PORT = 21;
    //private static final int INTAKE_SENSOR_PORT = 0;
    //private static final int HOPPER_SENSOR_PORT1 = 1;
    //private static final int HOPPER_SENSOR_PORT2 = 2;

    private static final WPI_TalonFX intakeMotor = new WPI_TalonFX(INTAKE_MOTOR_PORT);
    private static final WPI_TalonFX hopperMotor = new WPI_TalonFX(HOPPER_MOTOR_PORT);
    //private static final DigitalInput intakeBeamSensor = new DigitalInput(INTAKE_SENSOR_PORT);
    //private static final DigitalInput bottomHopperBeamSensor = new DigitalInput(HOPPER_SENSOR_PORT1);
    //private static final DigitalInput topHopperBeamSensor = new DigitalInput(HOPPER_SENSOR_PORT2);
    private static final SupplyCurrentLimitConfiguration MAX_AMPS = new SupplyCurrentLimitConfiguration(true, 10, 0, 0);
    //Pnuematics
    //DoubleSolenoid solenoidLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2); //Forward then Reverse Channel
    //DoubleSolenoid solenoidRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 4);

    public IntakeSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.supplyCurrLimit = MAX_AMPS;
        config.neutralDeadband = 0.1;

        intakeMotor.configAllSettings(config);
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        intakeMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        intakeMotor.setInverted(true);
        intakeMotor.selectProfileSlot(0, 0);

        hopperMotor.configAllSettings(config);
        hopperMotor.setNeutralMode(NeutralMode.Brake);
        hopperMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        hopperMotor.setInverted(false);
        hopperMotor.selectProfileSlot(0, 0);
    }

    public boolean intakeBall() {
        return false;//intakeBeamSensor.get();
    }

    public boolean bottomHopperBall(){
        return false;//bottomHopperBeamSensor.get();
    }
    
    public boolean topHopperBall(){
        return false;//topHopperBeamSensor.get();
    }

    /**
     * Run the intake motor(s)
     * 
     * @param intakeSpeed  Velocity of the intake, in the range [-1,1]
     */
    public void spinIntake(double intakePct) {
        /*if(intakePct == 0)
            retract();
        else
            extend();*/
        intakeMotor.set(ControlMode.PercentOutput, intakePct);
    }

    /**
     * Run the hopper motor(s)
     * 
     * @param hopperSpeed  Velocity of the hopper, in the range [-1,1]
     * @see https://motors.vex.com/vexpro-motors/falcon?q=&locale.name=English
     */
    public void spinHopper(double hopperPct) {
        hopperMotor.set(ControlMode.PercentOutput, hopperPct);
    }

    /** Extends the intake. */
    public void extend() {
        //solenoidLeft.set(Value.kReverse);
        //solenoidRight.set(Value.kReverse);
    }
    /** Retracts the intake. */
    public void retract() {
        //solenoidLeft.set(Value.kForward);
        //solenoidRight.set(Value.kForward);
    }

    public void disable() {
        intakeMotor.setNeutralMode(NeutralMode.Coast);
        hopperMotor.setNeutralMode(NeutralMode.Coast);
    }
    public void enable() {
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        hopperMotor.setNeutralMode(NeutralMode.Brake);
    }
}
