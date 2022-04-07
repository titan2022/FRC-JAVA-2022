package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private static final int INTAKE_MOTOR_PORT = 19;
    private static final int HOPPER_MOTOR_PORT = 21;
    private static final int HOPPER_SENSOR_PORT = 0;

    private final WPI_TalonFX intakeMotor = new WPI_TalonFX(INTAKE_MOTOR_PORT, "CANivore");
    private final WPI_TalonFX hopperMotor = new WPI_TalonFX(HOPPER_MOTOR_PORT, "CANivore");
    private final DigitalInput hopperSensor = new DigitalInput(HOPPER_SENSOR_PORT);
    private final SupplyCurrentLimitConfiguration MAX_AMPS = new SupplyCurrentLimitConfiguration(true, 10, 0, 0);
    //Pnuematics
    private DoubleSolenoid solenoid = new DoubleSolenoid(41, PneumaticsModuleType.CTREPCM, 0, 1);

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
        
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5000);
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 5000);
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 5000);
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 60);
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 5000);
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 5000);
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 5000);
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 5000);
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 20);
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 5000);
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5000);
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 5000);
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 5000);
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 5000);
        
        hopperMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        hopperMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5000);
        hopperMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 5000);
        hopperMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 5000);
        hopperMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 60);
        hopperMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 5000);
        hopperMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 5000);
        hopperMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 5000);
        hopperMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 5000);
        hopperMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 20);
        hopperMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 5000);
        hopperMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5000);
        hopperMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 5000);
        hopperMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 5000);
        hopperMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 5000);
    }

    /**
     * Determines whether there is a cargo in the sequencer.
     * 
     * @return  True if there is a cargo in the sequencer, otherwise false.
     */
    public boolean hasCargo() {
        return hopperSensor.get();
    }

    /**
     * Run the intake motor(s)
     * 
     * @param intakeSpeed  Velocity of the intake, in the range [-1,1]
     */
    public void spinIntake(double intakePct) {
        if(intakePct == 0)
            retract();
        else
            extend();
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
        //System.out.println("extend");
        solenoid.set(Value.kReverse);
        SmartDashboard.putBoolean("extended", true);
    }
    /** Retracts the intake. */
    public void retract() {
        //System.out.println("retract");
        solenoid.set(Value.kForward);
        SmartDashboard.putBoolean("extended", false);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("hopperSensor", hopperSensor.get());
    }
}
