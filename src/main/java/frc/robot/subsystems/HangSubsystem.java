package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

public class HangSubsystem{
    private static int winchMotorPort = 5;
    private static int encoderMotorPort = 1;
    private static WPI_TalonFX winchMotor = new WPI_TalonFX(winchMotorPort);
    private static CANCoder encoder = new CANCoder(encoderMotorPort);
    private static final StatorCurrentLimitConfiguration statorLimit = new StatorCurrentLimitConfiguration(true, 10 /*constant*/, 0, 0 /*spike*/);   
    private static final SupplyCurrentLimitConfiguration supplyLimit = new SupplyCurrentLimitConfiguration(true, 10, 0, 0); 
    public HangSubsystem(){
        //winchMotor.configRemoteFeedbackSensor(encoder, 0);
        //winchMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);
        winchMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        boolean Winch_Motor_Inversion = true;
        winchMotor.setInverted(Winch_Motor_Inversion);
        winchMotor.configNeutralDeadband(0.01);
        winchMotor.configSupplyCurrentLimit(supplyLimit);
        winchMotor.configStatorCurrentLimit(statorLimit);
        winchMotor.setNeutralMode(NeutralMode.Brake);
    }
    public static void reel(Double velocity){
        winchMotor.set(ControlMode.Velocity, velocity);
    }
    
}


