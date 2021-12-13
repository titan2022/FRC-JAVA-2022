package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class HangSubsystem{
    private static int WinchMotorPort = 5;
    private static int EncoderMotorPort = 1;
    private static WPI_TalonFX winchMotor = new WPI_TalonFX(WinchMotorPort);
    private static CANCoder encoder = new CANCoder(EncoderMotorPort);
    winchMotor.configRemoteFeedbackSensor(encoder, 0);
    public static void Raise(){

    }
    
}


