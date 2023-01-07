package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ApplicationSubsystem extends SubsystemBase {

    private static int MOTOR_ID = 1;
    private WPI_TalonFX motor = new WPI_TalonFX(MOTOR_ID);
    private StatorCurrentLimitConfiguration statorLimit = new StatorCurrentLimitConfiguration(true, 20, 0, 0);
    private SupplyCurrentLimitConfiguration supplyLimit = new SupplyCurrentLimitConfiguration(true, 20, 0, 0);

    public ApplicationSubsystem(){
        motor.configStatorCurrentLimit(statorLimit, 5);
        motor.configSupplyCurrentLimit(supplyLimit, 10);
        motor.setNeutralMode(NeutralMode.Coast);
    }

    public void spin(double percent) {
        motor.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  
}
