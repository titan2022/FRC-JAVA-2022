package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.ctre.phoenix.motorcontrol.*;

public class XanderClimbSubsystem {

    private static int rightMotorPort = 1;
    private static int leftMotorPort = 0;

    private static WPI_TalonFX leftMotor = new WPI_TalonFX(leftMotorPort);
    private static WPI_TalonFX rightMotor = new WPI_TalonFX(rightMotorPort);
    // private static DoubleSolenoid firstClaw = new
    // DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
    // private static DoubleSolenoid secondClaw = new
    // DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

    private static final StatorCurrentLimitConfiguration statorLimit = new StatorCurrentLimitConfiguration(true,
            10 /* constant */, 0, 0 /* spike */);
    private static final SupplyCurrentLimitConfiguration supplyLimit = new SupplyCurrentLimitConfiguration(true, 10, 0,
            0);
    boolean Rotate_Motor_Inversion = true;

    public static enum Claw {
        FirstClaw, SecondClaw
    }

    public XanderClimbSubsystem() {

        leftMotor.setInverted(Rotate_Motor_Inversion);
        leftMotor.configNeutralDeadband(0.01);
        leftMotor.configSupplyCurrentLimit(supplyLimit);
        leftMotor.configStatorCurrentLimit(statorLimit);
        leftMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.setInverted(Rotate_Motor_Inversion);
        rightMotor.configNeutralDeadband(0.01);
        rightMotor.configSupplyCurrentLimit(supplyLimit);
        rightMotor.configStatorCurrentLimit(statorLimit);
        rightMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.follow(leftMotor);
    }

    // 4096 ticks per rotation
    public void reel(double radians) {
        int tickCount = (int) (radians / (2 * Math.PI)) * 4096;
        leftMotor.set(ControlMode.Position, tickCount + leftMotor.getSelectedSensorPosition());
    }

    public double getReelTicks() {
        return leftMotor.getSelectedSensorPosition();
    }

    // public static void grab(Claw claw){
    // if(claw == Claw.FirstClaw) firstClaw.set(kForward);
    // else secondClaw.set(kForward);
    // }
    // public static void release(Claw claw){
    // if(claw == Claw.FirstClaw) firstClaw.set(kReverse);
    // else secondClaw.set(kReverse);
    // }

}
