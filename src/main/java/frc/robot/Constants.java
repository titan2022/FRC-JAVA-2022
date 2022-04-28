// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double COUNTS_PER_REVOLUTION = 4096;
    public static final XboxController XBOX_CONTROLLER = new XboxController(0);

    public static final class Unit {
        public static final double METERS = 1.0, M = 1.0;
        public static final double CM = 0.01, MM = 0.001;
        public static final double IN = Units.inchesToMeters(1);
        public static final double FT = Units.feetToMeters(1);
        public static final double RAD = 1.0;
        public static final double DEG = Units.degreesToRadians(1);
        public static final double ROT = Math.PI * 2;
        public static final double FALCON_CPR = 2048;
        public static final double CANCODER_CPR = 4096;
        public static final double FALCON_TICKS = ROT / FALCON_CPR;
        public static final double CANCODER_TICKS = ROT / CANCODER_CPR;
        public static final double SECONDS = 1.0, S = 1.0;
        public static final double MIN = 1.0 / 60;
        public static final double MS = 0.001;
    }

    public static final class Cargo {
        public static final Translation2d LEFT = new Translation2d(-81.643 * Unit.IN, -129.396 * Unit.IN);
        public static final Translation2d CENTER = new Translation2d(88.3033 * Unit.IN, -124.946 * Unit.IN);
        public static final Translation2d RIGHT = new Translation2d(150.790 * Unit.IN, -25.910 * Unit.IN);
        public static final Translation2d OPP_LEFT = new Translation2d(-124.946 * Unit.IN, -88.303 * Unit.IN);
        public static final Translation2d OPP_CENTER = new Translation2d(33.767 * Unit.IN, -149.227 * Unit.IN);
        public static final Translation2d OPP_RIGHT = new Translation2d(149.227 * Unit.IN, 33.767 * Unit.IN);
        public static final Translation2d STATION = new Translation2d(119.799 * Unit.IN, -284.248 * Unit.IN);
        public static final Translation2d HUMAN = new Translation2d(118.762 * Unit.IN, -283.164 * Unit.IN);
    }

    /**
     * Contains a velocity based PID configuration.
     * 
     * @return TalonFX Configuration Object
     */
    public static TalonFXConfiguration getSwerveDriveTalonDirectionalConfig() {
        TalonFXConfiguration talon = new TalonFXConfiguration();
        // Add configs here:
        talon.slot0.kP = 0.06;
        talon.slot0.kI = 0; // 250
        talon.slot0.kD = 0.2;
        talon.slot0.kF = 0.045;
        talon.slot0.integralZone = 900;
        talon.slot0.allowableClosedloopError = 20;
        talon.slot0.maxIntegralAccumulator = 254.000000;
        // talon.slot0.closedLoopPeakOutput = 0.869990; // Sets maximum output of the
        // PID controller
        // talon.slot0.closedLoopPeriod = 33; // Sets the hardware update rate of the
        // PID controller

        return talon;
    }

    /**
     * Contains a position based PID configuration
     * 
     * @return TalonFX Configuration Object
     */
    public static TalonFXConfiguration getSwerveDriveTalonRotaryConfig() {
        TalonFXConfiguration talon = new TalonFXConfiguration();
        // Add configs here:
        talon.slot0.kP = 0.45;
        talon.slot0.kI = 0.005;
        talon.slot0.kD = 0;
        talon.slot0.kF = 0;
        talon.slot0.integralZone = 75;
        talon.slot0.allowableClosedloopError = 5;// 217;
        talon.slot0.maxIntegralAccumulator = 5120;
        // talon.slot0.closedLoopPeakOutput = 0.869990; // Sets maximum output of the
        // PID controller
        // talon.slot0.closedLoopPeriod = 33; // Sets the hardware update rate of the
        // PID controller

        return talon;
    }

    public static TalonFXConfiguration getHoodConfig(double min, double max, double ratio) {
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        hoodConfig.neutralDeadband = 0;
        hoodConfig.peakOutputForward = 0.3;
        hoodConfig.peakOutputReverse = -0.3;
        hoodConfig.primaryPID.selectedFeedbackCoefficient = 1;
        hoodConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        hoodConfig.slot0.allowableClosedloopError = 80;
        hoodConfig.slot0.closedLoopPeakOutput = 0.2;
        hoodConfig.slot0.kP = 0.45;
        hoodConfig.slot0.kI = 0;
        hoodConfig.slot0.kD = 0;
        hoodConfig.slot0.kF = 0;
        hoodConfig.statorCurrLimit.enable = false;
        hoodConfig.supplyCurrLimit.currentLimit = 10;
        hoodConfig.supplyCurrLimit.enable = true;
        hoodConfig.reverseSoftLimitEnable = true;
        hoodConfig.reverseSoftLimitThreshold = 0;
        hoodConfig.forwardSoftLimitEnable = true;
        hoodConfig.forwardSoftLimitThreshold = (max - min) * ratio / Unit.FALCON_TICKS;
        return hoodConfig;
    }

    public static TalonFXConfiguration getShooterConfig() {
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        shooterConfig.slot0.kP = 0.3; // 0.3
        shooterConfig.slot0.kI = 0;
        shooterConfig.slot0.kD = 0;
        shooterConfig.slot0.kF = 0.0475; // 0.0475
        shooterConfig.slot0.allowableClosedloopError = 0;
        shooterConfig.slot0.closedLoopPeriod = 1;
        shooterConfig.slot0.integralZone = 300;
        shooterConfig.slot0.maxIntegralAccumulator = 5000;
        shooterConfig.supplyCurrLimit.currentLimit = 40;
        shooterConfig.supplyCurrLimit.enable = false;
        shooterConfig.supplyCurrLimit.triggerThresholdCurrent = 40;
        shooterConfig.supplyCurrLimit.triggerThresholdTime = 0.1;
        return shooterConfig;
    }

    public static void setTalonStatusFrames(WPI_TalonFX motor) {
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20); // 20 for open loop
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 5000);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 100);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 60);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 50);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 5000);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 5000);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 5000);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 5000);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5000);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 5000);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 5000);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 5000);
    }
}
