// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double COUNTS_PER_REVOLUTION = 4096;
    public static final XboxController XBOX_CONTROLLER = new XboxController(1);

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
        public static final double FALCON_TICKS = FALCON_CPR / ROT;
        public static final double CANCODER_TICKS = FALCON_CPR / ROT;
        public static final double SECONDS = 1.0, S = 1.0;
        public static final double MIN = 1.0/60;
        public static final double MS = 0.001;
    }

    /**
     * Contains a velocity based PID configuration.
     * @return TalonFX Configuration Object
     */
    public static TalonFXConfiguration getSwerveDriveTalonDirectionalConfig()
    {
        TalonFXConfiguration talon = new TalonFXConfiguration();
        // Add configs here:
        talon.slot0.kP = 1000;
        talon.slot0.kI = 0;  // 250
        talon.slot0.kD = 0;        
        talon.slot0.kF = 0;
        talon.slot0.integralZone = 900;
        talon.slot0.allowableClosedloopError = 217;
        talon.slot0.maxIntegralAccumulator = 254.000000;
        //talon.slot0.closedLoopPeakOutput = 0.869990; // Sets maximum output of the PID controller
        //talon.slot0.closedLoopPeriod = 33; // Sets the hardware update rate of the PID controller

        return talon;
    }

    /**
     * Contains a position based PID configuration
     * @return TalonFX Configuration Object
     */
    public static TalonFXConfiguration getSwerveDriveTalonRotaryConfig()
    {
        TalonFXConfiguration talon = new TalonFXConfiguration();
        // Add configs here:
        talon.slot0.kP = 10.;
        talon.slot0.kI = 0;
        talon.slot0.kD = 0;
        talon.slot0.kF = 0;
        talon.slot0.integralZone = 900;
        talon.slot0.allowableClosedloopError = 20;//217;
        talon.slot0.maxIntegralAccumulator = 254.000000;
        //talon.slot0.closedLoopPeakOutput = 0.869990; // Sets maximum output of the PID controller
        //talon.slot0.closedLoopPeriod = 33; // Sets the hardware update rate of the PID controller

        return talon;
    }
}
