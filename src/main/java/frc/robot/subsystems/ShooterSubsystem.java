package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import static frc.robot.Constants.Unit.*;
import static frc.robot.Constants.getHoodConfig;

public class ShooterSubsystem extends SubsystemBase {
    /** Gear ratio between the hood motor and hood rack */
    private static final double HOOD_RATIO = 245.71;
    private static final double HOOD_MIN_ANGLE = 0.35 * DEG;
    private static final double HOOD_MAX_ANGLE = 27.2 * DEG;
    private static final double FLYWHEEL_RATIO = 1;
    private static final double FLYWHEEL_RADIUS = 2 * IN;
    private static final double SHOOTER_HEIGHT = 26.5 * IN;

    private static final int RIGHT_MOTOR_PORT = 16;
    private static final int LEFT_MOTOR_PORT = 11;
    private static final int HOOD_MOTOR_ID = 14;
    private static final int QUEUE_MOTOR_ID = 10;
    private static final int QUEUE_SENSOR_PORT = 2;

    private static final WPI_TalonFX rightMotor = new WPI_TalonFX(RIGHT_MOTOR_PORT);
    private static final WPI_TalonFX leftMotor = new WPI_TalonFX(LEFT_MOTOR_PORT);
    private static final WPI_TalonFX hoodMotor = new WPI_TalonFX(HOOD_MOTOR_ID);
    private static final WPI_TalonFX queueMotor = new WPI_TalonFX(QUEUE_MOTOR_ID);
    private static final Rev2mDistanceSensor distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
    private static final DigitalInput queueSensor = new DigitalInput(QUEUE_SENSOR_PORT);
    private static final ColorSensorV3 colorSensor = new ColorSensorV3(Port.kOnboard);

    private static final Color kRed = new Color(1, 0, 0);
    private static final Color kBlue = new Color(0, 0, 1);
    private static final Color kWhite = new Color(1, 1, 1);
    private final ColorMatch colorMatch = new ColorMatch();

    public static enum CargoColor {
        NONE, RED, BLUE;
    }

    public ShooterSubsystem(){
        leftMotor.configFactoryDefault();
        rightMotor.configFactoryDefault();
        hoodMotor.configFactoryDefault();
        queueMotor.configFactoryDefault();

        rightMotor.follow(leftMotor);
        rightMotor.setSensorPhase(false);
        leftMotor.setSensorPhase(false);
        leftMotor.setInverted(false);
        rightMotor.setInverted(true);

        rightMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        leftMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        leftMotor.config_kP(0, 0.1);
        leftMotor.config_kI(0, 0);
        leftMotor.config_kD(0, 0);
        rightMotor.config_kP(0, 0.1);
        rightMotor.config_kI(0, 0);
        rightMotor.config_kD(0, 0);

        leftMotor.setNeutralMode(NeutralMode.Coast);
        rightMotor.setNeutralMode(NeutralMode.Coast);

        hoodMotor.configAllSettings(getHoodConfig());
        hoodMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        hoodMotor.setInverted(false);
        hoodMotor.setSensorPhase(true);

        queueMotor.setInverted(false);
        queueMotor.setNeutralMode(NeutralMode.Brake);

        colorMatch.addColorMatch(kRed);
        colorMatch.addColorMatch(kBlue);
        colorMatch.addColorMatch(kWhite);
    }

    /**
     * Runs the shooter at a specified velocity.
     * 
     * @param speed The target velocity of the shooter in meters per second.
     */
    public void run(double speed) {
        leftMotor.set(ControlMode.Velocity,
                FLYWHEEL_RATIO * speed * (M / S) / FLYWHEEL_RADIUS * RAD / (FALCON_TICKS / (100 * MS)));
    }

    /**
     * Runs the shooter at by prcent output velocity.
     * 
     * @param percent The target velocity of the shooter, as a proportion of the
     *                maximum output, [-1,1].
     */
    public void runPercent(double percent) {
        leftMotor.set(ControlMode.PercentOutput, percent);
    }

    /** Turns off the shooter */
    public void coast() {
        leftMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Sets the angle of the hood.
     * 
     * This method does nothing if the specified angle is out of bounds.
     * 
     * @param radians The angle of inclination of the hood in radians. This is
     *                the complement of the angle of inclination of the initial
     *                velocity of a
     *                projectile launched from the shooter.
     */
    public void setAngle(double radians) {
        SmartDashboard.putNumber("Hood angle requested", radians * RAD / DEG);
        if(HOOD_MIN_ANGLE < radians && radians < HOOD_MAX_ANGLE){
            hoodMotor.set(ControlMode.Position, HOOD_RATIO * (radians * RAD - HOOD_MIN_ANGLE) / FALCON_TICKS);
            SmartDashboard.putNumber("Hood angle", radians * RAD / DEG);
        }
    }

    /**
     * Runs the queue motor at a specified percent output.
     * 
     * @param percent The output of the queue motor, in the range [-1,1].
     */
    public void runQueue(double percent) {
        queueMotor.set(ControlMode.PercentOutput, percent);
    }

    /**
     * Returns the current angle of the hood.
     * 
     * @return The current measure angle of inclination of the hood in radians.
     *         This is the complement of the angle of inclination of the initial
     *         velocity of a projectile launched from the shooter.
     */
    public double getAngle() {
        return hoodMotor.getSelectedSensorPosition() / HOOD_RATIO * FALCON_TICKS + HOOD_MIN_ANGLE;
    }

    /**
     * Returns the current velocity of the shooter.
     * 
     * @return The current measured velocity of the shooter in meters per second.
     */
    public double getVelocity() {
        double rawVel = (leftMotor.getSelectedSensorVelocity() + rightMotor.getSelectedSensorVelocity()) / 2;
        return rawVel * (FALCON_TICKS / (100 * MS)) / RAD * FLYWHEEL_RADIUS;
    }

    /**
     * Gets the current launch height of the shooter.
     * 
     * @return The current height of a projectile as it leaves the shooter, in
     *         meters.
     */
    public double getHeight() {
        return SHOOTER_HEIGHT;
    }

    /** Returns the maximum hood angle. */
    public double getMinAngle() {
        return HOOD_MIN_ANGLE;
    }
    /** Returns the minimum hood angle. */
    public double getMaxAngle() {
        return HOOD_MAX_ANGLE;
    }

    /**
     * Checks whether there is a cargo in the shooter.
     * 
     * @return True if a cargo is detected in the shooter, or false otherwise.
     */
    public boolean hasCargo() {
        return distanceSensor.getRange() != -1;
    }

    /**
     * Determines the color of the cargo currently in the queue.
     * 
     * @return  The color of the cargo currently in the queue.
     */
    public CargoColor getQueueColor() {
        ColorMatchResult result = colorMatch.matchClosestColor(colorSensor.getColor());
        if(result.color == kRed)
            return CargoColor.RED;
        else if(result.color == kBlue)
            return CargoColor.BLUE;
        else
            return CargoColor.NONE;
    }
    /**
     * Checks whether there is a cargo ready to be fed into the shooter.
     * 
     * @return True if a cargo is detected ready to be fed into the shooter,
     *         false otherwise.
     */
    public boolean hasQueue() {
        return getQueueColor() != CargoColor.NONE;
    }
}
