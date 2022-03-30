package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.Constants.Unit;
import frc.robot.subsystems.I2CMuxer;
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
    //private static final int BEAM_BREAK_SENSOR_PORT = 1;

    private static final WPI_TalonFX rightMotor = new WPI_TalonFX(RIGHT_MOTOR_PORT);
    private static final WPI_TalonFX leftMotor = new WPI_TalonFX(LEFT_MOTOR_PORT);
    private static final WPI_TalonFX hoodMotor = new WPI_TalonFX(HOOD_MOTOR_ID);
    private static final WPI_TalonFX queueMotor = new WPI_TalonFX(QUEUE_MOTOR_ID);
    private static final ColorSensorV3 colorSensor = new ColorSensorV3(Port.kMXP);
    private static final Rev2mDistanceSensor distanceSensor = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kOnboard, Rev2mDistanceSensor.Unit.kMillimeters, RangeProfile.kDefault);

    private static final Color kRed = new Color(1, 0, 0);
    private static final Color kBlue = new Color(0, 0, 1);
    private static final Color kWhite = new Color(1, 1, 1);
    private final ColorMatch colorMatch = new ColorMatch();
    private static final I2CMuxer i2cMuxer= new I2CMuxer();

    public boolean queueEnabled = true;
    public boolean hoodEnabled = true;
    public boolean colorEnabled = true;
    public CargoColor colorOverride = CargoColor.NONE;

    public static enum CargoColor {
        NONE, RED, BLUE;
    }

    public CargoColor robotColor = CargoColor.RED;

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
        hoodMotor.setNeutralMode(NeutralMode.Brake);

        queueMotor.setInverted(false);
        queueMotor.setNeutralMode(NeutralMode.Brake);

        colorMatch.addColorMatch(kRed);
        colorMatch.addColorMatch(kBlue);
        colorMatch.addColorMatch(kWhite);
    }

    /**
     * Runs the shooter at a specified velocity.
     * 
     * @param speed  The target velocity of the shooter in meters per second.
     */
    public void run(double speed) {
        SmartDashboard.putNumber("Shooter vel requested", speed);
        SmartDashboard.putBoolean("Shooter mode velocity", true);
        leftMotor.set(ControlMode.Velocity,
            FLYWHEEL_RATIO * speed * (M / S) / FLYWHEEL_RADIUS * RAD / (FALCON_TICKS / (100 * MS)));
    }

    /**
     * Runs the shooter at by prcent output velocity.
     * 
     * @param percent  The target velocity of the shooter, as a proportion of the
     *  maximum output, [-1,1].
     */
    public void runPercent(double percent) {
        SmartDashboard.putNumber("Shooter vel requested", percent);
        SmartDashboard.putBoolean("Shooter mode velocity", false);
        leftMotor.set(ControlMode.PercentOutput, percent);
    }

    /** Turns off the shooter */
    public void coast() {
        runPercent(0);
    }

    /**
     * Sets the angle of the hood.
     * 
     * This method does nothing if the specified angle is out of bounds.
     * 
     * @param radians  The angle of inclination of the hood in radians. This is
     *  the complement of the angle of inclination of the initial velocity of a
     *  projectile launched from the shooter.
     */
    public void setAngle(double radians) {
        SmartDashboard.putNumber("Hood angle requested", radians * RAD / DEG);
        if(radians < HOOD_MIN_ANGLE){
            hoodMotor.set(ControlMode.Position, 0);
            SmartDashboard.putNumber("Hood angle", 0);
        }
        else if(radians > HOOD_MAX_ANGLE){
            hoodMotor.set(ControlMode.Position, HOOD_RATIO * (HOOD_MAX_ANGLE - HOOD_MIN_ANGLE) / FALCON_TICKS);
            SmartDashboard.putNumber("Hood angle", HOOD_MAX_ANGLE / DEG);
        }
        else{
            hoodMotor.set(ControlMode.Position, HOOD_RATIO * (radians * RAD - HOOD_MIN_ANGLE) / FALCON_TICKS);
            SmartDashboard.putNumber("Hood angle", radians * RAD / DEG);
        }
    }
    /**
     * Rotates the hood at a given relative speed.
     * 
     * @param percent  The rate to run the hood motor, in the range [-1,1]
     */
    public void spinHood(double percent) {
        double currentAngle = getAngle();
        SmartDashboard.putNumber("Hood angle", currentAngle / DEG);
        if(percent > 0 && currentAngle < getMaxAngle())
            hoodMotor.set(ControlMode.PercentOutput, percent * 0.2);
        else if(percent < 0 && currentAngle > getMinAngle())
            hoodMotor.set(ControlMode.PercentOutput, percent * 0.2);
        else
            hoodMotor.set(ControlMode.PercentOutput, 0);
    }

    /** Coasts the hood motor for easy manual actuation. */
    public void coastHood() {
        hoodMotor.setNeutralMode(NeutralMode.Coast);
        hoodMotor.set(ControlMode.PercentOutput, 0);
    }
    /** Brakes the hood motor to passively resist slippage. */
    public void brakeHood() {
        hoodMotor.setNeutralMode(NeutralMode.Brake);
        hoodMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Runs the queue motor at a specified percent output.
     * 
     * @param percent  The output of the queue motor, in the range [-1,1].
     */
    public void runQueue(double percent) {
        queueMotor.set(ControlMode.PercentOutput, percent);
    }

    /** Coasts the queue motor for easy manual actuation. */
    public void coastQueue() {
        queueMotor.setNeutralMode(NeutralMode.Coast);
        queueMotor.set(ControlMode.PercentOutput, 0);
    }
    /** Brakes the queue motor to passively resist slippage. */
    public void brakeQueue() {
        queueMotor.setNeutralMode(NeutralMode.Coast);
        queueMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Returns the current angle of the hood.
     * 
     * @return  The current measure angle of inclination of the hood in radians.
     *  This is the complement of the angle of inclination of the initial
     *  velocity of a projectile launched from the shooter.
     */
    public double getAngle() {
        return hoodMotor.getSelectedSensorPosition() / HOOD_RATIO * FALCON_TICKS + HOOD_MIN_ANGLE;
    }

    /**
     * Returns the current velocity of the shooter.
     * 
     * @return  The current measured velocity of the shooter in meters per second.
     */
    public double getVelocity() {
        double rawVel = (leftMotor.getSelectedSensorVelocity() + rightMotor.getSelectedSensorVelocity()) / 2;
        return rawVel * (FALCON_TICKS / (100 * MS)) / RAD * FLYWHEEL_RADIUS;
    }

    /**
     * Gets the current launch height of the shooter.
     * 
     * @return  The current height of a projectile as it leaves the shooter, in
     *  meters.
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
     * Finds distance to the ball from the Rev2mDistanceSensor at the bottom of the shooter
     * 
     * @return The distance to the cargo in millimeters
     */
    
    public double getCargoDistance() {
        i2cMuxer.setPort((byte) 1);
        return 1000 *distanceSensor.getRange(Rev2mDistanceSensor.Unit.kMillimeters);
    }


    /**
     * Needs to best tested to see what isRangeValid() does
     * 
     * Checks whether cargo is in shoooter
     * @return If ball is in shooter or not
     * 
     */
    public boolean hasCargo() {
        i2cMuxer.setPort((byte) 1);
        if (distanceSensor.isRangeValid()) 
            return true;
        else
            return false;
    }
    /**
     * Determines the color of the cargo currently in the queue.
     * 
     * @return  The color of the cargo currently in the queue.
     */
    public CargoColor getQueueColor() {
        if(!colorEnabled)
            return colorOverride;
        //0-7 for the port ids
        i2cMuxer.setPort((byte) 0);
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
     * @return  True if a cargo is detected ready to be fed into the shooter,
     *  false otherwise.
     */
    public boolean hasQueue() {
        return getQueueColor() != CargoColor.NONE;
    }

    public void sendDebug() {}

    public void periodic() {
        RawColor rawColor = colorSensor.getRawColor();
        SmartDashboard.putNumber("raw red", rawColor.red);
        SmartDashboard.putNumber("raw green", rawColor.green);
        SmartDashboard.putNumber("raw blue", rawColor.blue);
        SmartDashboard.putNumber("raw IR", rawColor.ir);
        Color normColor = colorSensor.getColor();
        SmartDashboard.putNumber("norm red", normColor.red);
        SmartDashboard.putNumber("norm green", normColor.green);
        SmartDashboard.putNumber("norm blue", normColor.blue);
        switch(getQueueColor()){
            case RED:
                SmartDashboard.putString("Cargo Color", "Red");
                break;
            case BLUE:
                SmartDashboard.putString("Cargo Color", "Blue");
                break;
            case NONE:
                SmartDashboard.putString("Cargo Color", "None");
                break;
            default:
                SmartDashboard.putString("Cargo Color", "Error");
                break;
        }
        SmartDashboard.putNumber("Hood angle", getAngle());
        SmartDashboard.putNumber("Shooter velocity", getVelocity());
        SmartDashboard.putBoolean("Flywheel Cargo", hasCargo());
    }

    /** Prepares the shooter for regular use. */
    public void enable() {
        brakeHood();
        brakeQueue();
    }
    /**
     * Turns off the shooter.
     * 
     * <p>WARNING: Calling this method does not prevent other methods from using
     * this subsystem. Do not manually actuate the shooter while the robot is
     * enabled.
     */
    public void disable() {
        coastHood();
        coastQueue();
        runPercent(0.0);
    }
}
