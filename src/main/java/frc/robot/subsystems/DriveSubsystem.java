package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Differential Drive Subsystem
 */
public class DriveSubsystem extends SubsystemBase {

  // port numbers to be added later
  public static final double ROBOT_DIAMETER = 42; // meter //TODO: Ask DI team for distance between wheels
  public static final double WHEEL_RADIUS = 3; // meters// TODO: Ask DI team for correct wheel radius
  public static final double METERS_PER_TICK = 0.1;

  // TODO: add constants to file later
  public static final int LEFT_PRIMARY_PORT = 1;
  public static final int LEFT_SECONDARY_PORT = 2;
  public static final int RIGHT_PRIMARY_PORT = 3;
  public static final int RIGHT_SECONDARY_PORT = 4;

  private TalonSRX leftPrimary, leftSecondary, rightPrimary, rightSecondary;

  private final static double MAX_SPEED = 10; // meters/sec

  // TODO: add encoders

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    // motor initialization block

    leftPrimary = new TalonSRX(LEFT_PRIMARY_PORT);
    leftSecondary = new TalonSRX(LEFT_SECONDARY_PORT);
    rightPrimary = new TalonSRX(RIGHT_PRIMARY_PORT);
    rightSecondary = new TalonSRX(RIGHT_SECONDARY_PORT);

    leftPrimary.configFactoryDefault();
    leftSecondary.configFactoryDefault();
    rightPrimary.configFactoryDefault();
    rightSecondary.configFactoryDefault();

    rightPrimary.setInverted(false);
    rightSecondary.setInverted(false);

    leftPrimary.setInverted(false);
    leftSecondary.setInverted(false);

    leftSecondary.follow(leftPrimary);
    rightSecondary.follow(rightPrimary);

    // Sets the direction that the talon will turn on the green LED when going 'forward'.
    leftPrimary.setSensorPhase(true);
    rightPrimary.setSensorPhase(true);

    // Current limits in amps
    leftPrimary.configPeakCurrentLimit(60);
    leftPrimary.configContinuousCurrentLimit(50);
    leftPrimary.enableCurrentLimit(true);

    rightPrimary.configPeakCurrentLimit(60);
    rightPrimary.configContinuousCurrentLimit(50);
    rightPrimary.enableCurrentLimit(true);
  }

  public DriveSubsystem(TalonSRXConfiguration leftConfig, TalonSRXConfiguration rightConfig) {
    this();

    leftPrimary.configAllSettings(leftConfig);
    leftSecondary.configAllSettings(leftConfig);
    rightPrimary.configAllSettings(rightConfig);
    rightSecondary.configAllSettings(rightConfig);
  }

  /**
   * Returns the current maximum drive speed in meters per second.
   * 
   * @return Maximum drive speed in meters per second.
   */
  public double getMaxSpeed() {
    return MAX_SPEED;
  }

  /**
   * Sets motor outputs using specified control mode
   * 
   * @param mode             a ControlMode enum
   * @param leftOutputValue  left side output value for ControlMode
   * @param rightOutputValue right side output value for ControlMode
   */
  public void setOutput(ControlMode mode, double leftOutputValue, double rightOutputValue) {
    if (mode == ControlMode.Velocity && leftOutputValue > MAX_SPEED) {
      leftOutputValue = MAX_SPEED;
    }

    if (mode == ControlMode.Velocity && rightOutputValue > MAX_SPEED) {
      rightOutputValue = MAX_SPEED;
    }

    // TODO: is check the current usage from Power Subsystem to restrict overcurrent
    leftPrimary.set(mode, leftOutputValue);
    rightPrimary.set(mode, rightOutputValue);
  }

  /**
   * Enables brake.
   */
  public void enableBrakes() {
    leftPrimary.setNeutralMode(NeutralMode.Brake);
    rightPrimary.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Disables brake.
   */
  public void disableBrakes() {
    leftPrimary.setNeutralMode(NeutralMode.Coast);
    rightPrimary.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * Stops the motors.
   */
  public void stop() {
    leftPrimary.set(ControlMode.PercentOutput, 0);
    rightPrimary.set(ControlMode.PercentOutput, 0);
  }

  // encoder methods

  /**
   * Gets the encoder count for a primary motor.
   * @param useLeft - Whether to use the left primary motor.
   * @return Encoder count for specified primary motor.
   */
  public double getEncoderCount(boolean useLeft) {

    if (useLeft) {

      return leftPrimary.getSelectedSensorPosition(0);
    
    } else {

      return rightPrimary.getSelectedSensorPosition(0);

    }

  }

  /**
   * Gets the distance from a primary motor.
   * @param useLeft - Whether to use the left primary motor.
   * @return Distance from specified primary motor.
   */
  public double getEncoderDist(boolean useLeft) {

    return getEncoderCount(useLeft) * METERS_PER_TICK;

  }
  
  /**
   * Gets current velocity of a primary motor.
   * @param useLeft - Whether to use the left primary motor.
   * @return Current velocity of a specified primary motor.
   */
  public double getEncoderVelocity(boolean useLeft) {

    if (useLeft) {

      return leftPrimary.getSelectedSensorVelocity(0) * METERS_PER_TICK;
    
    } else {

      return rightPrimary.getSelectedSensorVelocity(0) * METERS_PER_TICK;

    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}