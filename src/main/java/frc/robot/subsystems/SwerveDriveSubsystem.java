package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import static frc.robot.Constants.Unit.*;

public class SwerveDriveSubsystem implements DriveSubsystem
{
  // Physical parameters
  public static final double ROBOT_TRACK_WIDTH = 23.5 * IN; // 0.672; // meters (30 in)
  public static final double ROBOT_LENGTH = 23.5 * IN; // 0.672; // meter 
  public static final double WHEEL_RADIUS = 2 * IN; // 0.0508; // meters (2 in)
  public static final double GEAR_RATIO = 6.86;
  public static final double METERS_PER_TICKS = WHEEL_RADIUS * 2 * Math.PI / FALCON_CPR / GEAR_RATIO;

  // Deadbands
  private static final double WHEEL_DEADBAND = 0.01;
  private static final double ROTATOR_DEADBAND = 0.001;
    
  // CAN ID numbers
  private static final int LEFT_FRONT_MOTOR_PORT = 6;
  private static final int LEFT_BACK_MOTOR_PORT = 4;
  private static final int RIGHT_FRONT_MOTOR_PORT = 7;
  private static final int RIGHT_BACK_MOTOR_PORT = 2;
  private static final int LEFT_FRONT_MOTOR_ROTATOR_PORT = 5;
  private static final int LEFT_BACK_MOTOR_ROTATOR_PORT = 3;
  private static final int RIGHT_FRONT_MOTOR_ROTATOR_PORT = 8;
  private static final int RIGHT_BACK_MOTOR_ROTATOR_PORT = 1;

  private static final int LEFT_FRONT_ENCODER_ROTATOR_PORT = 35;
  private static final int LEFT_BACK_ENCODER_ROTATOR_PORT = 33;
  private static final int RIGHT_FRONT_ENCODER_ROTATOR_PORT = 37;
  private static final int RIGHT_BACK_ENCODER_ROTATOR_PORT = 31;

  // Rotator encoder offsets
  private static final int FRONT_LEFT_OFFSET = 173;
  private static final int BACK_LEFT_OFFSET = 823;
  private static final int FRONT_RIGHT_OFFSET = 1158;
  private static final int BACK_RIGHT_OFFSET = 851;
  private static final int[] OFFSETS = new int[]{FRONT_LEFT_OFFSET, BACK_LEFT_OFFSET, FRONT_RIGHT_OFFSET, BACK_RIGHT_OFFSET};

  // Motor inversions
  private static final boolean WHEEL_INVERTED = false;
  private static final boolean ROTATOR_INVERTED = false;
 
  // Sensor inversions
  private static final boolean WHEEL_PHASE = false;
  private static final boolean ROTATOR_PHASE = false;

  // Physical limits of motors that create translational motion
  private static final double MAX_WHEEL_SPEED = 10 * M / S;
  private static final int PEAK_CURRENT_LIMIT = 12;
  private static final int CONTINUOUS_CURRENT_LIMIT = 12;
  private static final StatorCurrentLimitConfiguration statorCurrentLimit = new StatorCurrentLimitConfiguration(true,
      PEAK_CURRENT_LIMIT, 0, 0);
  private static final SupplyCurrentLimitConfiguration supplyCurrentLimit = new SupplyCurrentLimitConfiguration(true,
      CONTINUOUS_CURRENT_LIMIT, 0, 0);

  public static class Module {
    public static final int FRONT_LEFT = 0;
    public static final int BACK_LEFT = 1;
    public static final int FRONT_RIGHT = 2;
    public static final int BACK_RIGHT = 3;
  }

  // Physical Hardware
  private static final WPI_TalonFX[] motors = new WPI_TalonFX[]{
    new WPI_TalonFX(LEFT_FRONT_MOTOR_PORT),
    new WPI_TalonFX(LEFT_BACK_MOTOR_PORT),
    new WPI_TalonFX(RIGHT_FRONT_MOTOR_PORT),
    new WPI_TalonFX(RIGHT_BACK_MOTOR_PORT)
  };
  private static final WPI_TalonFX[] rotators = new WPI_TalonFX[]{
    new WPI_TalonFX(LEFT_FRONT_MOTOR_ROTATOR_PORT),
    new WPI_TalonFX(LEFT_BACK_MOTOR_ROTATOR_PORT),
    new WPI_TalonFX(RIGHT_FRONT_MOTOR_ROTATOR_PORT),
    new WPI_TalonFX(RIGHT_BACK_MOTOR_ROTATOR_PORT)
  };
  private static final CANCoder[] encoders = new CANCoder[]{
    new CANCoder(LEFT_FRONT_ENCODER_ROTATOR_PORT),
    new CANCoder(LEFT_BACK_ENCODER_ROTATOR_PORT),
    new CANCoder(RIGHT_FRONT_ENCODER_ROTATOR_PORT),
    new CANCoder(RIGHT_BACK_ENCODER_ROTATOR_PORT)
  };

  // PID slots
  private static final int ROTATOR_SLOT_IDX = 0;
  private static final int MAIN_MOTOR_SLOT_IDX = 0;

  // Kinematics
  // Positions describe the position of each wheel relative to the center of the robot
  private static final Translation2d leftFrontPosition = new Translation2d(-ROBOT_TRACK_WIDTH/2, ROBOT_LENGTH/2);
  private static final Translation2d leftBackPosition = new Translation2d(-ROBOT_TRACK_WIDTH/2, -ROBOT_LENGTH/2);
  private static final Translation2d rightFrontPosition = new Translation2d(ROBOT_TRACK_WIDTH/2, ROBOT_LENGTH/2);
  private static final Translation2d rightBackPosition = new Translation2d(ROBOT_TRACK_WIDTH/2, -ROBOT_LENGTH/2);
  public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(leftFrontPosition, leftBackPosition, rightFrontPosition, rightBackPosition);

  /**
   * Creates the swerve drive subsystem
   * 
   * @param mainConfig Requires PID configuration in slot 0
   * @param rotatorConfig Requires PID configuration in slot 0
   */
  public SwerveDriveSubsystem(TalonFXConfiguration mainConfig, TalonFXConfiguration rotatorConfig)
  {
    setFactoryMotorConfig();

    if(mainConfig != null)
    {
      motors[0].configAllSettings(mainConfig);
    }
    if(rotatorConfig != null){
      rotators[0].configAllSettings(rotatorConfig);
    }
    
    mainConfig = new TalonFXConfiguration();
    rotatorConfig = new TalonFXConfiguration();
    motors[0].getAllConfigs(mainConfig);
    rotators[0].getAllConfigs(rotatorConfig);

    // Current limits
    rotatorConfig.statorCurrLimit = statorCurrentLimit;
    rotatorConfig.supplyCurrLimit = supplyCurrentLimit;
    mainConfig.statorCurrLimit = statorCurrentLimit;
    mainConfig.supplyCurrLimit = supplyCurrentLimit;

    // Deadbands
    rotatorConfig.neutralDeadband = ROTATOR_DEADBAND;
    mainConfig.neutralDeadband = WHEEL_DEADBAND;

    // Apply configurations
    for(WPI_TalonFX motor : motors){
      motor.configAllSettings(mainConfig);
      motor.setInverted(WHEEL_INVERTED);
      motor.setSensorPhase(WHEEL_PHASE);
      motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
      motor.selectProfileSlot(MAIN_MOTOR_SLOT_IDX, 0);
    }
    for(WPI_TalonFX rotator : rotators){
      rotator.configAllSettings(rotatorConfig);
      rotator.setInverted(ROTATOR_INVERTED);
      rotator.setSensorPhase(ROTATOR_PHASE);
      rotator.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);
      rotator.selectProfileSlot(ROTATOR_SLOT_IDX, 0);
    }
    for(CANCoder encoder : encoders)
      encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, 0);
    for(int i=0; i<4; i++)
      rotators[i].configRemoteFeedbackFilter(encoders[i], 0);
  }
  public SwerveDriveSubsystem() {
    this(null, null);
  }

  private void setFactoryMotorConfig() {
    for(WPI_TalonFX motor : motors)
      motor.configFactoryDefault();
    for(WPI_TalonFX rotator : rotators)
      rotator.configFactoryDefault();
  }

  /**
   * Returns the current maximum drive speed in meters per second.
   * 
   * @return Maximum drive speed in meters per second.
   */
  public double getMaxSpeed()
  {
    return MAX_WHEEL_SPEED;
  }

  /**
   * Sets motor outputs using specified control mode
   * 
   * TODO: Swerve drive modules
   * 
   * @param mode             a ControlMode enum
   * @param leftOutputValue  left side output value for ControlMode
   * @param rightOutputValue right side output value for ControlMode
   */
  @Override
  public void setVelocities(ChassisSpeeds inputChassisSpeeds) {
    SwerveModuleState[] modules = kinematics.toSwerveModuleStates(inputChassisSpeeds);

    SwerveDriveKinematics.normalizeWheelSpeeds(modules, MAX_WHEEL_SPEED);
    SmartDashboard.putNumber("Norm FL vel", modules[0].speedMetersPerSecond);
    SmartDashboard.putNumber("Norm BL vel", modules[1].speedMetersPerSecond);
    SmartDashboard.putNumber("Norm FR vel", modules[2].speedMetersPerSecond);
    SmartDashboard.putNumber("Norm BR vel", modules[3].speedMetersPerSecond);
    SmartDashboard.putNumber("Norm FL tgt", modules[0].angle.getRadians() * RAD / ROT * CANCODER_CPR);
    SmartDashboard.putNumber("Norm BL tgt", modules[1].angle.getRadians() * RAD / ROT * CANCODER_CPR);
    SmartDashboard.putNumber("Norm FR tgt", modules[2].angle.getRadians() * RAD / ROT * CANCODER_CPR);
    SmartDashboard.putNumber("Norm BR tgt", modules[3].angle.getRadians() * RAD / ROT * CANCODER_CPR);
    SmartDashboard.putNumber("RawT FL vel", modules[0].speedMetersPerSecond/(10 * METERS_PER_TICKS));
    SmartDashboard.putNumber("RawT BL vel", modules[1].speedMetersPerSecond/(10 * METERS_PER_TICKS));
    SmartDashboard.putNumber("RawT FR vel", modules[2].speedMetersPerSecond/(10 * METERS_PER_TICKS));
    SmartDashboard.putNumber("RawT BR vel", modules[3].speedMetersPerSecond/(10 * METERS_PER_TICKS));
    
    //for(int i=0; i<4; i++)
      //modules[i] = SwerveModuleState.optimize(modules[i], new Rotation2d(getRotatorEncoderPosition((i&1)==0, i>1)));

    for(int i=0; i<4; i++){
      motors[i].set(ControlMode.Velocity, modules[i].speedMetersPerSecond/(10 * METERS_PER_TICKS));
      rotators[i].set(ControlMode.Position, modules[i].angle.getRadians() * RAD / ROT * CANCODER_CPR + OFFSETS[i]);
    }

    getSwerveModuleStates();
  }

  /**
   * Estimates robot velocity from wheel speeds.
   * 
   * @return  The estimated robot velocity.
   */
  public ChassisSpeeds getVelocities() {
    SwerveModuleState[] states = getSwerveModuleStates();
    return kinematics.toChassisSpeeds(states);
  }

  public void setOutput(double omega, double XVelocity, double YVelocity)
  {
    setVelocities(new ChassisSpeeds(XVelocity, YVelocity, omega));
  }

  // TODO: Fix all the brake logic and semantics because disabling brakes into coast mode is not about disabling brakes.

  /**
   * Enables brake.
   */
  public void enableBrakes() {
    stop();
    stopRotators();
    for(WPI_TalonFX motor : motors)
      motor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Disables brake.
   */
  public void disableBrakes() {
    for(WPI_TalonFX motor : motors)
      motor.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * Enables brake for rotator motors.
   */
  public void enableRotatorBrakes() {
    for(WPI_TalonFX rotator : rotators)
      rotator.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Disables brake for rotator motors.
   */
  public void disableRotatorBrakes() {
    for(WPI_TalonFX rotator : rotators)
      rotator.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * Stops the motors.
   */
  public void stop() {
    for(WPI_TalonFX motor : motors)
      motor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Stops the rotator motors.
   */
  public void stopRotators() {
    for(WPI_TalonFX rotator : rotators)
      rotator.set(ControlMode.PercentOutput, 0);
  }

  // encoder methods

  /**
   * Gets the encoder count for a primary motor.
   * 
   * @param useLeft - Whether to use the left primary motor.
   * @return Encoder count for specified primary motor.
   */
  public double getEncoderCount(int module)
  {
    return motors[module].getSelectedSensorPosition();
  }

  // encoder methods

  /**
   * Gets the encoder count for a primary motor.
   * 
   * @param useLeft - Whether to use the left primary motor.
   * @return Encoder count for specified primary motor.
   */
  public double getRotatorEncoderCount(int module)
  {
    return motors[module].getSelectedSensorPosition() * CANCODER_TICKS;
  }

  /**
   * Gets the amount of rotation from a primary motor.
   * 
   * @param useLeft - Whether to use the left primary motor.
   * @return Angle of rotator motor in radians
   */
  public double getRotatorEncoderPosition(int module) {
    return getRotatorEncoderCount(module);
  }

  public double getEncoderVelocity(int module)
  {
    return motors[module].getSelectedSensorVelocity() * METERS_PER_TICKS * 10;
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for(int i=0; i<4; i++)
      states[i] = new SwerveModuleState(getEncoderVelocity(i), new Rotation2d(getRotatorEncoderPosition(i)));
    return states;
  }

  @Override
  public void periodic() {}
}
