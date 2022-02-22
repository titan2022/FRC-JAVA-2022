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
  private static final int LEFT_FRONT_MOTOR_PORT = 7;
  private static final int LEFT_BACK_MOTOR_PORT = 3;
  private static final int RIGHT_FRONT_MOTOR_PORT = 2;
  private static final int RIGHT_BACK_MOTOR_PORT = 4;
  private static final int LEFT_FRONT_MOTOR_ROTATOR_PORT = 0;
  private static final int LEFT_BACK_MOTOR_ROTATOR_PORT = 1;
  private static final int RIGHT_FRONT_MOTOR_ROTATOR_PORT = 5;
  private static final int RIGHT_BACK_MOTOR_ROTATOR_PORT = 6;

  private static final int LEFT_FRONT_ENCODER_ROTATOR_PORT = 9;
  private static final int LEFT_BACK_ENCODER_ROTATOR_PORT = 10;
  private static final int RIGHT_FRONT_ENCODER_ROTATOR_PORT = 8;
  private static final int RIGHT_BACK_ENCODER_ROTATOR_PORT = 11;

  // Rotator encoder offsets
  private static final int FRONT_LEFT_OFFSET = 173;
  private static final int BACK_LEFT_OFFSET = 823;
  private static final int FRONT_RIGHT_OFFSET = 1158;
  private static final int BACK_RIGHT_OFFSET = 851;

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

  // Physical limits of motors that rotate the wheel. Use radians.
  
  // Physical Hardware
  private static final WPI_TalonFX leftFrontMotor = new WPI_TalonFX(LEFT_FRONT_MOTOR_PORT)
    , leftBackMotor = new WPI_TalonFX(LEFT_BACK_MOTOR_PORT)
    , rightFrontMotor = new WPI_TalonFX(RIGHT_FRONT_MOTOR_PORT)
    , rightBackMotor = new WPI_TalonFX(RIGHT_BACK_MOTOR_PORT)
    , leftFrontRotatorMotor = new WPI_TalonFX(LEFT_FRONT_MOTOR_ROTATOR_PORT)
    , leftBackRotatorMotor = new WPI_TalonFX(LEFT_BACK_MOTOR_ROTATOR_PORT)
    , rightFrontRotatorMotor = new WPI_TalonFX(RIGHT_FRONT_MOTOR_ROTATOR_PORT)
    , rightBackRotatorMotor = new WPI_TalonFX(RIGHT_BACK_MOTOR_ROTATOR_PORT);
  
    private static final CANCoder leftFrontRotatorEncoder = new CANCoder(LEFT_FRONT_ENCODER_ROTATOR_PORT)
      , leftBackRotatorEncoder = new CANCoder(LEFT_BACK_ENCODER_ROTATOR_PORT)
      , rightFrontRotatorEncoder = new CANCoder(RIGHT_FRONT_ENCODER_ROTATOR_PORT)
      , rightBackRotatorEncoder = new CANCoder(RIGHT_BACK_ENCODER_ROTATOR_PORT);

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
      leftFrontMotor.configAllSettings(mainConfig);
    }
    if(rotatorConfig != null){
      leftFrontRotatorMotor.configAllSettings(rotatorConfig);
    }
    
    mainConfig = new TalonFXConfiguration();
    rotatorConfig = new TalonFXConfiguration();
    leftFrontMotor.getAllConfigs(mainConfig);
    leftFrontRotatorMotor.getAllConfigs(rotatorConfig);

    // Current limits
    rotatorConfig.statorCurrLimit = statorCurrentLimit;
    rotatorConfig.supplyCurrLimit = supplyCurrentLimit;
    mainConfig.statorCurrLimit = statorCurrentLimit;
    mainConfig.supplyCurrLimit = supplyCurrentLimit;

    // Deadbands
    rotatorConfig.neutralDeadband = ROTATOR_DEADBAND;
    mainConfig.neutralDeadband = WHEEL_DEADBAND;

    // Apply persistent configurations
    leftFrontMotor.configAllSettings(mainConfig);
    leftBackMotor.configAllSettings(mainConfig);
    rightFrontMotor.configAllSettings(mainConfig);
    rightBackMotor.configAllSettings(mainConfig);

    leftFrontRotatorMotor.configAllSettings(rotatorConfig);
    leftBackRotatorMotor.configAllSettings(rotatorConfig);
    rightFrontRotatorMotor.configAllSettings(rotatorConfig);
    rightBackRotatorMotor.configAllSettings(rotatorConfig);

    // Sensor initilizations
    leftFrontRotatorEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, 0);
    leftBackRotatorEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, 0);
    rightFrontRotatorEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, 0);
    rightBackRotatorEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, 0);

    // Motor inversions
    rightFrontMotor.setInverted(WHEEL_INVERTED);
    rightBackMotor.setInverted(WHEEL_INVERTED);
    rightFrontRotatorMotor.setInverted(WHEEL_INVERTED);
    rightBackRotatorMotor.setInverted(WHEEL_INVERTED);

    leftFrontMotor.setInverted(ROTATOR_INVERTED);
    leftBackMotor.setInverted(ROTATOR_INVERTED);
    leftFrontRotatorMotor.setInverted(ROTATOR_INVERTED);
    leftBackRotatorMotor.setInverted(ROTATOR_INVERTED);

    // Sensor inversions
    leftFrontMotor.setSensorPhase(WHEEL_PHASE);
    rightFrontMotor.setSensorPhase(WHEEL_PHASE);
    leftBackMotor.setSensorPhase(WHEEL_PHASE);
    rightBackMotor.setSensorPhase(WHEEL_PHASE);
    leftFrontRotatorMotor.setSensorPhase(ROTATOR_PHASE);
    rightFrontRotatorMotor.setSensorPhase(ROTATOR_PHASE);
    leftBackRotatorMotor.setSensorPhase(ROTATOR_PHASE);
    rightBackRotatorMotor.setSensorPhase(ROTATOR_PHASE);

    // Configure feedback sensors
    leftFrontRotatorMotor.configRemoteFeedbackFilter(leftFrontRotatorEncoder, 0);
    leftBackRotatorMotor.configRemoteFeedbackFilter(leftBackRotatorEncoder, 0);
    rightFrontRotatorMotor.configRemoteFeedbackFilter(rightFrontRotatorEncoder, 0);
    rightBackRotatorMotor.configRemoteFeedbackFilter(rightBackRotatorEncoder, 0);

    // Select feedback sensors
    leftFrontRotatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);
    leftBackRotatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);
    rightBackRotatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);
    rightFrontRotatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);

    leftFrontMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    rightFrontMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    leftBackMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    rightBackMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    // Select PID slots
    leftFrontRotatorMotor.selectProfileSlot(ROTATOR_SLOT_IDX, 0);
    rightFrontRotatorMotor.selectProfileSlot(ROTATOR_SLOT_IDX, 0);
    leftBackRotatorMotor.selectProfileSlot(ROTATOR_SLOT_IDX, 0);
    rightBackRotatorMotor.selectProfileSlot(ROTATOR_SLOT_IDX, 0);

    leftFrontMotor.selectProfileSlot(MAIN_MOTOR_SLOT_IDX, 0);
    rightFrontMotor.selectProfileSlot(MAIN_MOTOR_SLOT_IDX, 0);
    leftBackMotor.selectProfileSlot(MAIN_MOTOR_SLOT_IDX, 0);
    rightBackMotor.selectProfileSlot(MAIN_MOTOR_SLOT_IDX, 0);
  }
  public SwerveDriveSubsystem() {
    this(null, null);
  }

  private void setFactoryMotorConfig() {
    leftFrontMotor.configFactoryDefault();
    leftBackMotor.configFactoryDefault();
    rightFrontMotor.configFactoryDefault();
    rightBackMotor.configFactoryDefault();
    leftFrontRotatorMotor.configFactoryDefault();
    leftBackRotatorMotor.configFactoryDefault();
    rightFrontRotatorMotor.configFactoryDefault();
    rightBackRotatorMotor.configFactoryDefault();
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
    SmartDashboard.putNumber("State FL vel", modules[0].speedMetersPerSecond);
    SmartDashboard.putNumber("State BL vel", modules[1].speedMetersPerSecond);
    SmartDashboard.putNumber("State FR vel", modules[2].speedMetersPerSecond);
    SmartDashboard.putNumber("State BR vel", modules[3].speedMetersPerSecond);
    SmartDashboard.putNumber("State FL deg", modules[0].angle.getDegrees());
    SmartDashboard.putNumber("State BL deg", modules[1].angle.getDegrees());
    SmartDashboard.putNumber("State FR deg", modules[2].angle.getDegrees());
    SmartDashboard.putNumber("State BR deg", modules[3].angle.getDegrees());

    SwerveDriveKinematics.normalizeWheelSpeeds(modules, MAX_WHEEL_SPEED);
    SmartDashboard.putNumber("Norm FL vel", modules[0].speedMetersPerSecond);
    SmartDashboard.putNumber("Norm BL vel", modules[1].speedMetersPerSecond);
    SmartDashboard.putNumber("Norm FR vel", modules[2].speedMetersPerSecond);
    SmartDashboard.putNumber("Norm BR vel", modules[3].speedMetersPerSecond);
    SmartDashboard.putNumber("Norm FL deg", modules[0].angle.getDegrees());
    SmartDashboard.putNumber("Norm BL deg", modules[1].angle.getDegrees());
    SmartDashboard.putNumber("Norm FR deg", modules[2].angle.getDegrees());
    SmartDashboard.putNumber("Norm BR deg", modules[3].angle.getDegrees());
    SmartDashboard.putNumber("Norm FL tgt", modules[0].angle.getRadians() * RAD / ROT * CANCODER_CPR);
    SmartDashboard.putNumber("Norm BL tgt", modules[1].angle.getRadians() * RAD / ROT * CANCODER_CPR);
    SmartDashboard.putNumber("Norm FR tgt", modules[2].angle.getRadians() * RAD / ROT * CANCODER_CPR);
    SmartDashboard.putNumber("Norm BR tgt", modules[3].angle.getRadians() * RAD / ROT * CANCODER_CPR);
    
    //for(int i=0; i<4; i++)
      //modules[i] = SwerveModuleState.optimize(modules[i], new Rotation2d(getRotatorEncoderPosition((i&1)==0, i>1)));

    leftFrontMotor.set(ControlMode.Velocity, modules[0].speedMetersPerSecond/(10 * METERS_PER_TICKS));
    leftBackMotor.set(ControlMode.Velocity, modules[1].speedMetersPerSecond/(10 * METERS_PER_TICKS));
    rightFrontMotor.set(ControlMode.Velocity, modules[2].speedMetersPerSecond/(10 * METERS_PER_TICKS));
    rightBackMotor.set(ControlMode.Velocity, modules[3].speedMetersPerSecond/(10 * METERS_PER_TICKS));
    /*double percentVelocity = Math.sqrt(Math.pow(XboxMap.translationX(), 2) + Math.pow(XboxMap.translationY(), 2));
    leftFrontMotor.set(ControlMode.PercentOutput, percentVelocity);
    leftBackMotor.set(ControlMode.PercentOutput, percentVelocity);
    rightFrontMotor.set(ControlMode.PercentOutput, percentVelocity);
    rightBackMotor.set(ControlMode.PercentOutput, percentVelocity);*/

    SmartDashboard.putNumber("FL Tgt Raw Opt", modules[0].angle.getRadians() * RAD / CANCODER_TICKS /*+ 1162*/ + FRONT_RIGHT_OFFSET);
    SmartDashboard.putNumber("BL Tgt Raw Opt", modules[1].angle.getRadians() * RAD / CANCODER_TICKS /*+ 1162*/ + FRONT_RIGHT_OFFSET);
    SmartDashboard.putNumber("FR Tgt Raw Opt", modules[2].angle.getRadians() * RAD / CANCODER_TICKS /*+ 1162*/ + FRONT_RIGHT_OFFSET);
    SmartDashboard.putNumber("BR Tgt Raw Opt", modules[3].angle.getRadians() * RAD / CANCODER_TICKS /*+ 1162*/ + FRONT_RIGHT_OFFSET);
    
    leftFrontRotatorMotor.set(ControlMode.Position, modules[0].angle.getRadians() * RAD / CANCODER_TICKS /*+ 166*/ + FRONT_LEFT_OFFSET);
    leftBackRotatorMotor.set(ControlMode.Position, modules[1].angle.getRadians() * RAD / CANCODER_TICKS /*- 1807*/ + BACK_LEFT_OFFSET);
    rightFrontRotatorMotor.set(ControlMode.Position, modules[2].angle.getRadians() * RAD / CANCODER_TICKS/*+ 1162*/ + FRONT_RIGHT_OFFSET);
    rightBackRotatorMotor.set(ControlMode.Position, modules[3].angle.getRadians() * RAD / CANCODER_TICKS /*+ 710*/ + BACK_RIGHT_OFFSET);

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
    leftFrontMotor.setNeutralMode(NeutralMode.Brake);
    rightFrontMotor.setNeutralMode(NeutralMode.Brake);
    leftBackMotor.setNeutralMode(NeutralMode.Brake);
    rightBackMotor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Disables brake.
   */
  public void disableBrakes() {
    leftFrontMotor.setNeutralMode(NeutralMode.Coast);
    rightFrontMotor.setNeutralMode(NeutralMode.Coast);
    leftBackMotor.setNeutralMode(NeutralMode.Coast);
    rightBackMotor.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * Enables brake for rotator motors.
   */
  public void enableRotatorBrakes() {
    leftFrontRotatorMotor.setNeutralMode(NeutralMode.Brake);
    rightFrontRotatorMotor.setNeutralMode(NeutralMode.Brake);
    leftBackRotatorMotor.setNeutralMode(NeutralMode.Brake);
    rightBackRotatorMotor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Disables brake for rotator motors.
   */
  public void disableRotatorBrakes() {
    leftFrontRotatorMotor.setNeutralMode(NeutralMode.Coast);
    rightFrontRotatorMotor.setNeutralMode(NeutralMode.Coast);
    leftBackRotatorMotor.setNeutralMode(NeutralMode.Coast);
    rightBackRotatorMotor.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * Stops the motors.
   */
  public void stop() {
    leftFrontMotor.set(ControlMode.PercentOutput, 0);
    rightFrontMotor.set(ControlMode.PercentOutput, 0);
    leftBackMotor.set(ControlMode.PercentOutput, 0);
    rightBackMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Stops the rotator motors.
   */
  public void stopRotators() {
    leftFrontRotatorMotor.set(ControlMode.PercentOutput, 0);
    rightFrontRotatorMotor.set(ControlMode.PercentOutput, 0);
    leftBackRotatorMotor.set(ControlMode.PercentOutput, 0);
    rightBackRotatorMotor.set(ControlMode.PercentOutput, 0);
  }

  // encoder methods

  /**
   * Gets the encoder count for a primary motor.
   * 
   * @param useLeft - Whether to use the left primary motor.
   * @return Encoder count for specified primary motor.
   */
  public double getEncoderCount(boolean useLeft, boolean useBack)
  {
    if (useLeft)
    {
      if (useBack){
        SmartDashboard.putNumber("read BL m", leftBackMotor.getSelectedSensorPosition(0)*METERS_PER_TICKS);
        return leftBackMotor.getSelectedSensorPosition(0);
      }
      else{
        SmartDashboard.putNumber("read FL m", leftFrontMotor.getSelectedSensorPosition(0)*METERS_PER_TICKS);
        return leftFrontMotor.getSelectedSensorPosition(0);
      }
    }
    else
    {
      if (useBack){
        SmartDashboard.putNumber("read BR m", rightBackMotor.getSelectedSensorPosition(0)*METERS_PER_TICKS);
        return rightBackMotor.getSelectedSensorPosition(0);
      }
      else{
        SmartDashboard.putNumber("read FR m", rightFrontMotor.getSelectedSensorPosition(0)*METERS_PER_TICKS);
        return rightFrontMotor.getSelectedSensorPosition(0);
      }
    }
  }

  // encoder methods

  /**
   * Gets the encoder count for a primary motor.
   * 
   * @param useLeft - Whether to use the left primary motor.
   * @return Encoder count for specified primary motor.
   */
  public double getRotatorEncoderCount(boolean useLeft, boolean useBack)
  {
    if (useLeft)
    {
      if (useBack){
        SmartDashboard.putNumber("raw BL rot", leftBackRotatorMotor.getSelectedSensorPosition(0));
        return leftBackRotatorMotor.getSelectedSensorPosition(0) * CANCODER_TICKS;
      }
      else{
        SmartDashboard.putNumber("raw FL rot", leftFrontRotatorMotor.getSelectedSensorPosition(0));
        return leftFrontRotatorMotor.getSelectedSensorPosition(0) * CANCODER_TICKS;
      }
    }
    else
    {
      if (useBack){
        SmartDashboard.putNumber("raw BR rot", rightBackRotatorMotor.getSelectedSensorPosition(0));
        return rightBackRotatorMotor.getSelectedSensorPosition(0) * CANCODER_TICKS;
      }
      else{
        SmartDashboard.putNumber("raw FR rot", rightFrontRotatorMotor.getSelectedSensorPosition(0));
        return rightFrontRotatorMotor.getSelectedSensorPosition(0) * CANCODER_TICKS;
      }
    }
  }

  /**
   * Gets the amount of rotation from a primary motor.
   * 
   * @param useLeft - Whether to use the left primary motor.
   * @return Angle of rotator motor in radians
   */
  public double getRotatorEncoderPosition(boolean useLeft, boolean useBack) {
    return getRotatorEncoderCount(useLeft, useBack);
  }

  public double getEncoderVelocity(boolean useLeft, boolean useBack)
  {
    if (useLeft)
    {
      if (useBack){
        SmartDashboard.putNumber("read BL m/s", leftBackMotor.getSelectedSensorVelocity(0) * METERS_PER_TICKS * 10);
        return leftBackMotor.getSelectedSensorVelocity(0) * METERS_PER_TICKS * 10;
      }
      else{
        SmartDashboard.putNumber("read FL m/s", leftFrontMotor.getSelectedSensorVelocity(0) * METERS_PER_TICKS * 10);
        return leftFrontMotor.getSelectedSensorVelocity(0) * METERS_PER_TICKS * 10;
      }
    }
    else
    {
      if (useBack){
        SmartDashboard.putNumber("read BR m/s", rightBackMotor.getSelectedSensorVelocity(0) * METERS_PER_TICKS * 10);
        return rightBackMotor.getSelectedSensorVelocity(0) * METERS_PER_TICKS * 10;
      }
      else{
        SmartDashboard.putNumber("read FR m/s", rightFrontMotor.getSelectedSensorVelocity(0) * METERS_PER_TICKS * 10);
        return rightFrontMotor.getSelectedSensorVelocity(0) * METERS_PER_TICKS * 10;
      }
    }
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    SwerveModuleState leftBack = new SwerveModuleState(getEncoderVelocity(true, true),
        new Rotation2d(getRotatorEncoderPosition(true, true)));

    SwerveModuleState leftFront = new SwerveModuleState(getEncoderVelocity(true, false),
        new Rotation2d(getRotatorEncoderPosition(true, false)));

    SwerveModuleState rightBack = new SwerveModuleState(getEncoderVelocity(false, true),
        new Rotation2d(getRotatorEncoderPosition(false, true)));

    SwerveModuleState rightFront = new SwerveModuleState(getEncoderVelocity(false, false),
        new Rotation2d(getRotatorEncoderPosition(false, false)));
    
    return new SwerveModuleState[]{leftFront, leftBack, rightFront, rightBack};
  }

  @Override
  public void periodic() {
    getEncoderVelocity(false, false);
    getEncoderVelocity(false, true);
    getEncoderVelocity(true, false);
    getEncoderVelocity(true, true);
  }
}
