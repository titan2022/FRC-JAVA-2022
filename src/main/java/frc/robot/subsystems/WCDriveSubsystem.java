// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

public class WCDriveSubsystem extends SubsystemBase {

  //Motor IDs
  int lMotorID1;
  int lMotorID2;
  int rMotorID1;
  int rMotorID2;

  //Encoder IDs
  int lEncoderID1;
  int lEncoderID2;
  int rEncoderID1;
  int rEncoderID2;

  //WPI_TalonFX takes in ticks / 100ms 
  private WPI_TalonFX leftMotor1 = new WPI_TalonFX(lMotorID1);
  private WPI_TalonFX leftMotor2 = new WPI_TalonFX(lMotorID2);
  private WPI_TalonFX rightMotor1 = new WPI_TalonFX(rMotorID1);
  private WPI_TalonFX rightMotor2 = new WPI_TalonFX(rMotorID2);
  
  //Motor encoders
  private CANCoder leftEncoder1 = new CANCoder(lEncoderID1);
  private CANCoder leftEncoder2 = new CANCoder(lEncoderID2);
  private CANCoder rightEncoder1 = new CANCoder(rEncoderID1);
  private CANCoder rightEncoder2 = new CANCoder(rEncoderID2);

  //Wheel parameters
  //Need to put in these variables manually
  //Input wheelDiameter as meters
  double wheelDiameter;
  double ticksInWheel;

  //Motor ampere configurations
  StatorCurrentLimitConfiguration motorStatorLimit = new StatorCurrentLimitConfiguration(true, 10, 0, 0);
  SupplyCurrentLimitConfiguration motorSupplyLimit = new SupplyCurrentLimitConfiguration(true, 10, 0, 0);

  //Deadband range
  double motorDeadBand = 0.01;

  //Drive parameters
  DifferentialDriveKinematics math = new DifferentialDriveKinematics(wheelDiameter);
  DifferentialDriveWheelSpeeds motorSpeeds;

  public WCDriveSubsystem() {

    //Finds remote encoders
    leftMotor1.configRemoteFeedbackFilter(leftEncoder1, 0);
    leftMotor2.configRemoteFeedbackFilter(leftEncoder2, 0);
    rightMotor1.configRemoteFeedbackFilter(rightEncoder1, 0);
    rightMotor2.configRemoteFeedbackFilter(rightEncoder2, 0);

    //Use remote encoders
    leftMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);
    leftMotor2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);
    rightMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);
    rightMotor2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);
  
    //Initializaiton strategies
    //Whether to use offset or not
    //In thise case zero is adapative and set at initalization
    leftEncoder1.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    leftEncoder2.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    rightEncoder1.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    rightEncoder2.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);

    //Phase
    //Which direction to rotate
    //Still needs work
    leftMotor1.setSensorPhase(true);
    leftMotor2.setSensorPhase(true);
    rightMotor1.setSensorPhase(true);
    rightMotor2.setSensorPhase(true);

    //Offset
    //Which part of the motor is zero
    //No need to configure

    //Range
    //Which range of degrees to work with, ex: 0 to 2pi or -pi to pi
    //No need to configure

    //Set stator power draw limits for falcon motors
    leftMotor1.configStatorCurrentLimit(motorStatorLimit);
    leftMotor2.configStatorCurrentLimit(motorStatorLimit);
    rightMotor1.configStatorCurrentLimit(motorStatorLimit);
    rightMotor2.configStatorCurrentLimit(motorStatorLimit);

    //Set supply power draw limits for falcon motors
    leftMotor1.configSupplyCurrentLimit(motorSupplyLimit);
    leftMotor2.configSupplyCurrentLimit(motorSupplyLimit);
    rightMotor1.configSupplyCurrentLimit(motorSupplyLimit);
    rightMotor2.configSupplyCurrentLimit(motorSupplyLimit);

    //Deadbands
    leftMotor1.configNeutralDeadband(motorDeadBand);
    leftMotor2.configNeutralDeadband(motorDeadBand);
    rightMotor1.configNeutralDeadband(motorDeadBand);
    rightMotor2.configNeutralDeadband(motorDeadBand);

    //Sets each side's motor to follow each other
    leftMotor2.follow(leftMotor1);
    rightMotor2.follow(rightMotor1);
  }

  /**
   * 
   * @param speed = meters per sec
   * @param angle = how far to turn in radians
   */
  public void move(double speed, double angle) {
    DifferentialDriveWheelSpeeds motorSpeeds = math.toWheelSpeeds(new ChassisSpeeds(speed, 0, angle));
    leftMotor1.set((motorSpeeds.leftMetersPerSecond * ticksInWheel) / (10 * Math.PI * wheelDiameter));
    rightMotor1.set((motorSpeeds.rightMetersPerSecond * ticksInWheel) / (10 * Math.PI * wheelDiameter));
  }
  
  /**
   * 
   * @param spin = meters per sec
   */
  public void rotateLeftWheel(double spin) {
    leftMotor1.set((spin * ticksInWheel) / (10 * Math.PI * wheelDiameter));
  }

  /**
   * 
   * @param spin = meters per sec
   */
  public void rotateRightWheel(double spin) {
    rightMotor1.set((spin * ticksInWheel) / (10 * Math.PI * wheelDiameter));
  }

  //Stops robot in place
  public void brake() {
    leftMotor1.set(ControlMode.PercentOutput, 0);
    rightMotor1.set(ControlMode.PercentOutput, 0);
  }

  //Get estimate of the robot speed
  public double getSpeed() {
    return (motorSpeeds.leftMetersPerSecond + motorSpeeds.rightMetersPerSecond) / 2;
  }


  //N/A yet
  //Needs gyroscope class first to work
  public double[] getPosition() {

    return new double[] {0, 0};
  }

}
