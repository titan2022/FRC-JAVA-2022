// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

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

  

  //Wheel parameters
  //Need to put in these variables manually
  //Input wheelDiameter as meters
  double wheelDiameter;
  double ticksInWheel;

  DifferentialDriveKinematics math = new DifferentialDriveKinematics(wheelDiameter);
  DifferentialDriveWheelSpeeds motorSpeeds;

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


  public WCDriveSubsystem() {

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

  public void brake() {
    leftMotor1.set(ControlMode.PercentOutput, 0);
    rightMotor1.set(ControlMode.PercentOutput, 0);
  }

  //Get estimate of the robot speed
  public double getSpeed() {
    return (motorSpeeds.leftMetersPerSecond + motorSpeeds.rightMetersPerSecond) / 2;
  }

  public double[] getPosition() {

    return new double[] {0, 0};
  }

}
