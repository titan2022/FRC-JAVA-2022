// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.subsystems.interfaces.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.*;  // I'm lazy, fix this later (codeShare doesn't automaticly import)

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DifferentialDriveSubsystems extends SubsystemBase implements DriveInterface {

  private static int rightMotorPort = 0;
  private static int leftMotorPort = 1;

  private static WPI_TalonFX leftMotor = new WPI_TalonFX(leftMotorPort);
  private static WPI_TalonFX rightMotor = new WPI_TalonFX(rightMotorPort);
  private static final double COUNTS_PER_REVOLUTION = 4096;
  private static final double WHEEL_DIAMETER_MM = 70;
  private static final double TRACK_WIDTH_METERS = 1;
  private static final int TICKS_PER_METER = (int) (COUNTS_PER_REVOLUTION / WHEEL_DIAMETER_MM * 1000);
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
  ChassisSpeeds chassis;
  DifferentialDriveWheelSpeeds wheelSpeeds;

  /** Distance, Velocity, Angle
   * @param velocity Rotions per second
   * @param angleDeg the angle in degrees to drive relative to the robot
   */
  public void drive(double velocity, double angleDeg) {
    chassis = new ChassisSpeeds(velocity, 0, angleDeg);
    wheelSpeeds = kinematics.toWheelSpeeds(chassis);
    rightMotor.set(ControlMode.Velocity, metersToTicks(wheelSpeeds.rightMetersPerSecond));
    leftMotor.set(ControlMode.Velocity, metersToTicks(wheelSpeeds.leftMetersPerSecond));
  }
  //meters per second to ticks per 100 milliseconds
  private static double metersToTicks(double x) {
    return (x * TICKS_PER_METER / 10) / (WHEEL_DIAMETER_MM/1000);
  }

  public void brake() {
    leftMotor.set(ControlMode.PercentOutput, 0);
    rightMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * @param coast whether the motors should coast or break once their velocity is set to 0
   */
  public void brakeMode(boolean coast) {
    NeutralMode neutralMode = coast ? NeutralMode.Brake : NeutralMode.Coast;
    leftMotor.setNeutralMode(neutralMode);
		rightMotor.setNeutralMode(neutralMode);
  }

  // TODO: a method to see if the motors are stalled?

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
