// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDriveSubsystem extends SubsystemBase {

  private static double speed = 0.5;
  //Motor IDs
  private int lMotorID1;
  private int lMotorID2;
  private int rMotorID1;
  private int rMotorID2;

  //WPI_TalonFX takes in ticks / 100ms 
  private TalonSRX leftMotor1 = new TalonSRX(lMotorID1);
  private TalonSRX leftMotor2 = new TalonSRX(lMotorID2);
  private TalonSRX rightMotor1 = new TalonSRX(rMotorID1);
  private TalonSRX rightMotor2 = new TalonSRX(rMotorID2);
  
  //Motor ampere configurations
  SupplyCurrentLimitConfiguration motorSupplyLimit = new SupplyCurrentLimitConfiguration(true, 10, 0, 0);

  public TankDriveSubsystem() {
    //Set supply power draw limits for falcon motors
    leftMotor1.configSupplyCurrentLimit(motorSupplyLimit);
    leftMotor2.configSupplyCurrentLimit(motorSupplyLimit);
    rightMotor1.configSupplyCurrentLimit(motorSupplyLimit);
    rightMotor2.configSupplyCurrentLimit(motorSupplyLimit);

  }

  /**
   * Spins the left motors of the tank drive
   * @param spin Percentage from -1 to 1
   */
  public void spinLeftWheel(double perc) {
    leftMotor1.set(TalonSRXControlMode.PercentOutput, perc * speed);
    leftMotor2.set(TalonSRXControlMode.PercentOutput, perc * speed);
  }

  /**
   * Spins the right motors of the tank drive
   * @param spin Percentage from -1 to 1
   */
  public void spinRightWheel(double perc) {
    rightMotor1.set(TalonSRXControlMode.PercentOutput, perc * speed);
    rightMotor2.set(TalonSRXControlMode.PercentOutput, perc* speed);
  }

  /**
   * Sets the speed of the robot in terms of percentage of maximum speed
   * @param perc From 0 - 1
   */
  public void setSpeed(double perc) {
    speed = perc;
  }

  //Stops robot in place
  public void brake() {
    leftMotor1.set(ControlMode.PercentOutput, 0);
    rightMotor1.set(ControlMode.PercentOutput, 0);
  }

}
