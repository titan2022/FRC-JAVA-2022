// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;

public class SlothClimbSubsystem extends SubsystemBase {

  //Number of ticks in a Falcon 500 to go around
  private static int motorTicks;
  private static final double ticksPerMeter = motorTicks * 393.701;

  /**
   * These are the motor ids of the TalonFXs
   * The ID1 is for the static arm's updown motion
   * The ID2 is for the non-static arm's updown motion
   * The ID3 is for the non-static arm's rotation
   */
  private int lMotorID1;
  private int lMotorID2;
  private int lMotorID3;
  private int rMotorID1;
  private int rMotorID2;
  private int rMotorID3;

  private double deadbandTolerance = 0.01;

  //The TalonFX motor controllers
  //The gear ration is 20:1 so for everyone 10 rotations of the Falcon 500s, 1in will be turned
  private WPI_TalonFX lMotor1= new WPI_TalonFX(lMotorID1);
  private WPI_TalonFX lMotor2= new WPI_TalonFX(lMotorID2);
  private WPI_TalonFX lMotor3= new WPI_TalonFX(lMotorID3);
  private WPI_TalonFX rMotor1= new WPI_TalonFX(rMotorID1);
  private WPI_TalonFX rMotor2= new WPI_TalonFX(rMotorID2);
  private WPI_TalonFX rMotor3= new WPI_TalonFX(rMotorID3);

  private StatorCurrentLimitConfiguration statorLimit = new StatorCurrentLimitConfiguration(true, 30, 0, 0);
  private SupplyCurrentLimitConfiguration supplyLimit = new SupplyCurrentLimitConfiguration(true, 30, 0, 0);

  // Creates the SlothClimb subsystem and configs the motors
  public SlothClimbSubsystem() {
    //Sets the stator power limit for the motors
    lMotor1.configStatorCurrentLimit(statorLimit, 10);
    lMotor2.configStatorCurrentLimit(statorLimit, 10);
    lMotor3.configStatorCurrentLimit(statorLimit, 10);
    rMotor1.configStatorCurrentLimit(statorLimit, 10);
    rMotor2.configStatorCurrentLimit(statorLimit, 10);
    rMotor3.configStatorCurrentLimit(statorLimit, 10);

    //Sets the supply power limit for the motors
    lMotor1.configSupplyCurrentLimit(supplyLimit, 10);
    lMotor2.configSupplyCurrentLimit(supplyLimit, 10);
    lMotor3.configSupplyCurrentLimit(supplyLimit, 10);
    rMotor1.configSupplyCurrentLimit(supplyLimit, 10);
    rMotor2.configSupplyCurrentLimit(supplyLimit, 10);
    rMotor3.configSupplyCurrentLimit(supplyLimit, 10);

    //Sets range of the motors to unsigned
    lMotor1.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360, 10);
    lMotor2.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360, 10);
    lMotor3.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360, 10);
    rMotor1.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360, 10);
    rMotor2.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360, 10);
    rMotor3.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360, 10);

    //Set phase, needs to be implemented later

    //Set deadbands
    lMotor1.configNeutralDeadband(deadbandTolerance);
    lMotor2.configNeutralDeadband(deadbandTolerance);
    lMotor3.configNeutralDeadband(deadbandTolerance);
    rMotor1.configNeutralDeadband(deadbandTolerance);
    rMotor2.configNeutralDeadband(deadbandTolerance);
    rMotor3.configNeutralDeadband(deadbandTolerance);

    //Aligns motors of the left and right side together
    rMotor1.follow(lMotor1);
    rMotor2.follow(lMotor2);
    rMotor3.follow(lMotor3);
  }

  /**
   * Moves the static arm up or down
   * @param meter = how much to move the arm in meter, positive for upwards, negative for downwards
   */
  public void moveStaticArm(double meter) {
    lMotor1.set(TalonFXControlMode.Position, meter * ticksPerMeter);
  }

  /**
   * Moves the non static arm up or down
   * @param meter = how much to move the arm in meter, positive for upwards, negative for downwards
   */
  public void moveDynamicArm(double meter) {
    lMotor2.set(TalonFXControlMode.Position, meter * ticksPerMeter);
  }

  /**
   * Rotates the non static arm by angle
   * 
   * @param angle = angle in radians, can be positive or negative
   */
  public void rotateDynamicArm(double angle) {
    lMotor2.set(TalonFXControlMode.Position, (angle / (2 * Math.PI)) * motorTicks * 20);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
