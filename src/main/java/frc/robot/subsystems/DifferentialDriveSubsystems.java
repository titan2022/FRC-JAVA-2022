// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DifferentialDriveSubsystems extends SubsystemBase {
  WPI_TalonFX left_wheel = new WPI_TalonFX(10);
  WPI_TalonFX right_wheel = new WPI_TalonFX(12);
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    
  }
  public void forward(double distance, double velocity, double angle){
    
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
