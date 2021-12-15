// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.WCDriveSubsystem;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class TankDriveCommand extends CommandBase {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  //Drivebase
  WCDriveSubsystem differentialDrive;

  //Max speed of wheels
  double speed = 10;
  
  /**
   * 
   * @param subsystem = what subsystem is being used by the command
   */
  public TankDriveCommand(WCDriveSubsystem subsystem) {
    differentialDrive = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    differentialDrive.rotateLeftWheel(Constants.controller.getY(Hand.kLeft) * speed);
    differentialDrive.rotateRightWheel(Constants.controller.getY(Hand.kRight) * speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
