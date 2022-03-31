// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.TankDriveSubsystem;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class ManualTankDriveCommand extends CommandBase {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  //Drivebase
  private static final TankDriveSubsystem tankDrive = new TankDriveSubsystem();

  //Max speed of wheels
  private double speed = 10;
  
  public ManualTankDriveCommand() {
    addRequirements(tankDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tankDrive.spinLeftWheel(Constants.controller.getY(Hand.kLeft));
    tankDrive.spinRightWheel(Constants.controller.getY(Hand.kRight));
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
