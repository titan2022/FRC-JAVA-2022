// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import frc.robot.subsystems.interfaces.DriveInterface;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPSubsystrivateField", "PMD.SingularField"})
  private final DriveInterface chassis;

  /**
   * Creates a new DifferentialDriveCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(DriveInterface chassis) {
    this.chassis = chassis;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double horizontal = Constants.controller.getX(Hand.kLeft);
    double vertical = Constants.controller.getY(Hand.kRight);
    chassis.drive(horizontal * 120, vertical * 10);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
