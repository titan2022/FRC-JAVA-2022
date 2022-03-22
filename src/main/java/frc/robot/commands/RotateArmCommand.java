// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SlothClimbSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class RotateArmCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final SlothClimbSubsystem subsystem;
  private double angle;
  private boolean phase;

  /**
   * Creates a new RotateArmCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param angle = How much to rotate in terms of radians
   */
  public RotateArmCommand(SlothClimbSubsystem iSubsystem, double angle) {
    subsystem = iSubsystem;
    this.angle = angle;
    if (this.angle > 0) 
      phase = true;
    else 
      phase = false;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.rotateDynamicArm(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isFinished())
      end(false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.setCalRotationTicks(subsystem.getSensorRotationTicks());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (phase) {
      if (subsystem.getSensorRotationTicks() >= subsystem.getCalRotationTicks()) {
        return true;
      } else {
        return false;
      }
    } else {
      if (subsystem.getSensorRotationTicks() <= subsystem.getCalRotationTicks()) {
        return true;
      } else {
        return false;
      }
    }
  }
}
