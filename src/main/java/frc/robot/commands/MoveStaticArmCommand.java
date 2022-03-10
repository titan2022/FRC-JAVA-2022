// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SlothClimbSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class MoveStaticArmCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final SlothClimbSubsystem subsystem;
  public static final double armLength = 0.415;
  private double movement;
  private boolean phase;

  
  /**
   * Creates a new MoveStaticArmCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param percent = How far to move the arm from -1 to 1
   */
  public MoveStaticArmCommand(SlothClimbSubsystem iSubsystem, double percent) {
    subsystem = iSubsystem;
    movement = percent;
    if (movement > 0) 
      phase = true;
    else 
      phase = false;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.moveStaticArm(movement * armLength);
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
    subsystem.setCalStaticTicks(subsystem.getSensorStaticTicks());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (phase) {
      if (subsystem.getSensorStaticTicks() >= subsystem.getCalStaticTicks()) {
        return true;
      } else {
        return false;
      }
    } else {
      if (subsystem.getSensorStaticTicks() <= subsystem.getCalStaticTicks()) {
        return true;
      } else {
        return false;
      }
    }
  }
}
