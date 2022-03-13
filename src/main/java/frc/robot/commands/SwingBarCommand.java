// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SlothClimbSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class SwingBarCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  //Swings from bar to another bar, can be chained together back to back to swing across multiple bars
  public SwingBarCommand(SlothClimbSubsystem subsystem) {
    addCommands(
      //Hooks dynamic arm onto next bar
      new RotateArmCommand(subsystem, 0.610865),
      new MoveDynamicArmCommand(subsystem, 1),
      new RotateArmCommand(subsystem, -0.174533),
      new MoveDynamicArmCommand(subsystem, -0.8),

      //Release static arm from previous bar 
      new MoveStaticArmCommand(subsystem, 1),
      new RotateArmCommand(subsystem, -0.174533),
      new MoveStaticArmCommand(subsystem, -1),

      //Moves static arm onto current arm
      new RotateArmCommand(subsystem, -0.610865),
      new MoveStaticArmCommand(subsystem, 0.5),
      new RotateArmCommand(subsystem, -0.174533),
      new MoveStaticArmCommand(subsystem, -0.5),

      //Unlocks dynamic arm
      new RotateArmCommand(subsystem, 0.174533),
      new MoveDynamicArmCommand(subsystem, -0.2)
    );

    //addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
