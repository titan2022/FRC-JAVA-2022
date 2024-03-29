// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import static frc.robot.Constants

/** An example command that uses an example subsystem. */
public class ShooterCommand extends CommandBase {

    private ShooterSubsystem subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param speed = percent from -1 to 1
   */
    public ShooterCommand(ShooterSubsystem subsystem, double speed) {
        addRequirements(subsystem);
        this.subsystem = subsystem;
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.subsystem.shootPercent(Constants.XBOX_CONTROLLER.getTriggerAxis(Hand.kRight));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //schedule the second command
        //if gamepad button has been pressed since last updated than schedule
        
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
