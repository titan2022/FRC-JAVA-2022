
package frc.robot.commands;

import frc.robot.subsystems.SlothClimbSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SlothClimbCommand extends CommandBase {
  private SlothClimbSubsystem slothClimb;

  public SlothClimbCommand(SlothClimbSubsystem subsystem) {
    slothClimb = subsystem;

  }

  /**
   * Performs the full lift operation on the bars
   */
  public void performLift() {

  }
  
  public void pullUpStatic() {

  }

  public void pullUpDynamic() {

  }

  public void swingDynamicArm() {

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
