
package frc.robot.commands;

import frc.robot.subsystems.SlothClimbSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SlothClimbCommand extends CommandBase {

  private SlothClimbSubsystem slothClimb;
  //Length of arm in meters
  public static final double armLength = 0.415;

  public SlothClimbCommand(SlothClimbSubsystem subsystem) {
    slothClimb = subsystem;

  }
  /**
   * Gets the system on the bars by using the static arms firm
   */
  public void engageBar() {
    slothClimb.rotateDynamicArm(0.174533);
    this.moveStatic(1);
    this.moveStatic(-1);
  }

  /**
   * Full extends and contracts the static hook, which is a movement of around 41.5cm
   * 
   * @param percent = from -1 to 1, how much do you want to move the arm, negative values are for contraction, positive are for extension
   */
  public void moveStatic(double percent) {
    slothClimb.moveStaticArm(percent * armLength);
  }

  /**
   * Full extends and contracts the dynamic hook, which is a movement of around 41.5cm
   * 
   * @param percent = from -1 to 1, how much do you want to move the arm, negative values are for contraction, positive are for extension
   */
  public void moveDynamic(double percent) {
    slothClimb.moveDynamicArm(percent * armLength);
  }

  /**
   * Swings the dynamic arm
   * 
   * @param angle = angle to move in terms of radians
   */
  public void swingDynamicArm(double angle) {
    slothClimb.rotateDynamicArm(angle);
  }

  /**
   * Swings across one set of bars
   */
  public void swingBar() {
    //Hooks dynamic arm onto next bar
    slothClimb.rotateDynamicArm(0.610865);
    this.moveDynamic(1);
    slothClimb.rotateDynamicArm(-0.174533);
    this.moveDynamic(-0.8);

    //Release static arm from previous bar 
    this.moveStatic(1);
    slothClimb.rotateDynamicArm(-0.174533);
    this.moveStatic(-1);

    //Moves static arm onto current arm
    slothClimb.rotateDynamicArm(-0.610865);
    this.moveStatic(0.5);
    slothClimb.rotateDynamicArm(-0.174533);
    this.moveStatic(-0.5);

    //Unlocks dynamic arm
    slothClimb.rotateDynamicArm(0.174533);
    this.moveDynamic(-0.2);
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
