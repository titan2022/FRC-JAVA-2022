package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.XanderClimbSubsystem;

public class XClimbReelCommand extends CommandBase {

    public XanderClimbSubsystem sub;
    public double radians;
    public boolean isDone;
    public double initialPos;

    public XClimbReelCommand(XanderClimbSubsystem sub, double radians) {
        this.sub = sub;
        this.radians = radians;
    }

    public boolean isFinished() {
        double netGoal = ((radians / (2 * Math.PI)) * 4096) + initialPos;

        if (netGoal > initialPos && sub.getReelTicks() >= netGoal) {
            return true;
        } else if (netGoal < initialPos && sub.getReelTicks() <= netGoal) {
            return true;
        } else {
            return false;
        }
    }

    public void initialize() {
        initialPos = sub.getReelTicks();
        sub.reel(radians);
    }

    public void execute() {
    }

}
