package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LocalizationSubsystem;
import com.titanrobotics2022.mapping.LinearSegment;
import com.titanrobotics2022.mapping.Point;
import com.titanrobotics2022.motion.generation.rmpflow.rmps.PathFollowing;
import com.titanrobotics2022.motion.generation.rmpflow.RMPRoot;

public class RMPMoveToPositionCommand extends CommandBase {

    private RMPRoot root;
    private PathFollowing leaf;
    private Translation2d goal;
    private LocalizationSubsystem localization;
    private LinearSegment path;
    private double v;

    public RMPMoveToPositionCommand(RMPRoot root, Translation2d goal, LocalizationSubsystem localization, double v) {
        this.root = root;
        this.goal = goal;
        this.localization = localization;
        this.v = v;
    }

    @Override
    public void initialize() {
        Translation2d pos = localization.getPred(0);
        path = new LinearSegment(new Point(pos), new Point(goal));
        double P = 0.1, I = 0, A = 1, B = 1, K = 1, h = 0.5;
        leaf = new PathFollowing("Path Following", root, path, v, P, I, A, B, K, h);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        root.unlinkChild(leaf);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
