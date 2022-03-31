package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.TranslationalDrivebase;

import com.titanrobotics2022.mapping.LinearSegment;
import com.titanrobotics2022.mapping.Point;
import com.titanrobotics2022.motion.generation.rmpflow.rmps.PathFollowing;
import com.titanrobotics2022.motion.generation.rmpflow.RMPRoot;

public class RMPMoveToPositionCommand extends CommandBase {

    TranslationalDrivebase drivebase;
    private RMPRoot root;
    private PathFollowing leaf;
    private Translation2d goal;
    private LocalizationSubsystem localization;
    private LinearSegment path;
    private double v;
    private double tolerance;
    private double maxAcc;

    public RMPMoveToPositionCommand(TranslationalDrivebase drivebase, RMPRoot root, Translation2d goal,
            LocalizationSubsystem localization, double v, double tolerance, double maxAcc) {
        this.drivebase = drivebase;
        this.root = root;
        this.goal = goal;
        this.localization = localization;
        this.v = v;
        this.tolerance = tolerance;
        this.maxAcc = maxAcc;
    }

    @Override
    public void initialize() {
        Translation2d pos = localization.getPred(0);
        // Translation2d pos = new Translation2d(0, 0); // TEST
        path = new LinearSegment(new Point(pos), new Point(goal));
        double P = 5, I = 0, A = 1, B = 0.5, K = 1, h = 0.5;
        leaf = new PathFollowing("Path Following", root, path, v, P, I, A, B, K, h, maxAcc);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        root.unlinkChild(leaf);
        drivebase.setVelocity(new Translation2d(0, 0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("isFinished", goal.minus(localization.getPosition()).getNorm() < tolerance);
        return goal.minus(localization.getPosition()).getNorm() < tolerance;
    }

}
