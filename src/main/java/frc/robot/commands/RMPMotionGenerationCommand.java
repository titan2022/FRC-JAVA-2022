package frc.robot.commands;

import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.titanrobotics2022.motion.generation.rmpflow.RMPRoot;

import org.ejml.simple.SimpleMatrix;

public class RMPMotionGenerationCommand extends CommandBase {

    private LocalizationSubsystem localization;
    private RMPRoot rmp;
    private SwerveDriveSubsystem drivebase;
    private double deltaT;

    public RMPMotionGenerationCommand(LocalizationSubsystem localization, RMPRoot rmp, SwerveDriveSubsystem drivebase,
            double deltaT) {
        this.localization = localization;
        this.rmp = rmp;
        this.drivebase = drivebase;
        this.deltaT = deltaT;
        addRequirements(localization);
    }

    public RMPMotionGenerationCommand(LocalizationSubsystem localization, RMPRoot rmp, SwerveDriveSubsystem drivebase) {
        this.localization = localization;
        this.rmp = rmp;
        this.drivebase = drivebase;
        deltaT = 0.02;
        addRequirements(localization);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        Translation2d loc0 = localization.getPred(0);
        Translation2d loc1 = localization.getPred(1);
        SimpleMatrix x = new SimpleMatrix(1, 2, false, new double[] { loc0.getX(), loc0.getY() });
        SimpleMatrix x_dot = new SimpleMatrix(1, 2, false, new double[] { loc1.getX(), loc1.getY() });

        SimpleMatrix x_ddot = rmp.solve(x, x_dot);
        x_dot.set(0, x_dot.get(0) + x_ddot.get(0) * deltaT);
        x_dot.set(1, x_dot.get(1) + x_ddot.get(1) * deltaT);
        drivebase.setVelocities(new ChassisSpeeds(x_dot.get(0), x_dot.get(1), 0));

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivebase.setVelocities(new ChassisSpeeds(0, 0, 0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
