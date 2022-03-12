package frc.robot.commands;

import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.TranslationalDrivebase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.titanrobotics2022.motion.generation.rmpflow.RMPRoot;

import org.ejml.simple.SimpleMatrix;

public class RMPMotionGenerationCommand extends CommandBase {

    private LocalizationSubsystem localization;
    private RMPRoot root;
    private TranslationalDrivebase drivebase;
    private double deltaT;

    public RMPMotionGenerationCommand(LocalizationSubsystem localization, RMPRoot root,
            TranslationalDrivebase drivebase,
            double deltaT) {
        this.localization = localization;
        this.root = root;
        this.drivebase = drivebase;
        this.deltaT = deltaT;
        addRequirements(localization);
    }

    public RMPMotionGenerationCommand(LocalizationSubsystem localization, RMPRoot root,
            TranslationalDrivebase drivebase) {
        this.localization = localization;
        this.root = root;
        this.drivebase = drivebase;
        deltaT = 0.02;
        addRequirements(localization);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        Translation2d loc0 = localization.getPred(0);
        Translation2d loc1 = localization.getPred(1);
        // Translation2d loc0 = new Translation2d(1, 4); // TEST
        // Translation2d loc1 = new Translation2d(3, 2); // TEST
        SimpleMatrix x = new SimpleMatrix(1, 2, false, new double[] { loc0.getX(), loc0.getY() });
        SimpleMatrix x_dot = new SimpleMatrix(1, 2, false, new double[] { loc1.getX(), loc1.getY() });
        // System.out.printf("x: (%f, %f)\n", x.get(0), x.get(1));
        // System.out.printf("x_dot: (%f, %f)\n", x_dot.get(0), x_dot.get(1));

        SimpleMatrix x_ddot = root.solve(x, x_dot);
        // System.out.printf("x_ddot: (%f, %f)\n\n", x_ddot.get(0), x_ddot.get(1));
        x_dot.set(0, x_dot.get(0) + x_ddot.get(0) * deltaT);
        x_dot.set(1, x_dot.get(1) + x_ddot.get(1) * deltaT);
        drivebase.setVelocity(new Translation2d(x_dot.get(0), x_dot.get(1)));

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivebase.setVelocity(new Translation2d(0, 0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
