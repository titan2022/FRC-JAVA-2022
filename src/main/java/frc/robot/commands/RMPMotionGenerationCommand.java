package frc.robot.commands;

import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.TranslationalDrivebase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.titanrobotics2022.motion.generation.rmpflow.RMPRoot;

import org.ejml.simple.SimpleMatrix;

public class RMPMotionGenerationCommand extends CommandBase {

    private LocalizationSubsystem localization;
    private RMPRoot root;
    private TranslationalDrivebase drivebase;
    private double deltaT;
    // private Translation2d pos = new Translation2d(0, 0); // TEST
    // private Translation2d vel = new Translation2d(0, 0); // TEST

    public RMPMotionGenerationCommand(LocalizationSubsystem localization, RMPRoot root,
            TranslationalDrivebase drivebase,
            double deltaT) {
        this.localization = localization;
        this.root = root;
        this.drivebase = drivebase;
        this.deltaT = deltaT;
        addRequirements(localization);
        addRequirements(drivebase);
    }

    public RMPMotionGenerationCommand(LocalizationSubsystem localization, RMPRoot root,
            TranslationalDrivebase drivebase) {
        this(localization, root, drivebase, 0.02);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        Translation2d pos = localization.getPred(0);
        Translation2d vel = localization.getPred(1);
        // Translation2d pos = new Translation2d(0, 0); // TEST
        // Translation2d vel = new Translation2d(1, 1); // TEST
        SimpleMatrix x = new SimpleMatrix(1, 2, false, new double[] { pos.getX(), pos.getY() });
        SimpleMatrix x_dot = new SimpleMatrix(1, 2, false, new double[] { vel.getX(), vel.getY() });
        SmartDashboard.putNumber("x0", x.get(0));
        SmartDashboard.putNumber("x1", x.get(1));
        SmartDashboard.putNumber("x_dot0", x_dot.get(0));
        SmartDashboard.putNumber("x_dot1", x_dot.get(1));
        System.out.printf("x: (%f, %f)\n", x.get(0), x.get(1));
        System.out.printf("x_dot: (%f, %f)\n", x_dot.get(0), x_dot.get(1));

        SimpleMatrix x_ddot = root.solve(x, x_dot);
        SmartDashboard.putNumber("x_ddot0", x_ddot.get(0));
        SmartDashboard.putNumber("x_ddot1", x_ddot.get(1));
        System.out.printf("x_ddot: (%f, %f)\n\n", x_ddot.get(0), x_ddot.get(1));
        x_dot.set(0, x_dot.get(0) + x_ddot.get(0) * deltaT);
        x_dot.set(1, x_dot.get(1) + x_ddot.get(1) * deltaT);
        SmartDashboard.putNumber("rmp vel0", x_dot.get(0));
        SmartDashboard.putNumber("rmp vel1", x_dot.get(1));
        drivebase.setVelocity(new Translation2d(x_dot.get(0), x_dot.get(1)));

        // vel = new Translation2d(x_dot.get(0), x_dot.get(1)); // TEST
        // pos = new Translation2d(pos.getX() + vel.getX() * deltaT, pos.getY(
        // etY() * deltaT); // TEST

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
