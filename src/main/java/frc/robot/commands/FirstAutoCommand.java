package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TranslationalDrivebase;

import static frc.robot.Constants.Unit.*;

public class FirstAutoCommand extends SequentialCommandGroup {
    public FirstAutoCommand(TranslationalDrivebase drive, ShooterSubsystem shooter, IntakeSubsystem intake, ClimbSubsystem climb) {
        addCommands(
            new StartEndCommand(() -> climb.runClimb(0.6), () -> climb.runClimb(0.0), climb).withTimeout(4.0),
            new InstantCommand(() -> {intake.spinIntake(1.0); intake.spinHopper(1.0);}),
            new DriveDistance(drive, 167 * IN, 5, 2),
            new InstantCommand(() -> {intake.spinIntake(0.0); intake.spinHopper(0.0);}),
            new InstantCommand(() -> {shooter.run(7.9); shooter.setAngle(30.179 * DEG);}),
            new WaitUntilCommand(() -> Math.abs(shooter.getVelocity() - 7.9) < 0.05 && Math.abs(shooter.getAngle() - 30.179*DEG) < 0.5),
            new InstantCommand(() -> {intake.spinHopper(1.0); shooter.runQueue(1.0);}),
            new WaitCommand(3.0),
            new InstantCommand(() -> {intake.spinHopper(0.0); shooter.runQueue(0.0); shooter.runPercent(0.0);})
        );
    }
}
