package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.DriveSubsystem;

public class HolonomicDriveCommand extends CommandBase {
    private final DriveSubsystem drivebase;
    private final XboxController controller;
    private double velocity;
    private double turnRate;

    public HolonomicDriveCommand(DriveSubsystem drivebase, XboxController controller, double turnRate, double velocity) {
        this.drivebase = drivebase;
        this.controller = controller;
        this.turnRate = turnRate;
        this.velocity = velocity;
    }
    public HolonomicDriveCommand(DriveSubsystem drivebase, XboxController controller, double turnRate) {
        this(drivebase, controller, turnRate, 10.);
    }
    public HolonomicDriveCommand(DriveSubsystem drivebase, XboxController controller) {
        this(drivebase, controller, Math.PI);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double fieldY = controller.getY(Hand.kLeft) * velocity;
        double fieldX = controller.getX(Hand.kLeft) * velocity;
        double turn = controller.getX(Hand.kRight) * turnRate;
        drivebase.setVelocities(new ChassisSpeeds(fieldY, fieldX, turn));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted)
            new StartEndCommand(() -> {
                    controller.setRumble(RumbleType.kLeftRumble, 0.5);
                    controller.setRumble(RumbleType.kRightRumble, 0.5);
                }, () -> {
                    controller.setRumble(RumbleType.kLeftRumble, 0);
                    controller.setRumble(RumbleType.kRightRumble, 0);
                }).withTimeout(0.5).schedule();
    }
}