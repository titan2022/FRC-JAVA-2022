package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Manual drive command for a holonomic drive base.
 */
public class HolonomicDriveCommand extends CommandBase {
    private final DriveSubsystem drivebase;
    private final XboxController controller;
    private double velocity;
    private double turnRate;

    /**
     * Creates a new HolonomicDriveCommand.
     * 
     * @param drivebase  The drive subsystem to control.
     * @param controller  The joystick controller to use.
     * @param turnRate  The angular velocity represented by moving the rotational
     *  joystick all the way to one side, in radians per second.
     * @param velocity  The translational velocity represented by moving the
     *  translational joystick all the way towards a cardinal direction, in
     *  meters per second.
     */
    public HolonomicDriveCommand(DriveSubsystem drivebase, XboxController controller, double turnRate, double velocity) {
        addRequirements(drivebase);
        this.drivebase = drivebase;
        this.controller = controller;
        this.turnRate = turnRate;
        this.velocity = velocity;
    }
    /**
     * Creates a new HolonomicDriveCommand.
     * 
     * The maximum translation velocity is assumed to be 10 meters per second.
     * 
     * @param drivebase  The drive subsystem to control.
     * @param controller  The joystick controller to use.
     * @param turnRate  The angular velocity represented by moving the rotational
     *  joystick all the way to one side.
     */
    public HolonomicDriveCommand(DriveSubsystem drivebase, XboxController controller, double turnRate) {
        this(drivebase, controller, turnRate, 10.);
    }
    /**
     * Creates a new HolonomicDriveCommand.
     * 
     * The maximum translation velocity is assumed to be 10 meters per second.
     * The maximum angular velocity is assumed to be pi radians per second.
     * 
     * @param drivebase  The drive subsystem to control.
     * @param controller  The joystick controller to use.
     */
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