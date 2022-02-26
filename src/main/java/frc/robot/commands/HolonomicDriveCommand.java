package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        this(drivebase, controller, turnRate, 5.);
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
        this(drivebase, controller, 4 * Math.PI);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double joyX = controller.getX(Hand.kLeft);
        double joyY = controller.getY(Hand.kLeft);
        double joyTurn = controller.getX(Hand.kRight);
        double fieldY = -Math.signum(joyY) * joyY * joyY * velocity;
        double fieldX = Math.signum(joyX) * joyX * joyX * velocity;
        double turn = Math.signum(joyTurn) * joyTurn * joyTurn * turnRate;
        if(Math.abs(joyX) < 0.1)
            fieldX = 0;
        if(Math.abs(joyY) < 0.1)
            fieldY = 0;
        if(Math.abs(joyTurn) < 0.1)
            turn = 0;
        SmartDashboard.putNumber("fieldX", fieldX);
        SmartDashboard.putNumber("fieldY", fieldY);
        SmartDashboard.putNumber("turn", turn);
        SmartDashboard.putNumber("xbox X", controller.getX(Hand.kLeft));
        SmartDashboard.putNumber("xbox Y", controller.getY(Hand.kLeft));
        SmartDashboard.putNumber("raw turn", controller.getX(Hand.kRight));
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