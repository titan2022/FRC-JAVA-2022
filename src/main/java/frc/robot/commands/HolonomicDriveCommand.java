package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;

/**
 * Manual drive command for a holonomic drive base.
 */
public class HolonomicDriveCommand extends CommandBase {
    private final DriveSubsystem drivebase;
    private final XboxController controller;
    private final LocalizationSubsystem nav;
    private double velocity;
    private double turnRate;
    private boolean fieldOrientation = true;

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
    public HolonomicDriveCommand(DriveSubsystem drivebase, XboxController controller, LocalizationSubsystem nav, double turnRate, double velocity) {
        addRequirements(drivebase);
        this.drivebase = drivebase;
        this.controller = controller;
        this.turnRate = turnRate;
        this.velocity = velocity;
        this.nav = nav;
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
    public HolonomicDriveCommand(DriveSubsystem drivebase, XboxController controller, LocalizationSubsystem nav, double turnRate) {
        this(drivebase, controller, nav, turnRate, 5.);
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
    public HolonomicDriveCommand(DriveSubsystem drivebase, XboxController controller, LocalizationSubsystem nav) {
        this(drivebase, controller, nav, 4 * Math.PI);
    }

    @Override
    public void initialize() {}

    private static double applyDeadband(double joy, double deadband) {
        return Math.abs(joy) < deadband ? 0 : joy;
    }
    
    private double scaleVelocity(double joy) {
        return Math.signum(joy) * joy * joy * velocity;
    }

    @Override
    public void execute() {
        if(controller.getBButtonPressed())
            fieldOrientation = !fieldOrientation;
        if(controller.getAButtonReleased())
            nav.resetHeading();
        double joyX = applyDeadband(-controller.getLeftX(), 0.1);
        double joyY = applyDeadband(-controller.getLeftY(), 0.1);
        double joyTurn = applyDeadband(controller.getRightX(), 0.1);
        Translation2d fieldVel = new Translation2d(scaleVelocity(joyX), scaleVelocity(joyY));
        Translation2d robotVel = fieldOrientation ? fieldVel.rotateBy(nav.getOrientation()) : new Translation2d(scaleVelocity(joyY), scaleVelocity(joyX));
        double turn = Math.signum(joyTurn) * joyTurn * joyTurn * turnRate;
        SmartDashboard.putNumber("fieldX", fieldVel.getX());
        SmartDashboard.putNumber("fieldY", fieldVel.getY());
        SmartDashboard.putNumber("turn", turn);
        SmartDashboard.putNumber("xbox X", controller.getLeftX());
        SmartDashboard.putNumber("xbox Y", controller.getLeftY());
        SmartDashboard.putNumber("raw turn", controller.getRightX());
        SmartDashboard.putBoolean("isFieldOriented", fieldOrientation);
        drivebase.setVelocities(new ChassisSpeeds(robotVel.getX(), robotVel.getY(), turn));
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