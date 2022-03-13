package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.TranslationalDrivebase;

public class TranslationalDriveCommand extends CommandBase {
    private TranslationalDrivebase drive;
    private XboxController controller;
    private LocalizationSubsystem nav;
    private double maxVel;
    private boolean isFieldOriented = true;

    /**
     * Creates a new TranlationalDriveCommand with optional field orientation.
     * 
     * @param drivebase  The drivebase to control.
     * @param xbox  The joystick controller to use.
     * @param localizer  The localization subsystem to use for orientation
     *  correction. This will not be added as a requirement of this command.
     * @param velocity  The translational velocity represented by moving the
     *  translational joystick all the way towards a cardinal direction, in
     *  meters per second.
     */
    public TranslationalDriveCommand(TranslationalDrivebase drivebase, XboxController xbox, LocalizationSubsystem localizer, double velocity) {
        drive = drivebase;
        controller = xbox;
        nav = localizer;
        maxVel = velocity;
        addRequirements(drivebase);
    }
    /**
     * Creates a new TranlationalDriveCommand using intrinsic orientation.
     * 
     * @param drivebase  The drivebase to control.
     * @param xbox  The joystick controller to use.
     * @param velocity  The translational velocity represented by moving the
     *  translational joystick all the way towards a cardinal direction, in
     *  meters per second.
     */
    public TranslationalDriveCommand(TranslationalDrivebase drivebase, XboxController xbox, double velocity) {
        this(drivebase, xbox, null, velocity);
    }
    /**
     * Creates a new TranlationalDriveCommand with optional field orientation.
     * 
     * The maximum velocity in a cardinal direction defaults to 5 meters per second.
     * 
     * @param drivebase  The drivebase to control.
     * @param xbox  The joystick controller to use.
     * @param localizer  The localization subsystem to use for orientation
     *  correction. This will not be added as a requirement of this command.
     */
    public TranslationalDriveCommand(TranslationalDrivebase drivebase, XboxController xbox, LocalizationSubsystem localizer) {
        this(drivebase, xbox, localizer, 5.);
    }
    /**
     * Creates a new TranlationalDriveCommand using intrinsic orientation.
     * 
     * The maximum velocity in a cardinal direction defaults to 5 meters per second.
     * 
     * @param drivebase  The drivebase to control.
     * @param xbox  The joystick controller to use.
     */
    public TranslationalDriveCommand(TranslationalDrivebase drivebase, XboxController xbox) {
        this(drivebase, xbox, null, 5.);
    }

    @Override
    public void initialize() {}

    private static double applyDeadband(double joy, double deadband) {
        return Math.abs(joy) < deadband ? 0 : joy;
    }
    
    private double scaleVelocity(double joy) {
        return Math.signum(joy) * joy * joy * maxVel;
    }

    @Override
    public void execute() {
        if(controller.getBButtonPressed())
            isFieldOriented = !isFieldOriented;
        SmartDashboard.putBoolean("isFieldOriented", isFieldOriented);
        double joyX = applyDeadband(controller.getLeftX(), 0.1);
        double joyY = applyDeadband(-controller.getLeftY(), 0.1);
        SmartDashboard.putNumber("joyX", joyX);
        SmartDashboard.putNumber("joyY", joyY);
        Translation2d velocity = new Translation2d(scaleVelocity(joyX), scaleVelocity(joyY));
        SmartDashboard.putNumber("fieldX", velocity.getX());
        SmartDashboard.putNumber("fieldY", velocity.getY());
        if(isFieldOriented){
            Rotation2d heading = nav.getHeading();
            SmartDashboard.putNumber("heading", heading.getDegrees());
            velocity = velocity.rotateBy(heading);
        }
        SmartDashboard.putNumber("robotX", velocity.getX());
        SmartDashboard.putNumber("robotY", velocity.getY());
        drive.setVelocity(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        drive.setVelocity(new Translation2d(0, 0));
        if(interrupted)
            new StartEndCommand(() -> {
                    controller.setRumble(RumbleType.kLeftRumble, 0.5);
                }, () -> {
                    controller.setRumble(RumbleType.kLeftRumble, 0);
                }).withTimeout(0.5).schedule();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
