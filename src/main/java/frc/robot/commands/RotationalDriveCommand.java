package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.RotationalDrivebase;

public class RotationalDriveCommand extends CommandBase {
    private RotationalDrivebase drive;
    private XboxController controller;
    private double maxRate;

    /**
     * Controls the rotational velocity of the drivebase with a joystick. 
     * 
     * @param drivebase  The drivebase to control.
     * @param xbox  The joystick controller to use.
     * @param turnRate  The maximum rotational velocity in radians per second.
     */
    public RotationalDriveCommand(RotationalDrivebase drivebase, XboxController xbox, double turnRate) {
        drive = drivebase;
        controller = xbox;
        maxRate = turnRate;
        addRequirements(drivebase);
    }
    /**
     * Controls the rotational velocity of the drivebase with a joystick.
     * 
     * The maximum rotational velocity defaults to two revolutions per second.
     * 
     * @param drivebase  The drivebase to control.
     * @param xbox  The joystick controller to use.
     */
    public RotationalDriveCommand(RotationalDrivebase drivebase, XboxController xbox) {
        this(drivebase, xbox, 4 * Math.PI);
    }

    @Override
    public void initialize() {}

    private static double applyDeadband(double joy, double deadband) {
        return Math.abs(joy) < deadband ? 0 : joy;
    }
    
    private double scaleVelocity(double joy) {
        return Math.signum(joy) * joy * joy * maxRate;
    }

    @Override
    public void execute() {
        double joy = applyDeadband(controller.getRightX(), 0.1);
        SmartDashboard.putNumber("joyOmega", joy);
        double omega = scaleVelocity(joy);
        SmartDashboard.putNumber("omega", omega);
        drive.setRotation(omega);
    }

    @Override
    public void end(boolean interrupted) {
        drive.setRotation(0);
        if(interrupted)
            new StartEndCommand(() -> {
                    controller.setRumble(RumbleType.kRightRumble, 0.5);
                }, () -> {
                    controller.setRumble(RumbleType.kRightRumble, 0);
                }).withTimeout(0.5).schedule();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}