package frc.robot.commands;

import edu.wpi.first.cscore.CameraServerJNI.TelemetryKind;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.TranslationalDrivebase;

public class GetDriveInformationCommand extends CommandBase {
    private LocalizationSubsystem nav;
    private TranslationalDrivebase translator;
    private double var;

    public GetDriveInformationCommand(LocalizationSubsystem loc, TranslationalDrivebase trans, double std) {
        nav = loc;
        translator = trans;
        var = std * std;
    }

    public GetDriveInformationCommand(LocalizationSubsystem loc, TranslationalDrivebase trans) {
        this(loc, trans, 0.1);
    }

    @Override
    public void initialize() {
    }

    public void execute() {
        SmartDashboard.putNumber("odometry vel x",
                translator.getVelocity().rotateBy(nav.getHeading()).getX());
        SmartDashboard.putNumber("odometry vel y",
                translator.getVelocity().rotateBy(nav.getHeading()).getY());
        nav.addData(1, translator.getVelocity().rotateBy(nav.getHeading()), var);
    }

    @Override
    public void end(boolean interrupted) {
    }

    public boolean isFinished() {
        return false;
    }
}
