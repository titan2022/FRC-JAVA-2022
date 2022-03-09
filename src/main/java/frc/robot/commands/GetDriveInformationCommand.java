package frc.robot.commands;

import edu.wpi.first.cscore.CameraServerJNI.TelemetryKind;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.TranslationalDrivebase;

public class GetDriveInformationCommand extends CommandBase {
    
    
    LocalizationSubsystem localPosition;
    TranslationalDrivebase translator; //object translator is currently an interface, need to create a class for it
    public GetDriveInformationCommand(LocalizationSubsystem loc, TranslationalDrivebase trans){
        localPosition = loc;
        translator = trans;
    }

    @Override
    public void initialize() {

    }

    public void execute() {
        localPosition.addData(1, new Translation2d(localPosition.getOrientation().getCos() * translator.getVelocity().getNorm(), localPosition.getOrientation().getSin() * translator.getVelocity().getNorm()), 0.01);
    }

    @Override
    public void end(boolean interrupted) {}

    public boolean isFinished() {
        return false;
    }
}
