package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.ApplicationSubsystem;

public class ApplicationCommand extends CommandBase {
    private XboxController controller;
    private final ApplicationSubsystem aSubsystem;

    public ApplicationCommand(ApplicationSubsystem aSubsystem, XboxController controller) {
        this.controller = controller;
        this.aSubsystem = aSubsystem;
        addRequirements(aSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute(){
        if(controller.getAButton()){
            aSubsystem.spin(0.9);
        }
    }
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        aSubsystem.spin(0);
    }

}
