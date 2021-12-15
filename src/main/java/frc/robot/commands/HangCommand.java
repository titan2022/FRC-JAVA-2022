package frc.robot.commands;

import frc.robot.subsystems.HangSubsystem;
import frc.robot.Constants;

public class HangCommand {
    
    final double REEL_SPEED = 10;

    HangSubsystem sub;
    public HangCommand(HangSubsystem sub){
        this.sub = sub;
    }
    public void execute(){
        if(Constants.controller.getAButton()){
            sub.reel(REEL_SPEED);
        }
        else{
            sub.reel(0.0);
        }
    }

}
