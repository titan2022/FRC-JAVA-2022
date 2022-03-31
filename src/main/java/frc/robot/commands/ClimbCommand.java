package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends CommandBase {
    private ClimbSubsystem climb;

    //The gear ratio is for the lift 190.5:1 and the max RPM of the Falcon 500 is 6380 sot the max rpm of the lift per second is 0.54516 rps or 196.2576 degrees per second
    //The lift is around 31 degrees
    //121 degrees
    public ClimbCommand(ClimbSubsystem climb, double gearTicks) {
        
    }
}   
