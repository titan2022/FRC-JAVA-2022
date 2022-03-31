package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends CommandBase {
    private ClimbSubsystem climb;

    /**
     * Creates climb command for the autoclimb
     * 
     * @param climb The ClimbSubsystem
     * @param gearTicks T
     */
    public ClimbCommand(ClimbSubsystem climb, double gearTicks) {

    }
}   
