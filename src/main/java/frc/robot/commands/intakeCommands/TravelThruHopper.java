package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.IntakeSubsystem;

public class TravelThruHopper extends ParallelRaceGroup {
    
    private IntakeSubsystem sub = new IntakeSubsystem();

    public TravelThruHopper(){
        addCommands(
            new SpinHopper(sub, 1.0).until(()->sub.topHopperBall())
        );
    }

    public void end(boolean interupted){
        if(interupted){
        }
    }





}
