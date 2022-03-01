package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.IntakeSubsystem;

public class MasterIntakeSystem extends ParallelRaceGroup {
    
    private IntakeSubsystem sub = new IntakeSubsystem();

    public MasterIntakeSystem(){
        addCommands(
            new SpinIntake(sub, 5*Math.PI).until(()->sub.hopperBall()),
            new SpinHopper(sub, 5*Math.PI)
        );
    }

    
}
