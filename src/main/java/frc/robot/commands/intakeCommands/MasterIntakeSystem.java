package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.IntakeSubsystem;

public class MasterIntakeSystem extends ParallelRaceGroup {
    
    private IntakeSubsystem sub = new IntakeSubsystem();

    public MasterIntakeSystem(){
        addCommands(
            new SpinIntake(sub, 1.0).until(()->sub.bottomHopperBall()),
            new SpinHopper(sub, 0.25)
        );
    }

    public void end(boolean interupted){
        if(interupted){
            new SpinIntake(sub, -1.0).until(()->sub.intakeBall());
            sub.raiseOrLowerIntake(Value.kForward);
        }
    }





}
