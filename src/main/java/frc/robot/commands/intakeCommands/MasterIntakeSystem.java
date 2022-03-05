package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.IntakeSubsystem;

public class MasterIntakeSystem extends ParallelRaceGroup {
    
    private IntakeSubsystem sub = new IntakeSubsystem();

    public MasterIntakeSystem(){
        addCommands(
            new SpinIntake(sub, 5*Math.PI).until(()->sub.bottomHopperBall()),
            new SpinHopper(sub, 5*Math.PI)
        );
    }

    public void end(boolean interupted){
        if(interupted){
            new SpinIntake(sub, -5*Math.PI).until(()->sub.intakeBall());
            sub.raiseOrLowerIntake(Value.kForward);
        }
    }





}
