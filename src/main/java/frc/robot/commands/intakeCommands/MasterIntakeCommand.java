package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class MasterIntakeCommand extends CommandBase {
    
    private IntakeSubsystem intake;

    public MasterIntakeCommand(IntakeSubsystem intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.raiseOrLowerIntake(Value.kForward);
        intake.spinIntake(1.0);
    }

    @Override
    public void execute() {
        if(intake.topHopperBall())
            intake.spinHopper(-0.5);
        else if(intake.intakeBall())
            intake.spinHopper(0.5);
        else
            intake.spinHopper(0.0);
    }

    @Override
    public void end(boolean interupted){
        if(interupted){
            intake.spinHopper(0.0);
            new SpinIntake(intake, -1.0).until(()->!intake.intakeBall())
                .andThen(()->intake.raiseOrLowerIntake(Value.kReverse), intake)
                .schedule();
        }
        else{
            intake.spinIntake(0.0);
            intake.raiseOrLowerIntake(Value.kReverse);
            new SpinHopper(intake, 1.0).until(intake::topHopperBall).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return intake.bottomHopperBall();
    }
}
