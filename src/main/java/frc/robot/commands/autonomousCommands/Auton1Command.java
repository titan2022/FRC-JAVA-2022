package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OdometrySubsystemWrapper;
import frc.robot.subsystems.ShooterSubsystem;


public class Auton1Command extends CommandBase{
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final DriveSubsystem drivebase;
    private final OdometrySubsystemWrapper odemetry;
    
}
