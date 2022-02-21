package frc.robot.config;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.HolonomicDriveCommand;
//import frc.robot.subsystems.NavigationSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.RobotContainer;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class SwerveDriveContainer extends RobotContainer {
    // Subsystems
    private final SwerveDriveSubsystem swerveDriveSub;
    //private final NavigationSubsystem navSub;
    // private final VHopperSubsystem vhopperSub;
    // private final IntakeSubsystem intakeSub;

    // Command Groups
    private final ParallelCommandGroup autoGroup;
    private final ParallelCommandGroup teleopGroup;

    private static final XboxController xbox = new XboxController(1);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public SwerveDriveContainer(boolean simulated) {
        // Initialize Subsystems
        swerveDriveSub = new SwerveDriveSubsystem(getSwerveDriveTalonDirectionalConfig(), getSwerveDriveTalonRotaryConfig());
        //navSub = new NavigationSubsystem();
        //vhopperSub = new VHopperSubsystem();
        //intakeSub = new IntakeSubsystem();

        // Initialize Auto Commands

        // Initialize Teleop Commands
        HolonomicDriveCommand manualDriveCommand = new HolonomicDriveCommand(swerveDriveSub, xbox);
        // ManualVHopperCommand vhopperCommand = new ManualVHopperCommand(vhopperSub);
        // ManualWristCommand intakeCommand = new ManualWristCommand(intakeSub);

        // Initialize Command Groups
        autoGroup = new ParallelCommandGroup(); // These don't actually run in parallel.
        teleopGroup = new ParallelCommandGroup(manualDriveCommand); //, vhopperCommand, intakeCommand);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */

    @Override
    public Command getAutonomousCommand() {
        return autoGroup;
    }

    @Override
    public Command getTeleopCommand() {
        return teleopGroup;
    }

    /**
     * Contains a velocity based PID configuration.
     * @return TalonFX Configuration Object
     */
    public TalonFXConfiguration getSwerveDriveTalonDirectionalConfig()
    {
        TalonFXConfiguration talon = new TalonFXConfiguration();
        // Add configs here:
        talon.slot0.kP = 1000;
        talon.slot0.kI = 0;  // 250
        talon.slot0.kD = 0;        
        talon.slot0.kF = 0;
        talon.slot0.integralZone = 900;
        talon.slot0.allowableClosedloopError = 217;
        talon.slot0.maxIntegralAccumulator = 254.000000;
        //talon.slot0.closedLoopPeakOutput = 0.869990; // Sets maximum output of the PID controller
        //talon.slot0.closedLoopPeriod = 33; // Sets the hardware update rate of the PID controller
        
        return talon;
    }

    /**
     * Contains a position based PID configuration
     * @return TalonFX Configuration Object
     */
    public TalonFXConfiguration getSwerveDriveTalonRotaryConfig()
    {
        TalonFXConfiguration talon = new TalonFXConfiguration();
        // Add configs here:
        talon.slot0.kP = 10.;
        talon.slot0.kI = 0;
        talon.slot0.kD = 0;
        talon.slot0.kF = 0;
        talon.slot0.integralZone = 900;
        talon.slot0.allowableClosedloopError = 20;//217;
        talon.slot0.maxIntegralAccumulator = 254.000000;
        //talon.slot0.closedLoopPeakOutput = 0.869990; // Sets maximum output of the PID controller
        //talon.slot0.closedLoopPeriod = 33; // Sets the hardware update rate of the PID controller

        return talon;
    }
}
