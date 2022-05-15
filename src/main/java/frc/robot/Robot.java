// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.GetDriveInformationCommand;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveToCommand;
import frc.robot.commands.FirstAutoCommand;
import frc.robot.commands.GetDriveInformationCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ManualShooterCommand;
import frc.robot.commands.RMPMotionGenerationCommand;
import frc.robot.commands.RMPMoveToPositionCommand;
import frc.robot.commands.RotationalDriveCommand;
import frc.robot.commands.ShootCommand2;
import frc.robot.commands.ShootDistance;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TranslationalDriveCommand;
import frc.robot.commands.intakeCommands.IntakeCargo;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.TranslationalDrivebase;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.ShooterSubsystem.CargoColor;

import static frc.robot.Constants.getSwerveDriveTalonDirectionalConfig;
import static frc.robot.Constants.getSwerveDriveTalonRotaryConfig;
import static frc.robot.Constants.Unit.*;

import com.titanrobotics2022.motion.generation.rmpflow.RMPRoot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Controllers
  private static final XboxController xbox = new XboxController(0);
  private static final Xinmotek xinmotek = new Xinmotek(2, 3);

  // Subsystems
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final DriveSubsystem drivebase = new SwerveDriveSubsystem(getSwerveDriveTalonDirectionalConfig(),
      getSwerveDriveTalonRotaryConfig());
  private final LocalizationSubsystem nav = new LocalizationSubsystem(0.02, 1.0);
  private final GetDriveInformationCommand odometry = new GetDriveInformationCommand(nav,
      drivebase.getTranslational());
  // private final Compressor compressor = new Compressor(41,
  // PneumaticsModuleType.CTREPCM);

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // TODO: Display autonomous chooser on dashboard
    nav.resetHeading();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("nav heading", nav.getHeading().getDegrees());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    shooter.disable();
    drivebase.coast();
    // climb.coast();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // TODO: Create autonomous
    nav.translateTo(new Translation2d(0, 0));
    nav.resetHeading();
    RMPRoot root = new RMPRoot("root");
    Translation2d goal = new Translation2d(0, 3);
    double v = 1;
    double tolerance = 0.1;
    double maxAcc = 2;
    new ParallelRaceGroup(new RMPMoveToPositionCommand(drivebase.getTranslational(), root, goal, nav, v,
        tolerance, maxAcc),
        new RMPMotionGenerationCommand(nav, root,
            drivebase.getTranslational()))
        .andThen(new WaitCommand(5))
        .schedule();
    odometry.schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    new TranslationalDriveCommand(drivebase.getTranslational(), xbox).schedule();
    new RotationalDriveCommand(drivebase.getRotational(), xbox).schedule();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

}