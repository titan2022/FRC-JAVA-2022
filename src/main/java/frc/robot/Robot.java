// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ManualShooterCommand;
import frc.robot.commands.intakeCommands.IntakeCargo;
import frc.robot.commands.intakeCommands.SpinHopper;
import frc.robot.commands.intakeCommands.SpinIntake;
import frc.robot.commands.RotationalDriveCommand;
import frc.robot.commands.TranslationalDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

import static frc.robot.Constants.getSwerveDriveTalonDirectionalConfig;
import static frc.robot.Constants.getSwerveDriveTalonRotaryConfig;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Controllers
  private static final XboxController xbox = new XboxController(0);

  // Subsystems
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final DriveSubsystem drivebase =
    new SwerveDriveSubsystem(getSwerveDriveTalonDirectionalConfig(), getSwerveDriveTalonRotaryConfig());
  private final LocalizationSubsystem nav = new LocalizationSubsystem(0.02);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // TODO: Display autonomous chooser on dashboard
    nav.resetHeading();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // TODO: Create autonomous
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // TODO: Makes sure the autonomous stops running when teleop starts
    new JoystickButton(xbox, Button.kLeftBumper.value)
      .whenHeld(new IntakeCargo(intake, shooter));
    shooter.setDefaultCommand(new ManualShooterCommand(shooter));
    drivebase.getTranlational().setDefaultCommand(new TranslationalDriveCommand(drivebase.getTranlational(), xbox, nav, 5.));
    drivebase.getRotational().setDefaultCommand(new RotationalDriveCommand(drivebase.getRotational(), xbox, 4 * Math.PI));
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
