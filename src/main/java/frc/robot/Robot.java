// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveToCommand;
import frc.robot.commands.FirstAutoCommand;
import frc.robot.commands.GetDriveInformationCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ManualShooterCommand;
import frc.robot.commands.RotationalDriveCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TranslationalDriveCommand;
import frc.robot.commands.intakeCommands.IntakeCargo;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem.CargoColor;

import static frc.robot.Constants.getSwerveDriveTalonDirectionalConfig;
import static frc.robot.Constants.getSwerveDriveTalonRotaryConfig;
import static frc.robot.Constants.Unit.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Controllers
  private static final XboxController xbox = new XboxController(0);
  private static final Xinmotek xinmotek = new Xinmotek(1, 2);

  // Subsystems
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final DriveSubsystem drivebase =
    new SwerveDriveSubsystem(getSwerveDriveTalonDirectionalConfig(), getSwerveDriveTalonRotaryConfig());
  private final LocalizationSubsystem nav = new LocalizationSubsystem(0.02);
  private final ClimbSubsystem climb = new ClimbSubsystem();

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
  public void disabledInit() {
    shooter.disable();
    climb.coast();
  }

  @Override
  public void disabledPeriodic() {}

  private void enableRobot() {
    shooter.enable();
    climb.brake();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    enableRobot();
    shooter.robotColor = CargoColor.BLUE;
    new FirstAutoCommand(drivebase.getTranslational(), shooter, intake, climb).schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // TODO: Makes sure the autonomous stops running when teleop starts
    enableRobot();

    drivebase.getTranslational().setDefaultCommand(new TranslationalDriveCommand(drivebase.getTranslational(), xbox, nav, 5.));
    drivebase.getRotational().setDefaultCommand(new RotationalDriveCommand(drivebase.getRotational(), xbox, 4 * Math.PI));
    new JoystickButton(xbox, Button.kA.value).whenPressed(() -> nav.resetHeading());

    xinmotek.downButton.or(new JoystickButton(xbox, Button.kRightBumper.value)).whileActiveOnce(new IntakeCargo(intake, shooter));
    xinmotek.upButton.whenHeld(
      new ShooterCommand(shooter, drivebase.getRotational(), nav, 0.0, 2.0, 0.0, 0.0, 1.0, 5 * IN, 0.02));
    
    xinmotek.leftPad.topLeft.and(xinmotek.leftPad.bottomLeft).whenActive(() -> {
      if(shooter.colorEnabled)
        shooter.robotColor = CargoColor.NONE;
      else
        shooter.colorOverride = CargoColor.NONE;
    });
    xinmotek.leftPad.topLeft.and(xinmotek.leftPad.bottomLeft.negate()).whenActive(() -> {
      if(shooter.colorEnabled)
        shooter.robotColor = CargoColor.RED;
      else
        shooter.colorOverride = CargoColor.RED;
    });
    xinmotek.leftPad.bottomLeft.and(xinmotek.leftPad.topLeft.negate()).whenActive(() -> {
      if(shooter.colorEnabled)
        shooter.robotColor = CargoColor.BLUE;
      else
        shooter.colorOverride = CargoColor.BLUE;
    });
    xinmotek.leftPad.topRight.whenPressed(() -> {shooter.colorEnabled = false;});
    xinmotek.leftPad.bottomRight.whenPressed(() -> {shooter.colorEnabled = true;});

    xinmotek.middlePad.topLeft.whenHeld(new StartEndCommand(
      () -> {intake.spinIntake(1.0); intake.spinHopper(1.0);},
      () -> {intake.spinIntake(0.0); intake.spinHopper(0.0);},
      intake));
    xinmotek.middlePad.bottomLeft.whenHeld(new StartEndCommand(
      () -> {intake.spinIntake(-1.0); intake.spinHopper(-1.0);},
      () -> {intake.spinIntake(0.0); intake.spinHopper(0.0);},
      intake));
    xinmotek.middlePad.topRight.whenHeld(new StartEndCommand(
      () -> shooter.runQueue(0.5),
      () -> shooter.runQueue(0.0)));
    xinmotek.middlePad.bottomRight.whenHeld(new StartEndCommand(
      () -> shooter.runQueue(-0.5),
      () -> shooter.runQueue(0.0)));
    
    xinmotek.rightPad.topLeft.whenPressed(() -> {shooter.queueEnabled = false;});
    xinmotek.rightPad.bottomLeft.whenPressed(() -> {shooter.queueEnabled = true;});
    xinmotek.rightPad.topRight.whenHeld(new StartEndCommand(() -> climb.runClimb(0.65), () -> climb.runClimb(0.0), climb));
    xinmotek.rightPad.bottomRight.whenHeld(new StartEndCommand(() -> climb.runClimb(-0.65), () -> climb.runClimb(0.0), climb));

    Command flywheelOverride = new ManualShooterCommand(shooter, xinmotek);
    Command rightClimbControl = new RunCommand(() -> climb.spinRight(0.65 * xinmotek.getRightY()));
    new Trigger(() -> xinmotek.getRightX() > 0).whenActive(flywheelOverride).cancelWhenActive(rightClimbControl);
    new Trigger(() -> xinmotek.getRightX() < 0).cancelWhenActive(flywheelOverride).whenActive(rightClimbControl);

    Command hoodOverride = new FunctionalCommand(
      () -> {shooter.hoodEnabled = false;},
      () -> shooter.spinHood(xinmotek.getLeftY()),
      (interrupted) -> {shooter.hoodEnabled = true;}, () -> false);
    Command leftClimbControl = new RunCommand(() -> climb.spinLeft(0.65 * xinmotek.getLeftY()));
    new Trigger(() -> xinmotek.getLeftX() > 0).whenActive(hoodOverride).cancelWhenActive(leftClimbControl);
    new Trigger(() -> xinmotek.getLeftX() < 0).cancelWhenActive(hoodOverride).whenActive(leftClimbControl);
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
