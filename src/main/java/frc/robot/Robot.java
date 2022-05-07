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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveToCommand;
import frc.robot.commands.FirstAutoCommand;
import frc.robot.commands.GetDriveInformationCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ManualShooterCommand;
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
  private static final Xinmotek xinmotek = new Xinmotek(2, 3);

  // Subsystems
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final SwerveDriveSubsystem drivebase =
    new SwerveDriveSubsystem(getSwerveDriveTalonDirectionalConfig(), getSwerveDriveTalonRotaryConfig());
  private final LocalizationSubsystem nav = new LocalizationSubsystem(0.02);
  private final ClimbSubsystem climb = new ClimbSubsystem();

  //private final Compressor compressor = new Compressor(41, PneumaticsModuleType.CTREPCM);

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
    SmartDashboard.putNumber("heading", nav.getHeading().getDegrees());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    shooter.disable();
    drivebase.coast();
    climb.coast();
  }

  @Override
  public void disabledPeriodic() {}

  private void enableRobot() {
    shooter.enable();
    climb.brake();
    drivebase.brake();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    enableRobot();
    shooter.robotColor = CargoColor.BLUE;
    //new FirstAutoCommand(drivebase.getTranslational(), shooter, intake, climb).schedule();
    //new DriveDistance(drivebase.getTranslational(), 10, 3, 2).schedule();
    /*new SequentialCommandGroup(
      new InstantCommand(() -> shooter.runPercent(0.45)),
      new WaitCommand(1.0),
      new InstantCommand(() -> shooter.runQueue(1.0)),
      new WaitCommand(3.0),
      new InstantCommand(() -> {shooter.runPercent(0.0); shooter.runQueue(0.0);}),
      new StartEndCommand(
        () -> drivebase.getTranslational().setVelocity(new Translation2d(0, 3)),
        () -> drivebase.getTranslational().setVelocity(new Translation2d(0, 0)),
        drivebase.getTranslational()).withTimeout(2)
    ).schedule();*/
    new SequentialCommandGroup(
      new InstantCommand(() -> {shooter.runTicks(7000); shooter.setAngle(16.618 * DEG);}),
      new WaitCommand(1.5),
      new StartEndCommand(() -> shooter.runQueue(1.0), () -> {shooter.coast(); shooter.runQueue(0.0);}).withTimeout(2.0),
      /*new StartEndCommand(
        () -> {drivebase.getTranslational().setVelocity(new Translation2d(0, 3)); intake.spinIntake(1.0); intake.spinHopper(1.0);},
        () -> {drivebase.getTranslational().setVelocity(new Translation2d(0, 0)); intake.spinIntake(0.0); intake.spinHopper(0.0);}
      ).withTimeout(1.25),
      new StartEndCommand(
        () -> {drivebase.getTranslational().setVelocity(new Translation2d(0, -3));},
        () -> {drivebase.getTranslational().setVelocity(new Translation2d(0, 0));}
      ).withTimeout(1.275),
      new InstantCommand(() -> {shooter.runTicks(7000); shooter.setAngle(11.235 * DEG);}),
      new WaitCommand(1.5),
      new StartEndCommand(() -> shooter.runQueue(1.0), () -> {shooter.coast(); shooter.runQueue(0.0);}).withTimeout(2.0),*/
      new StartEndCommand(
        () -> drivebase.getTranslational().setVelocity(new Translation2d(0, 3)),
        () -> drivebase.getTranslational().setVelocity(new Translation2d(0, 0))
      ).withTimeout(1.25)
    );//.schedule();
    nav.resetHeading();
    new DriveDistance(drivebase.getTranslational(), nav, new Translation2d(-0.25, 0), 0.5, 5, 0.02).schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // TODO: Makes sure the autonomous stops running when teleop starts
    enableRobot();

    drivebase.getTranslational().setDefaultCommand(new TranslationalDriveCommand(drivebase.getTranslational(), xbox, nav, 10.));
    drivebase.getRotational().setDefaultCommand(new RotationalDriveCommand(drivebase.getRotational(), xbox, 3 * Math.PI));
    new JoystickButton(xbox, Button.kA.value).whenPressed(() -> nav.resetHeading());
    new JoystickButton(xbox, Button.kX.value).whenActive(() -> drivebase.getTranslational().setVelocity(new Translation2d(0, 0.1)));
    new JoystickButton(xbox, Button.kY.value).whenActive(() -> drivebase.getTranslational().setVelocity(new Translation2d(0.1, 0)));

    /*new JoystickButton(xbox, Button.kLeftBumper.value).whenPressed(new StartEndCommand(
      () -> {intake.spinIntake(1.0); intake.spinHopper(1.0); intake.extend();},
      () -> {intake.spinIntake(0.0); intake.spinHopper(0.0); intake.retract();},
      intake));
    new Trigger(() -> xbox.getLeftTriggerAxis() > 0).whileActiveOnce(new InstantCommand(() -> intake.retract()));*/

    xinmotek.downButton.or(new JoystickButton(xbox, Button.kRightBumper.value)).whileActiveOnce(
      new StartEndCommand(
        () -> {intake.spinIntake(1.0); intake.spinHopper(1.0);},
        () -> {intake.spinIntake(0.0); intake.spinHopper(0.0);},
        intake)
    );
    //new JoystickButton(xbox, Button.kLeftBumper.value)
      //.whileActiveOnce(new ShootDistance(shooter, 7.3*FT, 2*IN));
    //xinmotek.upButton.whenHeld(new StartEndCommand(() -> {shooter.runTicks(7000); shooter.setAngle(29.1676*Math.PI/180.0);}, () -> shooter.coast(), shooter));
    //new JoystickButton(xbox, Button.kLeftBumper.value).whenHeld(new StartEndCommand(() -> {shooter.runTicks(6218); shooter.setAngle(29.1676*Math.PI/180.0);}, () -> shooter.coast(), shooter));
    xinmotek.upButton.whenHeld(new ShootCommand2(shooter, drivebase.getRotational()));
    
    xinmotek.leftPad.topLeft.whenHeld(new StartEndCommand(
      () -> {shooter.runTicks(7000); shooter.setAngle(0);},  // against hub
      () -> shooter.coast(), shooter));
      xinmotek.leftPad.bottomLeft.whenHeld(new StartEndCommand(
        () -> {shooter.runTicks(7000); shooter.setAngle(16.618 * DEG);},  // 76in (32in edge to bumper)
        () -> shooter.coast(), shooter));
      xinmotek.leftPad.topRight.whenHeld(new StartEndCommand(
        () -> {shooter.runTicks(7000); shooter.setAngle(21.417 * DEG);},  // 108in (64in edge to bumper)
        () -> shooter.coast(), shooter));
      xinmotek.leftPad.bottomRight.whenHeld(new StartEndCommand(
        () -> {shooter.runTicks(7000); shooter.setAngle(27.518 * DEG);},  // 94.5in edge to bumper
        () -> shooter.coast(), shooter));

    xinmotek.middlePad.topLeft.whenHeld(new StartEndCommand(
      () -> {/*intake.spinIntake(1.0);*/ intake.spinHopper(1.0);},
      () -> {/*intake.spinIntake(0.0);*/ intake.spinHopper(0.0);},
      intake));
    xinmotek.middlePad.bottomLeft.whenHeld(new StartEndCommand(
      () -> {/*intake.spinIntake(-1.0);*/ intake.spinHopper(-0.5);},
      () -> {/*intake.spinIntake(0.0);*/ intake.spinHopper(0.0);},
      intake));
    xinmotek.middlePad.topRight.whenHeld(new StartEndCommand(
      () -> shooter.runQueue(0.5),
      () -> shooter.runQueue(0.0)));
    xinmotek.middlePad.bottomRight.whenHeld(new StartEndCommand(
      () -> shooter.runQueue(-0.25),
      () -> shooter.runQueue(0.0)));
    
    /*xinmotek.rightPad.topLeft.whenHeld(new StartEndCommand(
      () -> {shooter.runTicks(12000); shooter.setAngle(37.882 * DEG);},  // 176in edge to bumper
      () -> shooter.coast(), shooter));
    xinmotek.rightPad.bottomLeft.whenHeld(new StartEndCommand(
      () -> {shooter.runTicks(12000); shooter.setAngle(43.812 * DEG);},  // 208in edge to bumper
      () -> shooter.coast(), shooter));*/
    xinmotek.rightPad.topLeft.whenHeld(new StartEndCommand(() -> climb.runClimb(0.6), () -> climb.runClimb(0.0), climb));
    xinmotek.rightPad.bottomLeft.whenHeld(new StartEndCommand(() -> climb.runClimb(-0.4), () -> climb.runClimb(0.0), climb));
    //xinmotek.rightPad.topRight.whenHeld(new StartEndCommand(() -> shooter.runPercent(0.4), () -> shooter.runPercent(0.0), shooter));
    //xinmotek.rightPad.bottomRight.whenHeld(new StartEndCommand(() -> shooter.runPercent(-0.2), () -> shooter.runPercent(0.0), shooter));
    //xinmotek.rightPad.topRight.whenHeld(new InstantCommand(() -> intake.extend()));
    //xinmotek.rightPad.bottomRight.whenHeld(new InstantCommand(() -> intake.retract()));
    xinmotek.rightPad.topRight.whenHeld(new StartEndCommand(() -> climb.runClimb(0.27), () -> climb.runClimb(0.0), climb));
    xinmotek.rightPad.bottomRight.whenHeld(new StartEndCommand(() -> climb.runClimb(-0.27), () -> climb.runClimb(0.0), climb));

    //Command flywheelOverride = new ManualShooterCommand(shooter, xinmotek);
    //Command rightClimbControl = new RunCommand(() -> climb.spinRight(0.65 * xinmotek.getRightY()));
    //new Trigger(() -> xinmotek.getRightX() > 0).whenActive(flywheelOverride);//.cancelWhenActive(rightClimbControl);
    //new Trigger(() -> xinmotek.getRightX() < 0).cancelWhenActive(flywheelOverride);//.whenActive(rightClimbControl);

    Command hoodOverride = new FunctionalCommand(
      () -> {shooter.hoodEnabled = false;},
      () -> shooter.spinHood(xinmotek.getLeftY()),
      (interrupted) -> {shooter.hoodEnabled = true;}, () -> false);
    //Command leftClimbControl = new RunCommand(() -> climb.spinLeft(0.65 * xinmotek.getLeftY()));
    new Trigger(() -> xinmotek.getLeftX() > 0).whenActive(hoodOverride);//.cancelWhenActive(leftClimbControl);
    new Trigger(() -> xinmotek.getLeftX() < 0).cancelWhenActive(hoodOverride);//.whenActive(leftClimbControl);
    new Trigger(() -> xinmotek.getRightY() > 0).whenActive(() -> drivebase.setCurLimit(drivebase.getCurLimit() + 5));
    new Trigger(() -> xinmotek.getRightY() < 0).whenActive(() -> drivebase.setCurLimit(drivebase.getCurLimit() - 5));
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //shooter.runQueue(xbox.getLeftTriggerAxis());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
