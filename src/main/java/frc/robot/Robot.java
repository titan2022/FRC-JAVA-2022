// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveDistance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RotationalDriveCommand;
import frc.robot.commands.ShootDistance3;
import frc.robot.commands.TranslationalDriveCommand;
import frc.robot.subsystems.ClimbSubsystem;
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
    new DriveDistance(drivebase.getTranslational(), nav, new Translation2d(0, 2), 0.75, 5, 0.02).schedule();
    new RunCommand(() -> {
      drivebase.getRotational().setRotation(-nav.getRate());
    }).schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // TODO: Makes sure the autonomous stops running when teleop starts
    enableRobot();

    // XBox Left Joy = translational drive
    drivebase.getTranslational().setDefaultCommand(new TranslationalDriveCommand(drivebase.getTranslational(), xbox, nav, 10.));
    // XBox Right Joy = rotational drive
    drivebase.getRotational().setDefaultCommand(new RotationalDriveCommand(drivebase.getRotational(), xbox, nav, 3 * Math.PI));
    // XBox A = reset field orientation to match current orientation
    new JoystickButton(xbox, Button.kA.value).whenPressed(() -> nav.resetHeading());
    // XBox B = toggle for intrinsic/extrinsic coordinates
    // XBox X = orient wheels along x axis
    new JoystickButton(xbox, Button.kX.value).whenActive(() -> drivebase.getTranslational().setVelocity(new Translation2d(0.1, 0)));
    // XBox Y = orient wheels along y axis
    new JoystickButton(xbox, Button.kY.value).whenActive(() -> drivebase.getTranslational().setVelocity(new Translation2d(0, 0.1)));
    // XBox Right Bumper or Xinmotek Down = full intake
    xinmotek.downButton.or(new JoystickButton(xbox, Button.kRightBumper.value)).whileActiveOnce(
      new StartEndCommand(
        () -> {intake.spinIntake(1.0); intake.spinHopper(1.0);},
        () -> {intake.spinIntake(0.0); intake.spinHopper(0.0);},
        intake)
    );

    // Xinmotek Up = aim
    xinmotek.upButton.whenHeld(new ShootDistance3(shooter, drivebase.getRotational()));
    
    // Xinmotek Left Panel = shoot from setpoints
    xinmotek.leftPad.topLeft.whenHeld(new ShootDistance3(shooter, drivebase.getRotational(), 5*FT));
    xinmotek.leftPad.bottomLeft.whenHeld(new ShootDistance3(shooter, drivebase.getRotational(), 8*FT));
    xinmotek.leftPad.topRight.whenHeld(new ShootDistance3(shooter, drivebase.getRotational(), 11*FT));
    xinmotek.leftPad.bottomRight.whenHeld(new ShootDistance3(shooter, drivebase.getRotational(), 14*FT));

    // Xinmotek Middle Panel = hopper and queue
    xinmotek.middlePad.topLeft.whenHeld(new StartEndCommand(
      () -> {intake.spinHopper(1.0);},
      () -> {intake.spinHopper(0.0);},
      intake));
    xinmotek.middlePad.bottomLeft.whenHeld(new StartEndCommand(
      () -> {intake.spinHopper(-0.5);},
      () -> {intake.spinHopper(0.0);},
      intake));
    xinmotek.middlePad.topRight.whenHeld(new StartEndCommand(
      () -> shooter.runQueue(0.5),
      () -> shooter.runQueue(0.0)));
    xinmotek.middlePad.bottomRight.whenHeld(new StartEndCommand(
      () -> shooter.runQueue(-0.25),
      () -> shooter.runQueue(0.0)));
    
    // Xinmotek Right Panel = climb
    xinmotek.rightPad.topLeft.whenHeld(new StartEndCommand(() -> climb.runClimb(0.6), () -> climb.runClimb(0.0), climb));
    xinmotek.rightPad.bottomLeft.whenHeld(new StartEndCommand(() -> climb.runClimb(0.5), () -> climb.runClimb(0.0), climb));
    xinmotek.rightPad.topRight.whenHeld(new StartEndCommand(() -> climb.runClimb(0.27), () -> climb.runClimb(0.0), climb));
    xinmotek.rightPad.bottomRight.whenHeld(new StartEndCommand(() -> climb.runClimb(-0.27), () -> climb.runClimb(0.0), climb));
    
    // Xinmotek Left Joy = Hood Override
    Command hoodOverride = new FunctionalCommand(
      () -> {shooter.hoodEnabled = false;},
      () -> shooter.spinHood(xinmotek.getLeftY()),
      (interrupted) -> {shooter.hoodEnabled = true;},
      () -> false);
    new Trigger(() -> xinmotek.getLeftX() > 0).whenActive(hoodOverride);
    new Trigger(() -> xinmotek.getLeftX() < 0).cancelWhenActive(hoodOverride);

    // Xinmotek Right Joy = Drive Current Limit
    new Trigger(() -> xinmotek.getRightY() > 0).whenActive(() -> drivebase.setCurLimit(drivebase.getCurLimit() + 5));
    new Trigger(() -> xinmotek.getRightY() < 0).whenActive(() -> drivebase.setCurLimit(drivebase.getCurLimit() - 5));

    SmartDashboard.putNumber("Hub Distance", 10);
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
