// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.FinishShootCommand;
import frc.robot.commands.ShooterCommand;

import static frc.robot.Constants.XBOX_CONTROLLER;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final IntakeSubsystem intake = new IntakeSubsystem();
  ShooterSubsystem shooter = new ShooterSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // OOP combined with imperitive is so scuffed!
    new JoystickButton(XBOX_CONTROLLER, Button.kBumperLeft.value)
      .whenPressed(() -> intake.spinIntake(1))
      .whenReleased(() -> intake.spinIntake(0));
    new JoystickButton(XBOX_CONTROLLER, Button.kBumperRight.value)
      .whenPressed(() -> intake.spinIntake(-1))
      .whenReleased(() -> intake.spinIntake(0));

    new JoystickButton(XBOX_CONTROLLER, Button.kA.value)
      .whenActive(new ShooterCommand(shooter));

    // Testing purposes
    // new JoystickButton(XBOX_CONTROLLER, Button.kB.value)
    //   .whenActive(new FinishShootCommand(shooter));

      // TODO: new SequentalCommandGroup with AimCommand & FinishShoot command
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
