// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.Matrix;
import org.ejml.simple.SimpleMatrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

/**
 * A localizer based around a Kalman filter.
 * 
 * This subsystem should not be declared as a requirement for commands.
 */
public class LocalizationSubsystem extends SubsystemBase {
  private SwerveDrivePoseEstimator estimator;
  private double step;
  private WPI_Pigeon2 imu = new WPI_Pigeon2(40);
  private Rotation2d phiOffset = new Rotation2d(Math.PI / 4);
  private SwerveDriveSubsystem drivebase;
  private Translation2d pos = new Translation2d(0, 0);
  private Translation2d vel = new Translation2d(0, 0);

  /**
   * Creates a new LocalizationSubsystem.
   * 
   * @param step  The time difference between calls to the periodic method of
   *              this subsystem.
   * @param depth The maximum derivative of position to consider. For instance,
   *              1 for velocity, or 2 for acceleration.
   * @param drift The intrinsic covariance due to noise of the highest order
   *              derivative of position considered.
   */
  public LocalizationSubsystem(double step, int depth, double drift, SwerveDriveSubsystem drivebase) {
    this.drivebase = drivebase;
    this.step = step;
    SimpleMatrix stateStdDevs = new SimpleMatrix(3, 1, false, new double[] { 0.2, 0.2, 0.1 }); // TODO: tune values
    SimpleMatrix localMeasurementStdDevs = new SimpleMatrix(1, 1, false, new double[] { 0.005 }); // TODO: tune values
    SimpleMatrix visionMeasurementStdDevs = new SimpleMatrix(3, 1, false, new double[] { 0.02, 0.02, 0.01 }); // TODO:
                                                                                                              // tune
                                                                                                              // values
    estimator = new SwerveDrivePoseEstimator(getOrientation(), new Pose2d(new Translation2d(0, 0), getOrientation()),
        drivebase.getKinematics(), new Matrix<N3, N1>(stateStdDevs), new Matrix<N1, N1>(localMeasurementStdDevs),
        new Matrix<N3, N1>(visionMeasurementStdDevs), step);
  }

  /**
   * Creates a new LocalizationSubsystem.
   * 
   * No intrinsic drift is assumed to exist in the system.
   * 
   * @param step  The time difference between calls to the periodic method of
   *              this subsystem.
   * @param depth The maximum derivative of position to consider. For instance,
   *              1 for velocity, or 2 for acceleration.
   */
  public LocalizationSubsystem(double step, int depth, SwerveDriveSubsystem drivebase) {
    this(step, depth, 0.0, drivebase);
  }

  /**
   * Creates a new LocalizationSubsystem.
   * 
   * Velocity is the highest order derivative of position considered.
   * 
   * @param step  The time difference between calls to the periodic method of
   *              this subsystem.
   * @param drift The intrinsic covariance due to noise of the highest order
   *              derivative of position considered.
   */
  public LocalizationSubsystem(double step, double drift, SwerveDriveSubsystem drivebase) {
    this(step, 1, drift, drivebase);
  }

  /**
   * Creates a new LocalizationSubsystem.
   * 
   * Velocity is the highest order derivative of position considered and no
   * intrinsic drift is assumed to exist in the system.
   * 
   * @param step The time difference between calls to the periodic method of
   *             this subsystem.
   */
  public LocalizationSubsystem(double step, SwerveDriveSubsystem drivebase) {
    this(step, 1, 0.0, drivebase);
  }

  /**
   * Sets the current orientation to a specified value.
   * 
   * @param phi The new current orientation, measured counterclockwise from the
   *            positive x axis.
   */
  public void setOrientation(Rotation2d phi) {
    phiOffset = phiOffset.plus(getOrientation().minus(phi));
  }

  /**
   * Sets the current heading to a specified value.
   * 
   * @param heading The new current heading, measured clockwise from the
   *                positive y axis.
   */
  public void setHeading(Rotation2d heading) {
    setOrientation(new Rotation2d(Math.PI / 2).minus(heading));
  }

  /** Resets the current orientation to zero. */
  public void resetOrientation() {
    setOrientation(new Rotation2d(0));
  }

  /** Resets the current heading to zero. */
  public void resetHeading() {
    setHeading(new Rotation2d(0));
  }

  /**
   * Returns the current estimate of the position of the robot.
   * 
   * @return The current estimate of the position of the robot in meters.
   */
  public Translation2d getPosition() {
    return pos;
  }

  /**
   * Returns the current estimate of the velocity of the robot.
   * 
   * @return The current estimate of the velocity of the robot in meters per
   *         second.
   */
  public Translation2d getVelocity() {
    return vel;
  }

  /**
   * Returns the current estimate of the orientation of the robot.
   * 
   * @return The current estimate of the orientation of the robot measured
   *         counterclockwise from the positive x axis.
   */
  public Rotation2d getOrientation() {
    return imu.getRotation2d().minus(phiOffset);
  }

  /**
   * Returns the current estimate of the heading of the robot.
   * 
   * @return The current estimate of the heading of the robot, measured
   *         clockwise from the positive y axis.
   */
  public Rotation2d getHeading() {
    return new Rotation2d(Math.PI / 2).minus(getOrientation());
  }

  @Override
  public void periodic() {
    Translation2d lastPos = pos;
    pos = estimator.update(getOrientation(), drivebase.getSwerveModuleStates()).getTranslation();
    vel = pos.minus(lastPos).div(step);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
