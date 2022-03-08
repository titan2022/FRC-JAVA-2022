// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.titanrobotics2022.localization.KalmanFilter;

import org.ejml.data.DMatrix2;
import org.ejml.data.DMatrix2x2;

/**
 * A localizer based around a Kalman filter.
 * 
 * This subsystem should not be declared as a requirement for commands.
 */
public class LocalizationSubsystem extends SubsystemBase {
  private KalmanFilter filter;
  private double step;
  private DMatrix2 mean = new DMatrix2();
  private DMatrix2x2 prec = new DMatrix2x2();
  private WPI_Pigeon2 imu = new WPI_Pigeon2(40);
  private Rotation2d phiOffset = new Rotation2d(Math.PI / 4);

  /**
   * Creates a new LocalizationSubsystem.
   * 
   * @param step  The time difference between calls to the periodic method of
   *  this subsystem.
   * @param depth  The maximum derivative of position to consider. For instance,
   *  1 for velocity, or 2 for acceleration.
   * @param drift  The intrinsic covariance due to noise of the highest order
   *  derivative of position considered.
   */
  public LocalizationSubsystem(double step, int depth, double drift) {
    filter = new KalmanFilter(depth, new DMatrix2x2(drift, 0, 0, drift));
    this.step = step;
  }
  /**
   * Creates a new LocalizationSubsystem.
   * 
   * No intrinsic drift is assumed to exist in the system.
   * 
   * @param step  The time difference between calls to the periodic method of
   *  this subsystem.
   * @param depth  The maximum derivative of position to consider. For instance,
   *  1 for velocity, or 2 for acceleration.
   */
  public LocalizationSubsystem(double step, int depth) {
    this(step, depth, 0.0);
  }
  /**
   * Creates a new LocalizationSubsystem.
   * 
   * Velocity is the highest order derivative of position considered.
   * 
   * @param step  The time difference between calls to the periodic method of
   *  this subsystem.
   * @param drift  The intrinsic covariance due to noise of the highest order
   *  derivative of position considered.
   */
  public LocalizationSubsystem(double step, double drift) {
    this(step, 1, drift);
  }
  /**
   * Creates a new LocalizationSubsystem.
   * 
   * Velocity is the highest order derivative of position considered and no
   * intrinsic drift is assumed to exist in the system.
   * 
   * @param step  The time difference between calls to the periodic method of
   *  this subsystem.
   */
  public LocalizationSubsystem(double step) {
    this(step, 1, 0.0);
  }

  /**
   * Updates the state of the localizer with a new measurement.
   * 
   * @param degree  The order of the derivative of position to update.
   * @param x  The x component of the measurement.
   * @param y  The y component of the measurement.
   * @param varX  The variance in the x component of the measurement.
   * @param varY  The variance in the y component of the measurement.
   * @param covar  The covariance of the x and y components of the measurement.
   */
  public void addData(int degree, double x, double y, double varX, double varY, double covar) {
    mean.setTo(x, y);
    double idet = 1 / (varX * varY - covar * covar);
    prec.setTo(varY * idet, -covar * idet, -covar * idet, varX * idet);
    filter.update(degree, mean, prec);
  }
  /**
   * Updates the state of the localizer with a new measurement.
   * 
   * @param degree  The order of the derivative of position to update.
   * @param pred  The measurement.
   * @param varX  The variance in the x component of the measurement.
   * @param varY  The variance in the y component of the measurement.
   * @param covar  The covariance of the x and y components of the measurement.
   */
  public void addData(int degree, Translation2d pred, double varX, double varY, double covar) {
    addData(degree, pred.getX(), pred.getY(), varX, varY, covar);
  }
  /**
   * Updates the state of the localizer with a new measurement.
   * 
   * @param degree  The order of the derivative of position to update.
   * @param x  The x component of the measurement.
   * @param y  The y component of the measurement.
   * @param var  The variance of the noise in the measurement, which is assumed
   *  to be isotropic.
   */
  public void addData(int degree, double x, double y, double var) {
    mean.setTo(x, y);
    prec.setTo(1/var, 0, 0, 1/var);
    filter.update(degree, mean, prec);
  }
  /**
   * Updates the state of the localizer with a new measurement.
   * 
   * @param degree  The order of the derivative of position to update.
   * @param pred  The measurement.
   * @param var  The variance of the noise in the measurement, which is assumed
   *  to be isotropic.
   */
  public void addData(int degree, Translation2d pred, double var) {
    addData(degree, pred.getX(), pred.getY(), var);
  }

  public void resetHeading() {
    phiOffset = phiOffset.plus(getOrientation());
  }

  /**
   * Returns the current estimate of a given derivative of position.
   * 
   * @param degree  The order of the derivative of position to return the
   *  estimate of.
   * @return  The current estimate of the requested derivative of position.
   */
  public Translation2d getPred(int degree) {
    filter.getPred(degree, mean);
    return new Translation2d(mean.a1, mean.a2);
  }
  /**
   * Returns the current estimate of the position of the robot.
   * 
   * @return  The current estimate of the position of the robot in meters.
   */
  public Translation2d getPosition() {
    return getPred(0);
  }
  /**
   * Returns the current estimate of the velocity of the robot.
   * 
   * @return  The current estimate of the velocity of the robot in meters per
   *  second.
   */
  public Translation2d getVelocity() {
    return getPred(1);
  }

  /**
   * Returns the current estimate of the orientation of the robot.
   * 
   * @return  The current estimate of the orientation of the robot in radians
   *  counterclockwise from the positive x axis.
   */
  public Rotation2d getOrientation() {
    return imu.getRotation2d().minus(phiOffset);
  }

  @Override
  public void periodic() {
    filter.step(step);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
