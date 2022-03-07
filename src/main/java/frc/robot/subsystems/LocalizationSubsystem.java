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

public class LocalizationSubsystem extends SubsystemBase {
  private KalmanFilter filter;
  private double step;
  private DMatrix2 mean = new DMatrix2();
  private DMatrix2x2 prec = new DMatrix2x2();
  private WPI_Pigeon2 imu = new WPI_Pigeon2(40);

  public LocalizationSubsystem(double step, int depth, double drift) {
    filter = new KalmanFilter(depth, new DMatrix2x2(drift, 0, 0, drift));
    this.step = step;
  }
  public LocalizationSubsystem(double step, int depth) {
    this(step, depth, 0.0);
  }
  public LocalizationSubsystem(double step, double drift) {
    this(step, 1, drift);
  }
  public LocalizationSubsystem(double step) {
    this(step, 1, 0.0);
  }

  public void addData(int degree, double x, double y, double varX, double varY, double covar) {
    mean.setTo(x, y);
    double idet = 1 / (varX * varY - covar * covar);
    prec.setTo(varY * idet, -covar * idet, -covar * idet, varX * idet);
    filter.update(degree, mean, prec);
  }
  public void addData(int degree, Translation2d pred, double varX, double varY, double covar) {
    addData(degree, pred.getX(), pred.getY(), varX, varY, covar);
  }
  public void addData(int degree, double x, double y, double var) {
    mean.setTo(x, y);
    prec.setTo(1/var, 0, 0, 1/var);
    filter.update(degree, mean, prec);
  }
  public void addData(int degree, Translation2d pred, double var) {
    addData(degree, pred.getX(), pred.getY(), var);
  }

  public Translation2d getPred(int degree) {
    filter.getPred(degree, mean);
    return new Translation2d(mean.a1, mean.a2);
  }
  public Translation2d getPred() {
    return getPred(0);
  }
  public Translation2d getPosition() {
    return getPred(0);
  }
  public Translation2d getVelocity() {
    return getPred(1);
  }

  public Rotation2d getOrientation() {
    return imu.getRotation2d();
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
