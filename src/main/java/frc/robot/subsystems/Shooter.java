/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final CANSparkMax motorOne = new CANSparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_ONE, MotorType.kBrushless);
  private final CANSparkMax motorTwo = new CANSparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_TWO, MotorType.kBrushless);
  private final CANPIDController sparkPIDControllerOne = motorOne.getPIDController();
  private final CANPIDController sparkPIDControllerTwo = motorTwo.getPIDController();

  private final CANEncoder sparkEncoderOne = motorOne.getEncoder();
  private final CANEncoder sparkEncoderTwo = motorTwo.getEncoder();

  /**
   * Creates the Shooter subsystem
   */
  public Shooter() {
    configureSpark();
    // SmartDashboard.putNumber("desired velocity", 0.0);
    // SmartDashboard.putNumber("d value", .6);
  }

  /**
   * For manual control of the shooter
   * @param throttle (-1 to 1) voltage to set to shooter to
   */
  public void setThrottle(double throttle) {
    motorOne.set(throttle);
    motorTwo.set(throttle);
  }

  /**
   * Uses the PID method `setReference` to set the speed of the shooter
   */
  public void setVelocity(double velocityRPM) {
    sparkPIDControllerOne.setReference(velocityRPM, ControlType.kVelocity);
    sparkPIDControllerTwo.setReference(velocityRPM, ControlType.kVelocity);
  }

  /**
   * Fully configures all sparkMaxs
   *    - sets factory defaults
   *    - sets the feedback encoder
   *    - sets current limits and output ranges
   *    - sets the FPID constants
   */
  private void configureSpark() {
    motorOne.restoreFactoryDefaults();
    motorTwo.restoreFactoryDefaults();

    motorOne.setInverted(true);
    motorTwo.setInverted(true);

    motorOne.setIdleMode(IdleMode.kCoast);
    motorTwo.setIdleMode(IdleMode.kCoast);

    sparkPIDControllerOne.setFeedbackDevice(sparkEncoderOne);
    sparkPIDControllerTwo.setFeedbackDevice(sparkEncoderTwo);

    sparkPIDControllerOne.setOutputRange(-Constants.ShooterConstants.MIN_OUTPUT, Constants.ShooterConstants.MAX_OUTPUT);

    motorOne.setSmartCurrentLimit(40, 35);

    sparkPIDControllerTwo.setOutputRange(-Constants.ShooterConstants.MIN_OUTPUT, Constants.ShooterConstants.MAX_OUTPUT);

    motorTwo.setSmartCurrentLimit(40, 35);

    // set PID coefficients
    sparkPIDControllerOne.setFF(Constants.ShooterConstants.SHOOTER_ONE_FPID.kF);
    sparkPIDControllerOne.setP(Constants.ShooterConstants.SHOOTER_ONE_FPID.kP);
    sparkPIDControllerOne.setI(Constants.ShooterConstants.SHOOTER_ONE_FPID.kI);
    sparkPIDControllerOne.setD(Constants.ShooterConstants.SHOOTER_ONE_FPID.kD);

    sparkPIDControllerTwo.setFF(Constants.ShooterConstants.SHOOTER_TWO_FPID.kF);
    sparkPIDControllerTwo.setP(Constants.ShooterConstants.SHOOTER_TWO_FPID.kP);
    sparkPIDControllerTwo.setI(Constants.ShooterConstants.SHOOTER_TWO_FPID.kI);
    sparkPIDControllerTwo.setD(Constants.ShooterConstants.SHOOTER_TWO_FPID.kD);
  }

  /**
   * Logs velocity data to Smartdashboard
   */
  public void log() {
    SmartDashboard.putNumber("velocity one", sparkEncoderOne.getVelocity());
    SmartDashboard.putNumber("velocity two", sparkEncoderTwo.getVelocity());
  }

  public double getBallExitVelocity() {
    return (sparkEncoderOne.getVelocity() * Constants.ShooterConstants.FLYWHEEL_RADIUS * Constants.ShooterConstants.CONVERSION_FACTOR); // tangential velocity = angular velocity * radius
  }

  // returns RPM 
  public double calculateShooterSpeed(double distanceFromTarget, double armAngle) {
    return 600 + (((1 / (Math.sqrt((Constants.ShooterConstants.GOAL_HEIGHT - (distanceFromTarget * Math.tan(armAngle)) / (-.5 * Constants.ShooterConstants.G))))) * (distanceFromTarget / Math.cos(armAngle)) * (Constants.ShooterConstants.CONVERSION_FACTOR / Constants.ShooterConstants.FLYWHEEL_RADIUS)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println("One: " + sparkEncoderOne.getVelocity());
    // System.out.println("Two: " + sparkEncoderTwo.getVelocity());
    log();
    // setVelocity(SmartDashboard.getNumber("desired velocity", 0));
    // sparkPIDControllerOne.setD(SmartDashboard.getNumber("d value", 0));
  }
}
