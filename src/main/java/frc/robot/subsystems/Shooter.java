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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;;

public class Shooter extends SubsystemBase {
  private final CANSparkMax motorOne = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_ONE,
      MotorType.kBrushless);
  private final CANSparkMax motorTwo = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_TWO,
      MotorType.kBrushless);
  private final CANPIDController sparkPIDControllerOne = motorOne.getPIDController();
  private final CANPIDController sparkPIDControllerTwo = motorTwo.getPIDController();

  private final CANEncoder sparkEncoderOne = motorOne.getEncoder();
  private final CANEncoder sparkEncoderTwo = motorTwo.getEncoder();

  private Double mostRecentSetpoint = null;

  /**
   * Creates the Shooter subsystem
   */
  public Shooter() {
    configureSpark();
  }

  /**
   * For manual control of the shooter
   * 
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
    var velocityMotor = velocityRPM * ShooterConstants.GEAR_RATIO;
    sparkPIDControllerOne.setReference(velocityRPM, ControlType.kVelocity);
    sparkPIDControllerTwo.setReference(velocityRPM, ControlType.kVelocity);

    mostRecentSetpoint = velocityMotor;
  }

  /**
   * Fully configures all sparkMaxs - sets factory defaults - sets the feedback
   * encoder - sets current limits and output ranges - sets the FPID constants
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

    sparkPIDControllerOne.setOutputRange(-ShooterConstants.MIN_OUTPUT, ShooterConstants.MAX_OUTPUT);

    motorOne.setSmartCurrentLimit(40, 35);

    sparkPIDControllerTwo.setOutputRange(-ShooterConstants.MIN_OUTPUT, ShooterConstants.MAX_OUTPUT);

    motorTwo.setSmartCurrentLimit(40, 35);

    // set PID coefficients
    sparkPIDControllerOne.setFF(ShooterConstants.SHOOTER_ONE_FPID.kF);
    sparkPIDControllerOne.setP(ShooterConstants.SHOOTER_ONE_FPID.kP);
    sparkPIDControllerOne.setI(ShooterConstants.SHOOTER_ONE_FPID.kI);
    sparkPIDControllerOne.setD(ShooterConstants.SHOOTER_ONE_FPID.kD);

    sparkPIDControllerTwo.setFF(ShooterConstants.SHOOTER_TWO_FPID.kF);
    sparkPIDControllerTwo.setP(ShooterConstants.SHOOTER_TWO_FPID.kP);
    sparkPIDControllerTwo.setI(ShooterConstants.SHOOTER_TWO_FPID.kI);
    sparkPIDControllerTwo.setD(ShooterConstants.SHOOTER_TWO_FPID.kD);
  }

  /**
   * Logs velocity data to Smartdashboard
   */
  public void log() {
    SmartDashboard.putNumber("velocity one", sparkEncoderOne.getVelocity());
    SmartDashboard.putNumber("velocity two", sparkEncoderTwo.getVelocity());
  }

  public double getBallExitVelocity() {
    return (sparkEncoderOne.getVelocity() * ShooterConstants.FLYWHEEL_RADIUS
        * ShooterConstants.CONVERSION_FACTOR); // tangential velocity = angular velocity * radius
  }

  public double getShooterSpeed() {
    return (sparkEncoderOne.getVelocity() + sparkEncoderTwo.getVelocity()) / 2;
  }

  /**
   * Calculates the speed that the shooter must spin at to get the powercell in the power port
   * based on the distance from the power port and the angle of the arm
   * 
   * @param distanceFromTarget the distance from the target - METERS
   * @param armAngle           the angle of the arm         - RADIANS
   * @return                   the speed of the shooter     - RPM
   */
  public static double calculateShooterSpeed(double armAngle) {
    
    return 60 * Math.sqrt(2 * ShooterConstants.G * Limelight.getWorkingHeight(armAngle))
        / (ShooterConstants.FLYWHEEL_CIRCUMFERENCE * Math.sin(armAngle) * ShooterConstants.GEARING);

    // return 1 / (Math.sqrt((Limelight.getWorkingHeight(armAngle) - (distanceFromTarget * Math.tan(armAngle)) / (-0.5 * ShooterConstants.G)))
    //               * (distanceFromTarget / Math.cos(armAngle))
    //               * (ShooterConstants.CONVERSION_FACTOR / ShooterConstants.FLYWHEEL_RADIUS)
    //               * ShooterConstants.GEARING);

    // return (1 / (Math.sqrt((Limelight.getWorkingHeight(armAngle)
    //     - (distanceFromTarget * Math.tan(armAngle)) / (-.5 * ShooterConstants.G)))))
    //     * (distanceFromTarget / Math.cos(armAngle))
    //     * (ShooterConstants.CONVERSION_FACTOR / ShooterConstants.FLYWHEEL_RADIUS);
  }

  @Override
  public void periodic() {
    log();
  }
}
