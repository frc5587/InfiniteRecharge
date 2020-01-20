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
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final CANSparkMax motorOne = new CANSparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_ONE,
      MotorType.kBrushless);
  private final CANSparkMax motorTwo = new CANSparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_TWO,
      MotorType.kBrushless);
  private final CANPIDController sparkPIDControllerOne = motorOne.getPIDController();
  private final CANPIDController sparkPIDControllerTwo = motorTwo.getPIDController();

  private final CANEncoder sparkEncoderOne = motorOne.getEncoder();
  private final CANEncoder sparkEncoderTwo = motorTwo.getEncoder();


  public Shooter() {     
    configureSpark();
  }

  public void setSpeed(double speed) {
    // motorOne.set(speed);
    // motorTwo.set(speed);
    sparkPIDControllerOne.setReference(1500, ControlType.kVelocity);
    sparkPIDControllerTwo.setReference(1500, ControlType.kVelocity);
  }

  private void configureSpark() {
    motorOne.restoreFactoryDefaults();
    motorTwo.restoreFactoryDefaults();

    motorOne.setInverted(true);
    motorTwo.setInverted(false);

    sparkPIDControllerOne.setFeedbackDevice(sparkEncoderOne);
    sparkPIDControllerTwo.setFeedbackDevice(sparkEncoderTwo);

    sparkPIDControllerOne.setOutputRange(-Constants.ShooterConstants.MIN_OUTPUT, Constants.ShooterConstants.MAX_OUTPUT);

    motorOne.setSmartCurrentLimit(40, 35);

    sparkPIDControllerTwo.setOutputRange(-Constants.ShooterConstants.MIN_OUTPUT, Constants.ShooterConstants.MAX_OUTPUT);

    motorTwo.setSmartCurrentLimit(40, 35);

    // set PID coefficients
    sparkPIDControllerOne.setFF(Constants.ShooterConstants.SHOOTER_FPID.kf);
    sparkPIDControllerOne.setP(Constants.ShooterConstants.SHOOTER_FPID.kP);
    sparkPIDControllerOne.setI(Constants.ShooterConstants.SHOOTER_FPID.kI);
    sparkPIDControllerOne.setD(Constants.ShooterConstants.SHOOTER_FPID.kD);

    sparkPIDControllerTwo.setFF(Constants.ShooterConstants.SHOOTER_TWO_FPID.kf);
    sparkPIDControllerTwo.setP(Constants.ShooterConstants.SHOOTER_TWO_FPID.kP);
    sparkPIDControllerTwo.setI(Constants.ShooterConstants.SHOOTER_TWO_FPID.kI);
    sparkPIDControllerTwo.setD(Constants.ShooterConstants.SHOOTER_TWO_FPID.kD);
  }

  public void startRefresh() {
    SmartDashboard.putNumber("velocity one", sparkEncoderOne.getVelocity());
    SmartDashboard.putNumber("velocity two", sparkEncoderTwo.getVelocity());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
