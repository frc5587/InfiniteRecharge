/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends PIDSubsystem {
  private final CANSparkMax leftLeader = new CANSparkMax(DrivetrainConstants.LEFT_LEADER, MotorType.kBrushless);
  private final CANSparkMax leftFollower = new CANSparkMax(DrivetrainConstants.LEFT_FOLLOWER, MotorType.kBrushless);
  private final CANSparkMax rightLeader = new CANSparkMax(DrivetrainConstants.RIGHT_LEADER, MotorType.kBrushless);
  private final CANSparkMax rightFollower = new CANSparkMax(DrivetrainConstants.RIGHT_FOLLOWER, MotorType.kBrushless);

  private final CANEncoder leftEncoder = leftLeader.getEncoder();
  private final CANEncoder rightEncoder = rightLeader.getEncoder();
  private final AHRS ahrs = new AHRS();

  private final DifferentialDrive differentialDrive;
  private final DifferentialDriveOdometry odometry;

  /**
   * Creates a new Drive.
   */
  public Drivetrain() {
    super(
        // The PIDController used for auto-centring the drivetrain
        new PIDController(DrivetrainConstants.TURN_FPID.kP, DrivetrainConstants.TURN_FPID.kI,
            DrivetrainConstants.TURN_FPID.kD));

    var leftGroup = new SpeedControllerGroup(leftLeader, leftFollower);
    var rightGroup = new SpeedControllerGroup(rightLeader, rightFollower);

    leftGroup.setInverted(true);
    rightGroup.setInverted(true);

    this.differentialDrive = new DifferentialDrive(leftGroup, rightGroup);

    var currentAngle = Rotation2d.fromDegrees(getHeading());
    this.odometry = new DifferentialDriveOdometry(currentAngle);

    // Configure turn PID
    this.disable();
    var controller = this.getController();
    controller.enableContinuousInput(-180, 180);
    controller.setIntegratorRange(-1, 1);
    controller.setTolerance(DrivetrainConstants.TURN_PID_TOLERANCE_DEG);
  }

  public void arcadeDrive(double throttle, double curve) {
    differentialDrive.arcadeDrive(throttle, curve);
  }

  public void tankLR(double leftThrottle, double rightThrottle) {
    differentialDrive.tankDrive(leftThrottle, rightThrottle, false);
  }

  public void stop() {
    differentialDrive.stopMotor();
  }

  public double getLeftPosition() {
    return leftEncoder.getPosition();
  }

  public double getRightPosition() {
    return rightEncoder.getPosition();
  }

  public double getLeftVelocity() {
    // TODO: convert to m/s
    return leftEncoder.getVelocity();
  }

  public double getRightVelocity() {
    // TODO: convert to m/s
    return rightEncoder.getVelocity();
  }

  public double getHeading() {
    return ahrs.getAngle() * (DrivetrainConstants.GYRO_POSITIVE_COUNTERCLOCKWISE ? 1 : -1);
  }

  public double getHeading(double wrapValue) {
    var heading = getHeading() % 360;
    if (heading > 180) {
      return heading - 360;
    } else if (heading < -180) {
      return heading + 360;
    } else {
      return heading;
    }
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output and add the turn PID feed forward constant
    arcadeDrive(DrivetrainConstants.TURN_PID_FORWARD_THROTTLE, output + DrivetrainConstants.TURN_FPID.kF);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return ahrs.getAngle();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  @Override
  public void periodic() {
    // Update the pose
    var gyroAngle = Rotation2d.fromDegrees(getHeading());
    odometry.update(gyroAngle, getLeftPosition(), getRightPosition());
  }
}
