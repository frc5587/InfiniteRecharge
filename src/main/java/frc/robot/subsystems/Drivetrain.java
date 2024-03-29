/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.LimitedPoseMap;

public class Drivetrain extends PIDSubsystem {
  private final CANSparkMax leftLeader = new CANSparkMax(DrivetrainConstants.LEFT_LEADER, MotorType.kBrushless);
  private final CANSparkMax leftFollower = new CANSparkMax(DrivetrainConstants.LEFT_FOLLOWER, MotorType.kBrushless);
  private final CANSparkMax rightLeader = new CANSparkMax(DrivetrainConstants.RIGHT_LEADER, MotorType.kBrushless);
  private final CANSparkMax rightFollower = new CANSparkMax(DrivetrainConstants.RIGHT_FOLLOWER, MotorType.kBrushless);

  private final SpeedControllerGroup leftGroup = new SpeedControllerGroup(leftLeader, leftFollower);
  private final SpeedControllerGroup rightGroup = new SpeedControllerGroup(rightLeader, rightFollower);

  private final CANEncoder leftEncoder = leftLeader.getAlternateEncoder(AlternateEncoderType.kQuadrature,
      DrivetrainConstants.TICKS_PER_REV);
  private final CANEncoder rightEncoder = rightLeader.getAlternateEncoder(AlternateEncoderType.kQuadrature,
      DrivetrainConstants.TICKS_PER_REV);
  private final AHRS ahrs = new AHRS();

  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftGroup, rightGroup);
  private final DifferentialDriveOdometry odometry;
  private final LimitedPoseMap poseHistory = new LimitedPoseMap(DrivetrainConstants.HISTORY_LIMIT);

  /**
   * Creates a new Drive.
   */
  public Drivetrain() {
    super(
        // The PIDController used for auto-centring the drivetrain
        new PIDController(DrivetrainConstants.TURN_FPID.kP, DrivetrainConstants.TURN_FPID.kI,
            DrivetrainConstants.TURN_FPID.kD));

    leftLeader.restoreFactoryDefaults();
    leftFollower.restoreFactoryDefaults();
    rightLeader.restoreFactoryDefaults();
    rightFollower.restoreFactoryDefaults();

    setIdleMode(IdleMode.kCoast);

    leftGroup.setInverted(DrivetrainConstants.LEFT_SIDE_INVERTED);
    rightGroup.setInverted(DrivetrainConstants.RIGHT_SIDE_INVERTED);
    leftEncoder.setInverted(DrivetrainConstants.LEFT_ENCODER_INVERTED);
    rightEncoder.setInverted(DrivetrainConstants.RIGHT_ENCODER_INVERTED);

    leftLeader.setSmartCurrentLimit(DrivetrainConstants.SMART_CURRENT_LIMIT);
    leftFollower.setSmartCurrentLimit(DrivetrainConstants.SMART_CURRENT_LIMIT);
    rightLeader.setSmartCurrentLimit(DrivetrainConstants.SMART_CURRENT_LIMIT);
    rightFollower.setSmartCurrentLimit(DrivetrainConstants.SMART_CURRENT_LIMIT);

    leftLeader.setSecondaryCurrentLimit(DrivetrainConstants.HARD_CURRENT_LIMIT);
    leftFollower.setSecondaryCurrentLimit(DrivetrainConstants.HARD_CURRENT_LIMIT);
    rightLeader.setSecondaryCurrentLimit(DrivetrainConstants.HARD_CURRENT_LIMIT);
    rightFollower.setSecondaryCurrentLimit(DrivetrainConstants.HARD_CURRENT_LIMIT);

    var currentAngle = Rotation2d.fromDegrees(getHeading360());
    this.odometry = new DifferentialDriveOdometry(currentAngle);
    // TODO: Allow for selecting of several initial positions with odometry, instead
    // of assuming x=0, y=0

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

  public void tankLRVolts(double leftVolts, double rightVolts) {
    // Convert voltages to percents by dividing by maximum value
    // tankLR(-leftVoltage / 12.0, -rightVoltage / 12.0);
    leftGroup.setVoltage(-leftVolts);
    rightGroup.setVoltage(rightVolts);
    differentialDrive.feed();
  }

  public void stop() {
    differentialDrive.stopMotor();
  }

  public double getLeftPositionRotations() {
    return leftEncoder.getPosition() / 2;
  }

  public double getRightPositionRotations() {
    return rightEncoder.getPosition() / 2;
  }

  private double rotationsToMeters(double rotations) {
    // number of rotations * circumfrence of wheel
    return rotations * DrivetrainConstants.WHEEL_DIAMETER_METERS * Math.PI;
  }

  public double getLeftPositionMeters() {
    return rotationsToMeters(getLeftPositionRotations());
  }

  public double getRightPositionMeters() {
    return rotationsToMeters(getRightPositionRotations());
  }

  public double getRightVelocityRotationsPerMinute() {
    return rightEncoder.getVelocity() / 2;
  }

  public double getLeftVelocityRotationsPerMinute() {
    return leftEncoder.getVelocity() / 2;
  }

  private double rotationsPerMinuteToMetersPerSecond(double rotationsPerMinute) {
    var radiansPerSecond = Units.rotationsPerMinuteToRadiansPerSecond(rotationsPerMinute);
    var linearMetersPerSecond = radiansPerSecond * DrivetrainConstants.WHEEL_RADIUS_METERS;
    return linearMetersPerSecond;
  }

  public double getLeftVelocityMetersPerSecond() {
    return rotationsPerMinuteToMetersPerSecond(getLeftVelocityRotationsPerMinute());
  }

  public double getRightVelocityMetersPerSecond() {
    return rotationsPerMinuteToMetersPerSecond(getRightVelocityRotationsPerMinute());
  }

  /**
   * Get the raw, unbounded heading of the drivetrain's gyroscope in degrees. To
   * get the bounded gyro scope heading between -180 and +180, use
   * {@link #getHeading180()} instead.
   * 
   * @return the raw, unbounded heading of the drivetrain gyro in degrees
   */
  public double getHeading() {
    return ahrs.getAngle() * (DrivetrainConstants.INVERT_GYRO_DIRECTION ? -1 : 1);
    // return gyro.getAngle();
  }

  public double getHeading360() {
    return Math.IEEEremainder(getHeading(), 360.0);
    // return gyro.getAngle();
  }

  /**
   * Get the heading of the drivetrain's gyroscope, wrapped to be within -180 to
   * +180. With these bounds, 0 degrees is still the angle that the drivetrain
   * started with when the robot was turned on.
   * 
   * @returns the heading in degrees, where -180 < heading < +180
   */
  public double getHeading180() {
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
  protected void useOutput(double output, double setpoint) {
    // Use the output and add the turn PID feed forward constant
    arcadeDrive(DrivetrainConstants.TURN_PID_FORWARD_THROTTLE,
        output + Math.copySign(DrivetrainConstants.TURN_FPID.kF, output));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getHeading();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocityMetersPerSecond(), getRightVelocityMetersPerSecond());
  }

  public Pose2d getClosestPoseAtTime(double time) {
    return poseHistory.getClosest(time);
  }

  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public void resetHeading() {
    ahrs.reset();
  }

  public void resetOdometry() {
    resetHeading();
    resetEncoders();
    odometry.resetPosition(new Pose2d(), new Rotation2d());
  }

  public void setIdleMode(IdleMode idleMode) {
    leftLeader.setIdleMode(idleMode);
    leftFollower.setIdleMode(idleMode);
    rightLeader.setIdleMode(idleMode);
    rightFollower.setIdleMode(idleMode);
  }

  @Override
  public void periodic() {
    // Call periodic of PIDSubsystem to ensure PID controller runs
    super.periodic();

    // Update the pose
    var gyroAngle = Rotation2d.fromDegrees(getHeading360());
    odometry.update(gyroAngle, getLeftPositionMeters(), getRightPositionMeters());

    // Log the pose
    poseHistory.put(Timer.getFPGATimestamp(), getPose());

    // Log debug data
    SmartDashboard.putNumber("L Distance Feet", Units.metersToFeet(getLeftPositionMeters()));
    SmartDashboard.putNumber("R Distance Feet", Units.metersToFeet(getRightPositionMeters()));
    SmartDashboard.putNumber("L Distance Meters", getLeftPositionMeters());
    SmartDashboard.putNumber("R Distance Meters", getRightPositionMeters());
    SmartDashboard.putNumber("L Velocity Meters", getLeftVelocityMetersPerSecond());
    SmartDashboard.putNumber("R Velocity Meters", getRightVelocityMetersPerSecond());
    SmartDashboard.putNumber("Heading", getHeading360());
    SmartDashboard.putString("Pose", getPose().toString());
    SmartDashboard.putString("Wheel Speed", getWheelSpeeds().toString());
  }
}
