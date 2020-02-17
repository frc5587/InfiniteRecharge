/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.LimelightConstants.Target;

/**
 * The subsystem for the Limelight
 */
public class Limelight extends SubsystemBase {
  private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tv = limelightTable.getEntry("tv");
  private NetworkTableEntry tx = limelightTable.getEntry("tx");
  private NetworkTableEntry ty = limelightTable.getEntry("ty");

  /**
   * Get whether the target is being detected by the Limelight
   * 
   * @return whether the target is being detected
   */
  public boolean isTargetDetected() {
    return tv.getDouble(0) == 1;
  }

  /**
   * Get the horizontal angle between the Limelight and the target
   *
   * @return the horizontal angle between the Limelight and the target in degrees
   */
  public double getHorizontalAngleOffset() {
    return tx.getDouble(0);
  }

  /**
   * Get the vertical angle between the Limelight and the target
   *
   * @return the vertical angle between the Limelight and the target in degrees
   */
  public double getVerticalAngleOffset() {
    return ty.getDouble(0);
  }

  /**
   * Get the height that the Limelight is raised above the floor
   * 
   * @param currentArmAngle the angle that the arm is currently at, in radians
   * @return the height between the center of the Limelight and the floor, in inches
   */
  private double getLimelightHeight(double currentArmAngle) {
    return LimelightConstants.ARM_LENGTH_INCHES * Math.sin(currentArmAngle)
        + (LimelightConstants.ARM_HEIGHT_INCHES + LimelightConstants.STANDOFF_INCHES) * Math.cos(currentArmAngle)
        + LimelightConstants.ARM_HEIGHT_INCHES;
  }

  /**
   * Get the height that the shooter is raised above the floor
   * 
   * @param currentArmAngle the angle that the arm is currently at, in radians
   * @return the height between the center of the shooter and the floor, in inches
   */
  private double getShooterHeight(double currentArmAngle) {
    return LimelightConstants.ARM_LENGTH_INCHES * Math.sin(currentArmAngle)
        + (1 / 2.0) * LimelightConstants.ARM_HEIGHT_INCHES * Math.cos(currentArmAngle)
        + LimelightConstants.ARM_AXLE_HEIGHT_INCHES;
  }

  /**
   * Get the distance between the Limelight and the target
   * 
   * @param currentArmAngle the angle that the arm is currently at, in radians
   * @return the distance between the Limelight and the target, in inches
   */
  private double getLimelightDistance(double currentArmAngle) {
    return (LimelightConstants.GOAL_HEIGHT_INCHES - getLimelightHeight(currentArmAngle)) / Math.sin(currentArmAngle);
  }

  /**
   * Get the distance between the shooter and the target
   * 
   * @param currentArmAngle the angle that the arm is currently at, in radians
   * @return the distance between the center of the shooter and the target, in inches
   */
  private double getShooterDistance(double currentAmAngle) {
    return (LimelightConstants.GOAL_HEIGHT_INCHES - getShooterHeight(currentAmAngle)) / Math.sin(currentAmAngle);
  }

  /**
   * Get the angle between the shooter and the front of the target
   * 
   * @param currentArmAngle the angle that the arm is currently at, in radians
   * @return the angle between the shooter and the front of the target, in radians, without accounting for drop
   */
  private double getShooterFrontGoalAngle(double currentArmAngle) {
    return (Math.PI / 2.0) - Math.asin((getLimelightDistance(currentArmAngle) / getShooterDistance(currentArmAngle))
        * Math.sin((Math.PI / 2.0) + Math.toRadians(getVerticalAngleOffset())));
  }

  /**
   * Get the angle between the shooter and the inner target
   * 
   * @param currentArmAngle the angle that the arm is currently at, in radians
   * @return the angle between the shooter and the inner target, in radians, without accounting for drop
   */
  private double getShooterInnerGoalAngle(double currentArmAngle) {
    return Math.atan((LimelightConstants.GOAL_HEIGHT_INCHES - getShooterHeight(currentArmAngle))
        / (getShooterGoalHorizontalDifference(currentArmAngle, Target.FRONT) + LimelightConstants.INNER_OUTER_GOAL_DISTANCE_INCHES))
        - currentArmAngle;
  }

  /**
   * Get the angle between the shooter and the chosen target
   * 
   * @param currentArmAngle the angle that the arm is currently at, in radians
   * @param t the desired target
   * @return the angle between the shooter and the desired target, in radians, without accounting for drop
   */
  public double getUnadjustedAngle(double currentArmAngle, Target t) {
    if (t == Target.FRONT) {
      return getShooterFrontGoalAngle(currentArmAngle) + currentArmAngle;
    } else {
      return getShooterInnerGoalAngle(currentArmAngle) + currentArmAngle;
    }
  }

  /**
   * Get the horizontal difference between the shooter and the desired target
   * 
   * @param currentArmAngle the angle that the arm is currently at, in radians
   * @param t the desired target
   * @return the horizontal difference between the shooter and the desired target, in inches
   */
  public double getShooterGoalHorizontalDifference(double currentArmAngle, Target t) {
    return (LimelightConstants.GOAL_HEIGHT_INCHES - getShooterHeight(currentArmAngle)) 
        / Math.tan(getUnadjustedAngle(currentArmAngle, t)); 
  }

  /**
   * Get how much the ball being thrown deviates from a straight line trajectory
   * 
   * @param currentArmAngle the angle that the arm is currently at, in radians
   * @param t the desired target
   * @return the height difference between the straight line trajectory and the actual trajectory, in inches
   */
  private double getDropHeight(double currentArmAngle, Target t) {
    return (Math.pow(getShooterDistance(currentArmAngle), 2)
        / LimelightConstants.SHOOTER_VELOCITY_INCHES_PER_SECOND) * LimelightConstants.G_INCHES_PER_SECOND_SQUARED;
  }

  /**
   * Get the angle that the arm should be at, adjusted for the drop in height
   * 
   * @param currentArmAngle the angle that the arm is currently at, in radians
   * @param t the desired target
   * @return the angle that the arm should be at, in radians
   */
  public double getAdjustedAngle(double currentArmAngle, Target t) {
    var unadjustedAngle = getUnadjustedAngle(currentArmAngle, t);
    return Math.atan((LimelightConstants.GOAL_HEIGHT_INCHES - getShooterHeight(unadjustedAngle) + getDropHeight(unadjustedAngle, t))
        / getShooterGoalHorizontalDifference(unadjustedAngle, t));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
