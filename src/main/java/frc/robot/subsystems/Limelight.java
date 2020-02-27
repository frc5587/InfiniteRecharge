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

/**
 * The subsystem for the Limelight
 */
public class Limelight extends SubsystemBase {
  public NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  public NetworkTableEntry tv = limelightTable.getEntry("tv");
  public NetworkTableEntry tx = limelightTable.getEntry("tx");
  public NetworkTableEntry ty = limelightTable.getEntry("ty");
  public double lastDistance;

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
   * @return the height between the center of the Limelight and the floor, in
   *         meters
   */
  private static double getLimelightHeight(double currentArmAngle) {
    return LimelightConstants.ARM_LENGTH_METERS * Math.sin(currentArmAngle)
        + (LimelightConstants.ARM_HEIGHT_METERS + LimelightConstants.STANDOFF_METERS) * Math.cos(currentArmAngle)
        + LimelightConstants.ARM_HEIGHT_METERS;
  }

  /**
   * Get the height that the shooter is raised above the floor
   * 
   * @param currentArmAngle the angle that the arm is currently at, in radians
   * @return the height between the center of the shooter and the floor, in meters
   */
  public static double getShooterHeight(double currentArmAngle) {
    return LimelightConstants.ARM_LENGTH_METERS * Math.sin(currentArmAngle)
        + 0.5 * LimelightConstants.ARM_HEIGHT_METERS * Math.cos(currentArmAngle)
        + LimelightConstants.ARM_AXLE_HEIGHT_METERS;
  }

  /**
   * Gets the vertical distance in between the target and shooter
   * 
   * @param currentArmAngleRadians arm angle           - RADIANS
   * @return                       vertical difference - METERS
   */
  public static double getWorkingHeight(double currentArmAngleRadians) {
    return LimelightConstants.GOAL_HEIGHT_METERS - getShooterHeight(currentArmAngleRadians);
  }

  /**
   * Get the distance between the Limelight and the target
   * 
   * @param currentArmAngle the angle that the arm is currently at, in radians
   * @return the distance between the Limelight and the target, in meters
   */
  public double getLimelightGoalHorizontalDifference(double currentArmAngle) {
    return (LimelightConstants.GOAL_HEIGHT_METERS - getLimelightHeight(currentArmAngle)) / Math.tan(currentArmAngle + Math.toRadians(getVerticalAngleOffset()));
  }

  public double getStubbyThing(double currentArmAngle) {
    return (0.5 * LimelightConstants.ARM_HEIGHT_METERS + LimelightConstants.STANDOFF_METERS) * Math.sin(currentArmAngle);
  }

  public double getShooterGoalHorizontalDifference(double currentArmAngle) {
    this.lastDistance = this.isTargetDetected() ? getLimelightGoalHorizontalDifference(currentArmAngle) - getStubbyThing(currentArmAngle) : this.lastDistance;
    return this.lastDistance;
  }

   /**
   * Get the angle between the shooter and the front of the target
   * 
   * @param currentArmAngle the angle that the arm is currently at, in radians
   * @return the angle between the shooter and the front of the target, in
   *         radians, without accounting for drop
   */
  public double getShooterFrontGoalAngle(double currentArmAngle) {
    return Math.atan((LimelightConstants.GOAL_HEIGHT_METERS - getShooterHeight(currentArmAngle)) / getShooterGoalHorizontalDifference(currentArmAngle));
  }

  // /**
  //  * Get the distance between the Limelight and the target
  //  * 
  //  * @param currentArmAngle the angle that the arm is currently at, in radians
  //  * @return the distance between the Limelight and the target, in meters
  //  */
  // public double getLimelightDistance(double currentArmAngle) {
  //   return (LimelightConstants.GOAL_HEIGHT_METERS - getLimelightHeight(currentArmAngle)) / Math.sin(currentArmAngle);
  // }

  // /**
  //  * Get the distance between the shooter and the target
  //  * 
  //  * @param currentArmAngle the angle that the arm is currently at, in radians
  //  * @return the distance between the center of the shooter and the target, in
  //  *         meters
  //  */
  // public double getShooterDistance(double currentArmAngle) {
  //   return (LimelightConstants.GOAL_HEIGHT_METERS - getShooterHeight(currentArmAngle)) / Math.sin(currentArmAngle);
  // }

  // /**
  //  * Get the angle between the shooter and the front of the target
  //  * 
  //  * @param currentArmAngle the angle that the arm is currently at, in radians
  //  * @return the angle between the shooter and the front of the target, in
  //  *         radians, without accounting for drop
  //  */
  // public double getShooterFrontGoalAngle(double currentArmAngle) {
  //   return (Math.PI / 2.0) - Math.asin((getLimelightDistance(currentArmAngle) / getShooterDistance(currentArmAngle))
  //       * Math.sin((Math.PI / 2.0) + Math.toRadians(getVerticalAngleOffset())));
  // }

  /**
   * Get the angle between the shooter and the inner target
   * 
   * @param currentArmAngle the angle that the arm is currently at, in radians
   * @return the angle between the shooter and the inner target, in radians,
   *         without accounting for drop
   */
  private double getShooterInnerGoalAngle(double currentArmAngle) {
    return Math.atan((LimelightConstants.GOAL_HEIGHT_METERS - getShooterHeight(currentArmAngle))
        / (getShooterGoalHorizontalDifference(currentArmAngle, Target.FRONT)
            + LimelightConstants.INNER_OUTER_GOAL_DISTANCE_METERS))
        - currentArmAngle;
  }

  /**
   * Get the angle between the shooter and the chosen target
   * 
   * @param currentArmAngle the angle that the arm is currently at, in radians
   * @param t               the desired target
   * @return the angle between the shooter and the desired target, in radians,
   *         without accounting for drop
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
   * @param t               the desired target
   * @return the horizontal difference between the shooter and the desired target,
   *         in meters
   */
  private double getShooterGoalHorizontalDifference(double currentArmAngle, Target t) {
    return (LimelightConstants.GOAL_HEIGHT_METERS - getShooterHeight(currentArmAngle))
        / Math.tan(getUnadjustedAngle(currentArmAngle, t));
  }

  /**
   * Get how much the ball being thrown deviates from a straight line trajectory
   * 
   * @param currentArmAngle the angle that the arm is currently at, in radians
   * @param t               the desired target
   * @param shooterVelocity the velocity of the shooter, in meters / second
   * @return the height difference between the straight line trajectory and the
   *         actual trajectory, in meters
   */
  public double getDropHeight(double currentArmAngle, double shooterVelocity) {
    var unadjustedAngle = getShooterFrontGoalAngle(currentArmAngle);
    var horizontalDifference = getShooterGoalHorizontalDifference(unadjustedAngle);
    return horizontalDifference * Math.tan(unadjustedAngle) - 0.5 * LimelightConstants.G_METERS_PER_SECOND_SQUARED
        * Math.pow(horizontalDifference / (shooterVelocity * Math.cos(unadjustedAngle)), 2);
  }

  /**
   * Get the angle that the arm should be at, adjusted for the drop in height
   * 
   * @param currentArmAngle the angle that the arm is currently at, in radians
   * @param t               the desired target
   * @param shooterVelocity the velocity of the shooter, in meters / second
   * @return the angle that the arm should be at, in radians
   */
  public double getAdjustedAngle(double currentArmAngle, double shooterVelocity) {
    var unadjustedAngle = getShooterFrontGoalAngle(currentArmAngle);
    return Math.atan((LimelightConstants.GOAL_HEIGHT_METERS - getShooterHeight(unadjustedAngle) + getDropHeight(unadjustedAngle, shooterVelocity))
            / getShooterGoalHorizontalDifference(unadjustedAngle));
  }

  /***
   * Calculates the angle (in degrees) that the arm should move to based on the
   * current angle the arm is at (in radians) and various distances it reads from
   * the limelight
   * 
   * @param currentArmAngleRadians the angle of the arm                                        - RADIANS
   * @param t                      the desired target
   * @return                       the angle the arm should move to in order to hit the target - DEGREES
   *                               at the top of the arc in the powercell's trajectory         
   */
  public double calculateArmMovement(double currentArmAngleRadians, Target t) {
    // the horizontal distance between the target and shooter
    double distanceMeters = this.getShooterGoalHorizontalDifference(currentArmAngleRadians);
    return calculateArmAngleDegrees(distanceMeters, getWorkingHeight(currentArmAngleRadians));
  }

  /**
   * Calculates the optimum angle to set the arm, in order to use the least amount of
   * power shooting the powercells. This means that the powercell must enter the power
   * port at the top of its arc.
   * 
   * @param distanceMeters the horizontal distance between the shooter and target - METERS
   * @param height         the height of the target - METERS
   * @return               the angle to set the arm - DEGREES
   */
  private static double calculateArmAngleDegrees(double distanceMeters, double height) {
    return Math.toDegrees(Math.atan(2 * (height) / distanceMeters));
  }

  /**
   * Calculates the speed that the shooter should spin at to get the power cell in the power
   * port using the lowest speed possible
   * 
   * @param currentArmAngleRadians arm angle               - RADIANS
   * @param t                      the target
   * @return                       the speed of the shooter - RPM
   */
public double calculateShooterSpeed(double currentArmAngleRadians, Target t) {
  return Shooter.calculateShooterSpeed(this.getShooterGoalHorizontalDifference(currentArmAngleRadians, t), currentArmAngleRadians);
}


  /**
   * Targets that the robot can shoot at
   */
  public static enum Target {
    FRONT, INNER
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
