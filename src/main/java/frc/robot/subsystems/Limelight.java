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
 * The subsystem for the Limelight.
 */
public class Limelight extends SubsystemBase {
  private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  public NetworkTableEntry ty = limelightTable.getEntry("ty"); // angle measured in degrees

  /**
   * Gets the height that the Limelight is raised above the floor.
   * 
   * @param armAngle the angle that the arm is at, in radians
   * @return the height between the center of the Limelight and the floor, in inches
   */
  public double getLimelightHeight(double armAngle) {
    return LimelightConstants.ARM_LENGTH_INCHES * Math.sin(armAngle)
        + (LimelightConstants.ARM_HEIGHT_INCHES + LimelightConstants.STANDOFF_INCHES) * Math.cos(armAngle)
        + LimelightConstants.ARM_HEIGHT_INCHES;
  }

  /**
   * Gets the height that the shooter is raised above the floor.
   * 
   * @param armAngle the angle that the arm is at, in radians
   * @return the height between the center of the shooter and the floor, in inches
   */
  public double getShooterHeight(double armAngle) {
    return LimelightConstants.ARM_LENGTH_INCHES * Math.sin(armAngle)
        + (1 / 2.0) * LimelightConstants.ARM_HEIGHT_INCHES * Math.cos(armAngle)
        + LimelightConstants.ARM_AXLE_HEIGHT_INCHES;
  }

  /**
   * Gets the distance between the Limelight and the goal.
   * 
   * @param armAngle the angle that the arm is at, in radians
   * @return the distance between the Limelight and the goal, in inches
   */
  public double getLimelightDistance(double armAngle) {
    return (LimelightConstants.GOAL_HEIGHT_INCHES - getLimelightHeight(armAngle)) / Math.sin(armAngle);
  }

  /**
   * Gets the distance between the shooter and the goal.
   * 
   * @param armAngle the angle that the arm is at, in radians
   * @return the distance between the center of the shooter and the goal, in inches
   */
  public double getShooterDistance(double armAngle) {
    return (LimelightConstants.GOAL_HEIGHT_INCHES - getShooterHeight(armAngle)) / Math.sin(armAngle);
  }

  /**
   * Gets the angle between the shooter and the front of the goal.
   * 
   * @param armAngle the angle that the arm is at, in radians
   * @return the angle between the shooter and the front of the goal, in radians
   */
  public double getShooterGoalAngle(double armAngle) {
    return (Math.PI / 2.0) - Math.asin((getLimelightDistance(armAngle) / getShooterDistance(armAngle))
        * Math.sin((Math.PI / 2.0) + Math.toRadians(ty.getDouble(0.0))));
  }

  /**
   * Gets the horizontal difference between the shooter and the goal.
   * 
   * @param armAngle the angle that the arm is at, in radians
   * @return the horizontal difference between the shooter and the goal, in inches
   */
  public double getShooterHorizontalDifference(double armAngle) {
    return (LimelightConstants.GOAL_HEIGHT_INCHES - getShooterHeight(armAngle))
        / Math.tan(getShooterGoalAngle(armAngle) + armAngle);
  }

  /**
   * Gets the angle between the shooter and the inner goal.
   * 
   * @param armAngle the angle that the arm is at, in radians
   * @return the angle between the shooter and the inner goal, in radians
   */
  public double getShooterInnerGoalAngle(double armAngle) {
    return Math.atan((LimelightConstants.GOAL_HEIGHT_INCHES - getShooterHeight(armAngle))
        / (getShooterHorizontalDifference(armAngle) + LimelightConstants.INNER_OUTER_GOAL_DISTANCE_INCHES))
        - armAngle;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
