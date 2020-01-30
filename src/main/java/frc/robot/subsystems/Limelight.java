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
   * Gets the angle between the shooter and the front of the goal.
   * 
   * @param armAngle the angle that the arm is at, in radians
   * @return the angle between the shooter and the front of the goal, in radians
   */
  public double getShooterGoalAngle(double armAngle) {

    // The height between the center of the Limelight and the floor, in inches
    double limelightHeight = LimelightConstants.ARM_LENGTH_INCHES * Math.sin(armAngle)
        + (LimelightConstants.ARM_HEIGHT_INCHES + LimelightConstants.STANDOFF_INCHES) * Math.cos(armAngle)
        + LimelightConstants.ARM_HEIGHT_INCHES;

    // The height between the center of the shooter and the floor, in inches
    double shooterHeight = LimelightConstants.ARM_LENGTH_INCHES * Math.sin(armAngle)
        + (1 / 2.0) * LimelightConstants.ARM_HEIGHT_INCHES * Math.cos(armAngle)
        + LimelightConstants.ARM_AXLE_HEIGHT_INCHES;

    // The distance between the center of the Limelight and the goal, in inches
    double limelightDistance = (LimelightConstants.GOAL_HEIGHT_INCHES - limelightHeight) / Math.sin(armAngle);

    // The distance between the center of the shooter and the goal, in inches
    double shooterDistance = (LimelightConstants.GOAL_HEIGHT_INCHES - shooterHeight) / Math.sin(armAngle);

    return (Math.PI / 2.0) - Math.asin((limelightDistance / shooterDistance) * Math.sin((Math.PI / 2.0) + Math.toRadians(ty.getDouble(0.0))));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
