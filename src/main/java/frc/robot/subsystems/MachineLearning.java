/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MLConstants;

public class MachineLearning extends SubsystemBase {
  private final NetworkTable mlTable = NetworkTableInstance.getDefault().getTable("ML");
  private final NetworkTableEntry numberObjectsEntry = mlTable.getEntry("nb_objects");
  private final NetworkTableEntry boxesEntry = mlTable.getEntry("boxes");

  /**
   * Creates a new MachineLearning.
   */
  public MachineLearning() {

  }

  public Box getClosestBox() {
    if (numberObjectsEntry.getDouble(0) > 0) {
      var boxes = boxesEntry.getDoubleArray(new double[] {});

      Box closestBox = null;
      var index = 0;
      var maxArea = Double.MIN_VALUE;
      while (index < boxes.length) {
        var currentBox = new Box(boxes[index], boxes[index + 1], boxes[index + 2], boxes[index + 3]);
        var area = currentBox.getArea();
        if (area > maxArea) {
          maxArea = area;
          closestBox = currentBox;
        }

        // Move on to the next box's index
        index += 4;
      }

      return closestBox;
    }

    return null;
  }

  public double getDistanceClosestBox() {
    var closestBox = getClosestBox();
    if (closestBox != null) {
      var verticalDifferencePixels = closestBox.getCenterY() - (MLConstants.CAMERA_HEIGHT_PIXELS / 2);
      var tanVerticalAngle = verticalDifferencePixels / MLConstants.CAMERA_FOCAL_LENGTH_PIXELS;

      var heightDifference = MLConstants.CENTER_CAMERA_HEIGHT_METERS - MLConstants.POWER_CELL_RADIUS_METERS;
      return heightDifference / tanVerticalAngle;
    }
    return Double.NaN;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static class Box {
    public final double topLeftX, topLeftY, bottomRightX, bottomRightY;

    public Box(double... boxPoints) {
      this.topLeftX = boxPoints[0];
      this.topLeftY = boxPoints[1];
      this.bottomRightX = boxPoints[2];
      this.bottomRightY = boxPoints[3];
    }

    public double getArea() {
      return (topLeftX - bottomRightX) * (topLeftY - bottomRightY);
    }

    public double getCenterY() {
      return (topLeftY + bottomRightY) / 2;
    }

    public double getCenterX() {
      return (topLeftX + bottomRightX) / 2;
    }

    public double getVerticalAngleRadians() {
      var yDifference = getCenterY() - (MLConstants.CAMERA_HEIGHT_PIXELS / 2);
      return Math.atan2(yDifference, MLConstants.CAMERA_FOCAL_LENGTH_PIXELS);
    }

    public double getHorizontalAngleRadians() {
      var xDifference = getCenterX() - (MLConstants.CAMERA_HEIGHT_PIXELS / 2);
      return Math.atan2(xDifference, MLConstants.CAMERA_FOCAL_LENGTH_PIXELS);
    }
  }
}
