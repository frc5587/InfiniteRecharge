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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MLConstants;

public class MachineLearning extends SubsystemBase {
  private final NetworkTable mlTable = NetworkTableInstance.getDefault().getTable("ML");
  private final NetworkTableEntry numberObjectsEntry = mlTable.getEntry("nb_objects");
  private final NetworkTableEntry boxesEntry = mlTable.getEntry("boxes");
  private final NetworkTableEntry captureTimeEntry = mlTable.getEntry("pre_time");
  private final NetworkTableEntry postAnalysisTimeEntry = mlTable.getEntry("post_time");

  /**
   * Creates a new MachineLearning.
   */
  public MachineLearning() {

  }

  public double getImageCaptureFPGATime() {
    // Get the difference b/t now on the RPi and when the picture was taken
    var timeDelta = postAnalysisTimeEntry.getDouble(0.0) - captureTimeEntry.getDouble(0.0);
    // Adjust the FPGA time by that difference
    return Timer.getFPGATimestamp() - timeDelta;
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

  public static class Box {
    private final double topLeftX, topLeftY, bottomRightX, bottomRightY;

    public Box(double... boxPoints) {
      this.topLeftX = boxPoints[0];
      this.topLeftY = boxPoints[1];
      this.bottomRightX = boxPoints[2];
      this.bottomRightY = boxPoints[3];
    }

    public double getArea() {
      return (topLeftX - bottomRightX) * (topLeftY - bottomRightY);
    }

    private double getCenterY() {
      return (topLeftY + bottomRightY) / 2;
    }

    private double getCenterX() {
      return (topLeftX + bottomRightX) / 2;
    }

    public Rotation2d getVerticalAngleDelta() {
      var yDifference = getCenterY() - (MLConstants.CAMERA_HEIGHT_PIXELS / 2);
      var rawAngle = Math.atan2(yDifference, MLConstants.CAMERA_FOCAL_LENGTH_PIXELS);
      return new Rotation2d(rawAngle);
    }

    public Rotation2d getHorizontalAngleDelta() {
      var xDifference = getCenterX() - (MLConstants.CAMERA_HEIGHT_PIXELS / 2);
      var rawAngle = Math.atan2(xDifference, MLConstants.CAMERA_FOCAL_LENGTH_PIXELS);
      return new Rotation2d(rawAngle);
    }

    private double getDistance() {
      var verticalDifferencePixels = getCenterY() - (MLConstants.CAMERA_HEIGHT_PIXELS / 2);
      var tanVerticalAngle = verticalDifferencePixels / MLConstants.CAMERA_FOCAL_LENGTH_PIXELS;

      var heightDifference = getHeightDifference();
      return heightDifference / tanVerticalAngle;
    }

    public Transform2d getTransform() {
      var distance = getDistance();
      var horizontalAngle = getHorizontalAngleDelta();
      var xOffset = distance * horizontalAngle.getCos();
      var yOffset = distance * horizontalAngle.getSin();
      var translation = new Translation2d(xOffset, yOffset);
      return new Transform2d(translation, horizontalAngle);
    }

    private static double getHeightDifference() {
      return MLConstants.CENTER_CAMERA_HEIGHT_METERS - MLConstants.POWER_CELL_RADIUS_METERS;
    }
  }
}
