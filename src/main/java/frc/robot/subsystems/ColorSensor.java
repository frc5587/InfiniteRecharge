/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants.ControlPanelConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

/**
 * The subsystem for the color sensor.
 */
public class ColorSensor extends SubsystemBase {
  private ColorSensorV3 colorSensor = new ColorSensorV3(ControlPanelConstants.i2cPort);
  private ColorMatch colorMatcher = new ColorMatch();

  /**
   * Add all colors that are desired matches to an ArrayList of matches that
   * belongs to the colormatcher
   */
  public void addAllColorMatches() {
    colorMatcher.addColorMatch(ControlPanelConstants.BLUE_TARGET);
    colorMatcher.addColorMatch(ControlPanelConstants.GREEN_TARGET);
    colorMatcher.addColorMatch(ControlPanelConstants.RED_TARGET);
    colorMatcher.addColorMatch(ControlPanelConstants.YELLOW_TARGET);
  }

  /**
   * Get the color that the color sensor is currently detecting
   * 
   * @return the color that the sensor is detecting
   */
  public Color getCurrentDetectedColor() {
    return colorSensor.getColor(); 
  }

  /**
   * Get the color in the list of desired matches that matches the detected color
   * the closest
   * 
   * @return the color that matches the detected color the closest from the list
   * of options
   */
  public ColorMatchResult getClosestColorMatch() {
    return colorMatcher.matchClosestColor(getCurrentDetectedColor());
  }

  /**
   * Convert the closest matching Color to a String
   * 
   * @return the Color that matches as a String
   */
  public String getClosestColorMatchToString() {
    if (getClosestColorMatch().color == ControlPanelConstants.BLUE_TARGET) {
      return "Blue";
    } else if (getClosestColorMatch().color == ControlPanelConstants.GREEN_TARGET) {
      return "Green";
    } else if (getClosestColorMatch().color == ControlPanelConstants.RED_TARGET) {
      return "Red";
    } else if (getClosestColorMatch().color == ControlPanelConstants.YELLOW_TARGET) {
      return "Yellow";
    } else {
      return "Unknown";
    }
  }

  /**
   * Get the confidence that the colormatcher feels that it has regarding the
   * color match
   * 
   * @return a double, where 0 <= double < 1, that represents how confident the
   * colormatcher is with its choice
   */
  public double getConfidence() {
    return getClosestColorMatch().confidence;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
