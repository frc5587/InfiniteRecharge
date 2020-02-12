/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ControlPanelConstants;

/**
 * The subsystem for the color sensor.
 */
public class ColorSensor extends SubsystemBase {
  private ColorSensorV3 colorSensor = new ColorSensorV3(ControlPanelConstants.i2cPort);
  private ColorMatch colorMatcher = new ColorMatch();
  private CANSparkMax colorRotator = new CANSparkMax(ControlPanelConstants.CONTROL_PANEL_MOTOR, MotorType.kBrushless);

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
  private Color getCurrentDetectedColor() {
    return colorSensor.getColor();
  }

  /**
   * Get the color in the list of desired matches that matches the detected color
   * the closest
   * 
   * @return the color that matches the detected color the closest from the list
   *         of options
   */
  private ColorMatchResult getClosestColorMatch() {
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
   * Set the motor to a particular value
   * 
   * @param percent percent to set the motor to
   */
  public void set(double percent) {
    colorRotator.set(percent);
  }

  /**
   * Get the confidence that the colormatcher feels that it has regarding the
   * color match
   * 
   * @return a double, where 0 <= double < 1, that represents how confident the
   *         colormatcher is with its choice
   */
  public double getConfidence() {
    return getClosestColorMatch().confidence;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
