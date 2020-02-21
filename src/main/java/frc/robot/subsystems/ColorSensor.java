/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ControlPanelConstants;

/**
 * The subsystem for the color sensor.
 */
public class ColorSensor extends SubsystemBase {
  private ColorSensorV3 colorSensor = new ColorSensorV3(ControlPanelConstants.i2cPort);
  private ColorMatch colorMatcher = new ColorMatch();
  private TalonSRX colorRotator = new TalonSRX(ControlPanelConstants.CONTROL_PANEL_MOTOR);

  public ColorSensor() {
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
   * Convert the closest matching Color to a TargetColor
   * 
   * @return the Color that matches as a TargetColor
   */
  public TargetColor getTargetColor() {
    if (getClosestColorMatch().color == ControlPanelConstants.BLUE_TARGET) {
      return TargetColor.BLUE;
    } else if (getClosestColorMatch().color == ControlPanelConstants.GREEN_TARGET) {
      return TargetColor.GREEN;
    } else if (getClosestColorMatch().color == ControlPanelConstants.RED_TARGET) {
      return TargetColor.RED;
    } else if (getClosestColorMatch().color == ControlPanelConstants.YELLOW_TARGET) {
      return TargetColor.YELLOW;
    } else {
      return TargetColor.UNKNOWN;
    }
  }

  /**
   * Set the motor to a particular value
   * 
   * @param percent percent to set the motor to
   */
  public void set(double percent) {
    colorRotator.set(ControlMode.PercentOutput, percent);
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

  /**
   * Possible colors in the control panel
   */
  public static enum TargetColor {
    BLUE, GREEN, RED, YELLOW, UNKNOWN
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Confidence", getConfidence());
    SmartDashboard.putString("Detected Color", getTargetColor().toString());
  }
}
