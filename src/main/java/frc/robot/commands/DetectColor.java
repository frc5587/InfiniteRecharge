/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensor;

/**
 * Detects whatever desired match is closest to the color currently being detected.
 */
public class DetectColor extends CommandBase {
  private ColorSensor colorSensor;
  /**
   * Creates a new DetectColor.
   */
  public DetectColor(ColorSensor colorSensor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.colorSensor = colorSensor;
    addRequirements(colorSensor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    colorSensor.addAllColorMatches();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Confidence", colorSensor.getConfidence());
    SmartDashboard.putString("Detected Color", colorSensor.getClosestColorMatchToString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
