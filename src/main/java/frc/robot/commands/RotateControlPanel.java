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
 * Counts rotations of the control panel wheel and spins the wheel.
 */
public class RotateControlPanel extends CommandBase {
  private ColorSensor colorSensor;
  private String previousColor;
  private double rotations = 0;

  /**
   * Creates a new RotateControlPanel.
   */
  public RotateControlPanel(ColorSensor colorSensor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.colorSensor = colorSensor;
    addRequirements(colorSensor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    colorSensor.set(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    String color = colorSensor.getClosestColorMatchToString();
    if (previousColor != null && !color.equals(previousColor)) {
      rotations += (1.0 / 8);
      SmartDashboard.putNumber("number_of_Rotations", rotations); 
    }
    previousColor = color;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    colorSensor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotations > 4;
  }
}
