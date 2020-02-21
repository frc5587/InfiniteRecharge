/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ColorSensor;

public class SpinToColor extends CommandBase {
  private ColorSensor colorSensor;
  private char desiredColor;

  /**
   * Creates a new SpinToColor.
   */
  public SpinToColor(ColorSensor colorSensor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.colorSensor = colorSensor;
    addRequirements(colorSensor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    String fmsData = DriverStation.getInstance().getGameSpecificMessage();
    if (fmsData.length() > 0) {
      desiredColor = fmsData.charAt(0);
      colorSensor.set(0.5);
    } else {
      desiredColor = '0';
      DriverStation.reportError("No game data detected", false);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    colorSensor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return desiredColor == '0' || desiredColor == colorSensor.getTargetColor().toString().charAt(0);
  }
}
