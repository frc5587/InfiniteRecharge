/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.ColorSensor.TargetColor;
import frc.robot.subsystems.Conveyor;

public class SpinToColor extends CommandBase {
  private ColorSensor colorSensor;
  private Conveyor conveyor;
  private TargetColor targetColor;

  /**
   * Creates a new SpinToColor.
   */
  public SpinToColor(ColorSensor colorSensor ,Conveyor conveyor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.colorSensor = colorSensor;
    this.conveyor = conveyor;
    addRequirements(colorSensor, conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    conveyor.set();
    targetColor = colorSensor.getTargetColor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.stopSet();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return targetColor != colorSensor.getTargetColor();
  }
}
