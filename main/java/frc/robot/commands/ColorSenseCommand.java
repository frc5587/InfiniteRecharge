package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSense;

public class ColorSenseCommand extends CommandBase {
  private final ColorSense m_colorsubsys;

  public ColorSenseCommand(ColorSense subsystem) {
    m_colorsubsys = subsystem;
    addRequirements(subsystem);
  }

  public void initialize() {
    m_colorsubsys.periodic();
  }
}