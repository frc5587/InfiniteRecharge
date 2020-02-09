package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSense;

public class ColorCommand extends CommandBase {
  private final ColorSense m_colors;
  public ColorCommand(ColorSense subsystem) {
    m_colors = subsystem;
    addRequirements(subsystem);
  }
  @Override
  public void execute() {
    SmartDashboard.putString("Detected Color:", m_colors.getCurrentColor());
    SmartDashboard.putNumber("Confidence:", m_colors.getConfidence());
    SmartDashboard.putNumber("Rotations:", m_colors.getRotations());
    SmartDashboard.putString("FMS Color:", m_colors.getFMSColor());
  }
  @Override
  public boolean isFinished() {
    return false;
  }
}