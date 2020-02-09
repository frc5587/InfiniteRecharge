/* package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ColorSense;


public class ColorSenseCommand extends CommandBase {
  private final ColorSense m_colorsubsys;
  public ColorSenseCommand(ColorSense subsystem) {
    int colorIDHolder = subsystem.colorID;
    m_colorsubsys = subsystem;
    addRequirements(subsystem);

    //int newColorID = subsystem.colorID;
    String newColorString = subsystem.colorString;
    double rotations = 0;
    if (subsystem.colorID > colorIDHolder) {
      rotations += 0.125;
    }
    if (subsystem.colorID < colorIDHolder) {
      rotations += 0.125;
    }
    if (subsystem.colorID > 4) {
     subsystem.colorID = 1;
    }
    if (subsystem.colorID < 1) {
      subsystem.colorID = 4;
    }
    
    SmartDashboard.putNumber("Red:", subsystem.detectedColor.red);
    SmartDashboard.putNumber("Green:", subsystem.detectedColor.green);
    SmartDashboard.putNumber("Blue:", subsystem.detectedColor.blue);
    SmartDashboard.putNumber("Confidence:", subsystem.match.confidence);
    SmartDashboard.putString("Detected Color:", subsystem.colorString);
    SmartDashboard.putNumber("Rotations:", rotations);
  }
} */