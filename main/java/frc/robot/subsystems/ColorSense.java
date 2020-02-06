/*For the control panel 
  Using color sensor [RevRobotics ColorSensor V3]
*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ColorSense extends SubsystemBase {
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  private final ColorMatch m_colorMatcher = new ColorMatch();

  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  public ColorSense() {
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
  }

  @Override
  public void periodic() {
    Color detectedColor = m_colorSensor.getColor();

    String colorString;
    int colorID = 0;

    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
      colorID = 1;
    } else if (match.color == kRedTarget) {
      colorString = "Red";
      colorID = 2;
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
      colorID = 3;
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
      colorID = 4;
    } else {
      colorString = "Unknown";
      colorID = 0;
    }

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);

    double rotations = 0;

    int colorIDHolder = colorID;
    if (colorID > colorIDHolder && colorIDHolder <= 4) {
      rotations += 0.125;
    }
    if (colorID < colorIDHolder && colorIDHolder >= 1) {
      rotations -= 0.125;
    }
    if (colorID > 4) {
     colorID = 1;
    }
    if (colorID < 1) {
      colorID = 4;
    }
    SmartDashboard.putNumber("Rotations", rotations);
  }
}