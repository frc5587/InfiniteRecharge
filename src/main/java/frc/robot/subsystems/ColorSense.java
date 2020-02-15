package frc.robot.subsystems;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CPConstants;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DriverStation;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.CANSparkMax;

public class ColorSense extends SubsystemBase {
  //make a color sensor and a color matcher
  private final ColorSensorV3 colorSensor = new ColorSensorV3(CPConstants.i2cPort);
  private final ColorMatch colorMatcher = new ColorMatch();
  //store the color initially detected
  private Color detectedColor = colorSensor.getColor();
  private ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
  //store the values of what targets are called
  public enum ColorEnum {RED, GREEN, BLUE, YELLOW}
  //make variables for checking rotations
  private static int colorID = 1;
  private static int colorChecker = colorID;
  private int rotations;
  //make a variable to check the fms for various data
  private String FMSData;

  public ColorSense() { 
    //add some colors for the color matcher to reference
    colorMatcher.addColorMatch(CPConstants.kBlueTarget);
    colorMatcher.addColorMatch(CPConstants.kGreenTarget);
    colorMatcher.addColorMatch(CPConstants.kRedTarget);
    colorMatcher.addColorMatch(CPConstants.kYellowTarget);
  }

  @Override
  public void periodic() {}

  public String getCurrentColor(){
    //get the detected color and match it to one of the targets
    Color detectedColor = colorSensor.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
    //change the enum's value based on the detected color, if it's disconnected return unknown
    if(match.color == CPConstants.kRedTarget){
      return ColorEnum.RED.toString();
    }
    if(match.color == CPConstants.kGreenTarget){
      return ColorEnum.GREEN.toString();
    }
    if(match.color == CPConstants.kBlueTarget){
      return ColorEnum.BLUE.toString();
    }
    if(match.color == CPConstants.kYellowTarget){
      return ColorEnum.YELLOW.toString();
    }
    else {
      return "UNKNOWN";
    }
  }
  public double getConfidence() {
    return match.confidence;
  }
  public int getRotations() {
    switch (getCurrentColor()){
      case "RED":
        colorID = 1;
      case "GREEN":
        colorID = 2;
      case "BLUE":
        colorID = 3;
      case "YELLOW":
        colorID = 4;
    }
    if(colorChecker != colorID) {
      rotations += 1;
    }
    colorChecker = colorID;
    return rotations;
  }
  public String getFMSColor() {
    FMSData = DriverStation.getInstance().getGameSpecificMessage();
    if(FMSData.length() > 0) {
      switch(FMSData.charAt(0)){
        case 'R':
          return "Red";
        case 'G':
          return "Green";
        case 'B':
          return "Blue";
        case 'Y':
          return "Yellow";
        default:
          return "Unavailable";
      } 
    }
    else {
      return "Unavailable";
    }
  }
}