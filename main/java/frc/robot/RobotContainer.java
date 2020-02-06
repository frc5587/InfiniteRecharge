package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ColorSense;
import frc.robot.commands.ColorSenseCommand;

public class RobotContainer {
  private ColorSense colors = new ColorSense();

  public RobotContainer() {
    colors.setDefaultCommand(new ColorSenseCommand(colors));
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    final XboxController xb = new XboxController(0);
    final var aButton = new JoystickButton(xb, XboxController.Button.kA.value);
    final var bButton = new JoystickButton(xb, XboxController.Button.kB.value);
    final var xButton = new JoystickButton(xb, XboxController.Button.kX.value);
    final var yButton = new JoystickButton(xb, XboxController.Button.kY.value);
    aButton.whenActive(new PrintCommand("A button working"));
    bButton.whenActive(new PrintCommand("B button working!"));
    xButton.whenActive(new PrintCommand("X button working!"));
    yButton.whenActive(new PrintCommand("Y button working!"));
  }
}