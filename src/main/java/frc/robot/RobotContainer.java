package frc.robot;

import java.io.Console;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ColorSense;
import frc.robot.commands.ColorCommand;

public class RobotContainer {
  private ColorSense colors = new ColorSense();

  private void configureButtonBindings() {
    final XboxController xb = new XboxController(0);
    final var aButton = new JoystickButton(xb, XboxController.Button.kA.value);
    final var bButton = new JoystickButton(xb, XboxController.Button.kB.value);
    final var xButton = new JoystickButton(xb, XboxController.Button.kX.value);
    final var yButton = new JoystickButton(xb, XboxController.Button.kY.value);
    aButton.whenActive(new ColorCommand(colors));
    bButton.whenActive(() -> System.out.println("B button working!"));
    xButton.whenActive(new PrintCommand("X button working!"));
    yButton.whenActive(new PrintCommand("Y button working!"));
  }
  
  public RobotContainer() {
    colors.setDefaultCommand(new ColorCommand(colors));
    configureButtonBindings();
  }
}