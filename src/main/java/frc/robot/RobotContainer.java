/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.DoubleSupplier;

import org.frc5587.lib.control.DeadbandXboxController;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ManualArmControl;
import frc.robot.subsystems.Arm;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...

  private final Joystick joy = new Joystick(0);
  private final DeadbandXboxController xb = new DeadbandXboxController(1);

  // private final Conveyor conveyor = new Conveyor();
  // private final Shooter shooter = new Shooter();
  // private final Shoot shoot = new Shoot(shooter, joy::getY);
  private final Arm m_arm = new Arm();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    // shooter.setDefaultCommand(shoot);
    // controlLED;
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    var rightJoy = new Trigger(() -> xb.getY(Hand.kRight) != 0);
    var leftTrigger = new Trigger(() -> xb.getTrigger(Hand.kLeft));
    var xButton = new JoystickButton(xb, XboxController.Button.kX.value);
    var yButton = new JoystickButton(xb, XboxController.Button.kY.value);
    var armLimitSwitch = new Trigger(() -> m_arm.getLimitSwitchVal());

    // arm
    // determines whether the arm should be manually controlled
    leftTrigger.and(rightJoy).whileActiveContinuous(new ManualArmControl(m_arm, () -> xb.getY(Hand.kRight)));

    xButton.whenPressed(() -> {
    m_arm.setArmAngleDegrees(14);
    }, m_arm);

    // moves arm to the highest position
    yButton.whenPressed(() -> {
    m_arm.setArmAngleDegrees(55);
    }, m_arm);

    // reset encoder
    armLimitSwitch.whenActive(m_arm::resetEncoder);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}