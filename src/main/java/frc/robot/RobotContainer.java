/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.frc5587.lib.control.DeadbandXboxController;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.subsystems.Conveyor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;


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
  // private final Arm m_arm = new Arm();
  // private final Conveyor conveyor = new Conveyor();
  // private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
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
    var xButton = new JoystickButton(xb, XboxController.Button.kX.value);
    var yButton = new JoystickButton(xb, XboxController.Button.kY.value);
    var leftBumper = new JoystickButton(xb, XboxController.Button.kBumperLeft.value);
    var rightBumper = new JoystickButton(xb, XboxController.Button.kBumperRight.value);
    var rightTrigger = new Trigger(() -> xb.getTriggerAxis(Hand.kRight) > .2);

    // xButton.whenPressed(() -> intake.set(1), intake).whenReleased(() -> intake.set(0), intake);
    // yButton.whenPressed(() -> intake.set(-1), intake).whenReleased(() -> intake.set(0), intake);
    // leftBumper.whenPressed(conveyor::moveBackward).whenReleased(conveyor::stopMovement);
    // rightBumper.whenPressed(conveyor::moveForward).whenReleased(conveyor::stopMovement);
    // rightTrigger.whileActiveContinuous(() -> shooter.setThrottle(xb.getTriggerAxis(Hand.kRight))).whenInactive(() -> shooter.setThrottle(0));
    xButton.whenActive(() -> shooter.setVelocity(shooter.calculateShooterSpeed(3.658, 30))).whenInactive(() -> shooter.setVelocity(0));
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
