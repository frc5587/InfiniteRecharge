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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.commands.IntakeStopper;;

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

  private final Intake intake = new Intake();
  private final Conveyor conveyor = new Conveyor();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    intake.setDefaultCommand(new IntakeStopper(intake, conveyor));
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /**
     * Binds the ability to move the intake and conveyor to the right and left bumpers
     */
    var rightBumper = new JoystickButton(xb, XboxController.Button.kBumperRight.value);
    rightBumper.whileHeld(() -> {
      intake.moveIntakeForward();
      conveyor.moveConveyorForward();
    }, intake).whenReleased(() -> {
      conveyor.stopConveyorMovement();
      intake.stopIntakeMovement();
    });

    var leftBumper = new JoystickButton(xb, XboxController.Button.kBumperLeft.value);
    leftBumper.whileHeld(() -> {
      conveyor.moveConveyorBackward();
      intake.moveIntakeBackward();
    }).whenReleased(() -> {
      conveyor.stopConveyorMovement();
      intake.stopIntakeMovement();
    });

    SmartDashboard.putData("Ball Reset", new InstantCommand(conveyor::reset));

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
