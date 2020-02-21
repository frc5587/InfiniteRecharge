/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.frc5587.lib.control.DeadbandXboxController;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeStopper;
import frc.robot.commands.ManualArmControl;
import frc.robot.commands.RotateControlPanel;
import frc.robot.commands.SpinToColor;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorSensor;
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
  public final Climber climber = new Climber();
  private final Arm m_arm = new Arm();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final ColorSensor colorSensor = new ColorSensor();

  private final Joystick joy = new Joystick(0);
  private final DeadbandXboxController xb = new DeadbandXboxController(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // shooter.setDefaultCommand(new Shoot(shooter, joy::getY));
    intake.setDefaultCommand(new IntakeStopper(intake));

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
    var upDPad = new POVButton(xb, 0);
    var leftTrigger = new Trigger(() -> xb.getTrigger(Hand.kLeft));
    var rightJoy = new Trigger(() -> xb.getY(Hand.kRight) != 0);
    var xButton = new JoystickButton(xb, XboxController.Button.kX.value);
    var yButton = new JoystickButton(xb, XboxController.Button.kY.value);
    var armLimitSwitch = new Trigger(() -> m_arm.getLimitSwitchVal());
    var leftBumper = new JoystickButton(xb, XboxController.Button.kBumperLeft.value);
    var rightBumper = new JoystickButton(xb, XboxController.Button.kBumperRight.value);
    var backButton = new JoystickButton(xb, XboxController.Button.kBack.value);

    // Intake
    rightBumper.whileHeld(() -> {
      intake.moveIntakeForward();
      intake.moveConveyorForward();
    }, intake).whenReleased(() -> {
      intake.stopConveyorMovement();
      intake.stopIntakeMovement();
    });
    leftBumper.whileHeld(() -> {
      intake.moveConveyorBackward();
      intake.moveIntakeBackward();
    }).whenReleased(() -> {
      intake.stopConveyorMovement();
      intake.stopIntakeMovement();
    });
    SmartDashboard.putData("Ball Count Reset", new InstantCommand(intake::reset));

    // arm
    // determines whether the arm should be manually controlled
    leftTrigger.and(rightJoy).whileActiveContinuous(new ManualArmControl(m_arm, () -> xb.getY(Hand.kRight)));

    // moves arm to the lowest and highest positions
    xButton.whenPressed(() -> m_arm.setArmAngleDegrees(14), m_arm);
    yButton.whenPressed(() -> m_arm.setArmAngleDegrees(55), m_arm);

    // reset elevator encoder
    armLimitSwitch.whenActive(m_arm::resetEncoder);

    upDPad.whenActive(() -> climber.set(0.5), climber).whenInactive(() -> climber.set(0), climber);

    // Control panel
    backButton.whenPressed(new RotateControlPanel(colorSensor));
    leftTrigger.and(backButton).whenActive(new SpinToColor(colorSensor));
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