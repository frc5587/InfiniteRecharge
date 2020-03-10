/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import org.frc5587.lib.control.DeadbandXboxController;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Conveyor;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ArmThread;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.FindTarget;
import frc.robot.commands.LimelightCentering;
import frc.robot.commands.ManualArmControl;
import frc.robot.commands.RamseteCommandWrapper;
import frc.robot.commands.ResetEncoder;
import frc.robot.commands.ShooterThread;
import frc.robot.commands.TargetBall;
import frc.robot.commands.RamseteCommandWrapper.AutoPaths;
import frc.robot.commands.autonomous.CloseWall;
import frc.robot.commands.autonomous.FrontOfGoal;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.MachineLearning;
import frc.robot.subsystems.ShooterJRAD;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Drivetrain drivetrain = new Drivetrain();
  private final Limelight limelight = new Limelight();
  private final Climber climber = new Climber();
  private final Arm arm = new Arm();
  private final Intake intake = new Intake();
  // private final Shooter shooter = new Shooter();
  private final ShooterJRAD shooter = new ShooterJRAD();

  private final Joystick joy = new Joystick(0);
  private final DeadbandXboxController xb = new DeadbandXboxController(1);

  private final Conveyor conveyor = new Conveyor();
  private final LimelightCentering centeringCommand = new LimelightCentering(drivetrain, limelight);

  private final ArmThread armThread = new ArmThread(arm, limelight);
  private final ShooterThread shooterThread = new ShooterThread(arm, shooter, limelight, conveyor);
  private final ResetEncoder resetEncoder = new ResetEncoder(arm);
  private final FindTarget findTarget = new FindTarget(arm, limelight);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, /*joy::getY,*/ () -> joy.getY(), () -> -joy.getX()));

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
    var buttonTwelve = new JoystickButton(joy, 12);
    var upDPad = new POVButton(xb, 0);
    var leftTrigger = new Trigger(() -> xb.getTrigger(Hand.kLeft));
    var rightTrigger = new Trigger(() -> xb.getTrigger(Hand.kLeft));
    var rightJoy = new Trigger(() -> xb.getY(Hand.kRight) != 0);
    var xButton = new JoystickButton(xb, XboxController.Button.kX.value);
    var yButton = new JoystickButton(xb, XboxController.Button.kY.value);
    var armLimitSwitch = new Trigger(() -> arm.getLimitSwitchVal());
    var leftBumper = new JoystickButton(xb, XboxController.Button.kBumperLeft.value);
    var rightBumper = new JoystickButton(xb, XboxController.Button.kBumperRight.value);
    var aButton = new JoystickButton(xb, XboxController.Button.kA.value);

    var leftStickButton = new JoystickButton(xb, XboxController.Button.kStickLeft.value);
    var rightStickButton = new JoystickButton(xb, XboxController.Button.kStickRight.value);

    // Intake
    rightBumper.whileHeld(() -> {
        intake.moveIntakeForward();
        conveyor.moveConveyorForward();
    }, intake).whenReleased(() -> {
        conveyor.stopConveyorMovement();
        intake.stopIntakeMovement();
    });
    leftBumper.whileHeld(() -> {
        conveyor.moveConveyorBackward();
        intake.moveIntakeBackward();
    }).whenReleased(() -> {
        conveyor.stopConveyorMovement();
        intake.stopIntakeMovement();
    });

    // arm
    // determines whether the arm should be manually controlled
    leftTrigger.and(rightJoy).whileActiveContinuous(new ManualArmControl(arm, () -> xb.getY(Hand.kRight)));

    // reset elevator encoder
    armLimitSwitch.whenActive(arm::resetEncoder);

    // Run climber up
    upDPad.whenActive(() -> climber.set(0.5), climber).whenInactive(() -> climber.set(0), climber);

    buttonTwelve
      .whenPressed(centeringCommand)
      .whenReleased(() -> centeringCommand.cancel());
      
    // buttonEleven.whenPressed(new TargetBall(drivetrain, machineLearning));

    rightTrigger
      .whileActiveContinuous(() -> shooter.setThrottle(xb.getTriggerAxis(Hand.kRight)))
      .whenInactive(() -> shooter.setThrottle(0));

    leftStickButton.whenHeld(armThread);
    // rightStickButton.whileHeld(() -> shooter.setVelocity(1600), shooter).whenReleased(() -> shooter.setVelocity(0), shooter);
    rightStickButton.whileHeld(shooterThread);

    // Stops the armThread from running
    // leftTrigger.whenActive(() -> {}, arm);

    aButton.whenHeld(findTarget);
    xButton.whileHeld(resetEncoder);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
    //     new SimpleMotorFeedforward(DrivetrainConstants.KS_VOLTS, DrivetrainConstants.KV_VOLT_SECONDS_PER_METER,
    //         DrivetrainConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
    //     DrivetrainConstants.DRIVETRAIN_KINEMATICS, 10);

    // // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(AutoConstants.MAX_VELOCITY_METERS_PER_SECOND,
    //     AutoConstants.MAX_ACCEL_METERS_PER_SECOND_SQUARED)
    //         // Add kinematics to ensure max speed is actually obeyed
    //         .setKinematics(DrivetrainConstants.DRIVETRAIN_KINEMATICS)
    //         // Apply the voltage constraint
    //         .addConstraint(autoVoltageConstraint);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     // Pass config
    //     config);

    // // return new RamseteCommandWrapper(drivetrain, exampleTrajectory);
    // return new RamseteCommandWrapper(drivetrain, AutoPaths.SuperCoolPath);

    CloseWall closeWall = new CloseWall(arm, conveyor, drivetrain, intake, limelight, shooter);
    return closeWall;
  }
}