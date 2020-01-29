/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class FollowPath extends CommandBase {
  private final Drivetrain drivetrain;

  private final EncoderFollower leftEncoderFollower, rightEncoderFollower;
  private final Notifier followNotifier;
  private final double followNotifierTimeStep;

  private boolean pathFinished;

  /**
   * Creates a new FollowPath.
   */
  public FollowPath(Drivetrain drivetrain, String pathName) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
    this.pathFinished = false;

    Trajectory trajectory = null;
    try {
      // Read the path from a file and prepare it for reading
      var trajectoryFile = new File(pathName + ".csv");
      trajectory = Pathfinder.readFromCSV(trajectoryFile);
    } catch (IOException e) {
      e.printStackTrace();
    }

    if (trajectory != null) {
      var tankModifier = new TankModifier(trajectory);

      // Create and configure PID for left encoder follower
      this.leftEncoderFollower = new EncoderFollower(tankModifier.getLeftTrajectory());
      var leftPID = DrivetrainConstants.LEFT_PATHFINDER_PIDVA;
      this.leftEncoderFollower.configurePIDVA(leftPID.kP, leftPID.kI, leftPID.kD, leftPID.kV, leftPID.kA);

      // Create and configure PID for right encoder follower
      this.rightEncoderFollower = new EncoderFollower(tankModifier.getRightTrajectory());
      var rightPID = DrivetrainConstants.RIGHT_PATHFINDER_PIDVA;
      this.rightEncoderFollower.configurePIDVA(rightPID.kP, rightPID.kI, rightPID.kD, rightPID.kV, rightPID.kA);

      this.followNotifier = new Notifier(this::followPath);
      this.followNotifierTimeStep = trajectory.get(0).dt;
    } else {
      // TODO: Regenerate paths to handle error (in earlier catch)
      this.leftEncoderFollower = null;
      this.rightEncoderFollower = null;
      this.followNotifier = null;
      this.followNotifierTimeStep = Double.NaN;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (followNotifier != null) {
      this.leftEncoderFollower.configureEncoder((int) drivetrain.getLeftPosition(), DrivetrainConstants.TICKS_PER_REV,
          DrivetrainConstants.WHEEL_DIAMETER);
      this.rightEncoderFollower.configureEncoder((int) drivetrain.getRightPosition(), DrivetrainConstants.TICKS_PER_REV,
          DrivetrainConstants.WHEEL_DIAMETER);

      // Start running a notifier at the rate from the generated trajectory
      this.followNotifier.startPeriodic(followNotifierTimeStep);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    followNotifier.stop();
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathFinished;
  }

  private void followPath() {
    if (leftEncoderFollower.isFinished() || rightEncoderFollower.isFinished()) {
      followNotifier.stop();
      pathFinished = true;
    } else {
      // Calculate left and right speed based on current encoder positions
      var leftSpeed = leftEncoderFollower.calculate((int) drivetrain.getLeftPosition());
      var rightSpeed = rightEncoderFollower.calculate((int) drivetrain.getRightPosition());

      // Use the heading for additional information
      var heading = drivetrain.getHeading();
      var desiredHeading = Pathfinder.r2d(leftEncoderFollower.getHeading());
      var headingError = Pathfinder.boundHalfDegrees(desiredHeading - heading);

      // Set turn based on the turn kP
      var turn = DrivetrainConstants.PATHFINDER_TURN_P * headingError;

      drivetrain.tankLR(leftSpeed + turn, rightSpeed - turn);
    }
  }
}
