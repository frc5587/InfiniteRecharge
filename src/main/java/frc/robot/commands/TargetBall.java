/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.MachineLearning;

public class TargetBall extends CommandBase {
  private final Drivetrain drivetrain;
  private final MachineLearning machineLearning;

  /**
   * Creates a new TargetBall.
   */
  public TargetBall(Drivetrain drivetrain, MachineLearning machineLearning) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    // machineLearning not required because it's just for data

    this.drivetrain = drivetrain;
    this.machineLearning = machineLearning;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Fetch the distance to the nearest
    var closestBox = machineLearning.getClosestBox();
    var transform = closestBox.getTransform();

    // Fetch the robot's pose from when the image was taken
    var captureTime = machineLearning.getImageCaptureFPGATime();
    var capturePose = drivetrain.getClosestPoseAtTime(captureTime);

    // Adjsut the desired pose
    var adjustedPose = capturePose.plus(transform);
    System.out.println(adjustedPose);

    // Log data to SmartDashboard for testing
    SmartDashboard.putString("Delta", transform.toString());
    SmartDashboard.putString("Current", drivetrain.getPose().toString());
    SmartDashboard.putString("Adjusted", adjustedPose.toString());

    // TODO: Generate and follow path
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // TODO: cancel path following and stop motors
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO: detect if path is finisehd
    return true;
  }
}
