/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class RamseteCommandWrapper extends CommandBase {
  private final Drivetrain drivetrain;
  private final Trajectory trajectory;

  private Command pathFollowCommand;

  /**
   * Creates a new RamseteCommandWrapper.
   */
  public RamseteCommandWrapper(Drivetrain drivetrain, AutoPaths path) {
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;

    // Get the path to the trajectory on the RoboRIO's filesystem
    var trajectoryPath = path.getJSONPath();

    // Get the trajectory based on the file path (throws IOException if not found)
    Trajectory trajectory = null;
    try {
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open " + path + " trajectory: " + trajectoryPath, ex.getStackTrace());
    }
    // Now set trajectory (or null, if not found)
    this.trajectory = trajectory;
  }

  public RamseteCommandWrapper(Drivetrain drivetrain, Trajectory trajectory) {
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
    this.trajectory = trajectory;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setIdleMode(IdleMode.kBrake);

    // Start the pathFollowCommand
    if (trajectory != null) {
      // Shift the trajectory to start at to the robot's current position
      var currentPose = drivetrain.getPose();
      var transform = new Transform2d(currentPose.getTranslation(), currentPose.getRotation());
      var shiftedTrajectory = trajectory.transformBy(transform);

      // Create the RamseteCommand based on the drivetrain's constants
      var ramsete = new RamseteCommand(shiftedTrajectory, drivetrain::getPose,
          new RamseteController(AutoConstants.RAMSETE_B, AutoConstants.RAMSETE_ZETA),
          new SimpleMotorFeedforward(DrivetrainConstants.KS_VOLTS, DrivetrainConstants.KV_VOLT_SECONDS_PER_METER,
              DrivetrainConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
          DrivetrainConstants.DRIVETRAIN_KINEMATICS, drivetrain::getWheelSpeeds,
          new PIDController(DrivetrainConstants.RAMSETE_KP_DRIVE_VEL, 0, 0),
          new PIDController(DrivetrainConstants.RAMSETE_KP_DRIVE_VEL, 0, 0),
          // RamseteCommand passes volts to the callback
          drivetrain::tankLRVolts, drivetrain);

      // Run path following command, then stop at the end
      pathFollowCommand = ramsete.andThen(drivetrain::stop, drivetrain);

      pathFollowCommand.schedule();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain and path following command just in case
    if (pathFollowCommand != null) {
      pathFollowCommand.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (pathFollowCommand != null) {
      return !pathFollowCommand.isScheduled();
    } else {
      return true;
    }
  }

  public enum AutoPaths {
    RightStartToPowerPort, BackwardsRightStartToPowerPort, FarSideOfTrenchToLoading, SuperCoolPath, ForwardStop, SCurve, ReverseRightSideToPowerPort, ReverseToRendezvous, SCurveFromRendezvous, ReverseToTrench, ReverseUnderTrench, Back, Forward, Circle;

    /**
     * Get the path to the corresponding path JSON file (generated with PathWeaver)
     * in the roboRIO's filesystem for a given enum value
     * 
     * @return the complete path under the roboRIO's filesystem for the
     *         corresponding path JSON
     */
    public Path getJSONPath() {
      var path = "paths/";
      switch (this) {
        case RightStartToPowerPort:
          path += "Right Start to Power Port.wpilib.json";
          break;
        case BackwardsRightStartToPowerPort:
          path += "Backwards Right Start to Power Port.wpilib.json";
          break;
        case FarSideOfTrenchToLoading:
          path += "Far Side of Trench to Loading.wpilib.json";
          break;
        case SuperCoolPath:
          path += "Super Cool Path.wpilib.json";
          break;
        case ForwardStop:
          path += "Forward Stop.wpilib.json";
          break;
        case SCurve:
          path += "S Curve.wpilib.json";
          break;
        case ReverseRightSideToPowerPort:
          path += "Reverse Right Side To Power Port.wpilib.json";
          break;
        case ReverseToRendezvous:
          path += "Reverse To Rendezvous.wpilib.json";
          break;
        case SCurveFromRendezvous:
          path += "S Curve From Rendezvous.wpilib.json";
          break;
        case ReverseUnderTrench:
          path += "Reverse Under Trench.wpilib.json";
          break;
        case ReverseToTrench:
          path += "Reverse To Trench.wpilib.json";
          break;
        case Back:
          path += "Back.wpilib.json";
          break;
        case Forward:
          path += "Forward.wpilib.json";
          break;
        case Circle:
          path += "Circle.wpilib.json";
      }

      // Join the path with where the code is deployed to on the roboRIO, in order to
      // get the complete path
      return Filesystem.getDeployDirectory().toPath().resolve(path);
    }
  }
}
