/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Limelight;

public class AutoAim extends CommandBase {
  private Arm arm;
  private Limelight limelight;
  private Timer timer = new Timer();

  /**
   * Creates a new AutoAim.
   */
  public AutoAim(Arm arm, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.limelight = limelight;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.advanceIfElapsed(LimelightConstants.UPDATE_PERIOD)) {
      if (limelight.isTargetDetected()) {
        // Only find the angle based on the Limelight's data when a target is detected
        var desiredAngle = limelight.getShooterFrontGoalAngle(arm.getAngleRadians());
        arm.setArmAngleRadians(desiredAngle);
      } else {
        // Default to the last setpoint if there is nothing currently detected
        arm.setArmAngleTicks(arm.getLastSetpoint());
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}