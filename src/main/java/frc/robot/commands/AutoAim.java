/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.Target;

public class AutoAim extends CommandBase {
  private Arm arm;
  private Limelight limelight;
  private int counter;
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
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // arm.setArmAngleDegrees(arm.getAngleDegrees() + limelight.getVerticalAngleOffset());
    // arm.setArmAngleDegrees(limelight.getShooterFrontGoalAngle(arm.getAngleDegrees()));
    if (counter % 15 == 0) {
      arm.setArmAngleDegrees(Math.toDegrees(limelight.getShooterFrontGoalAngle(Math.toRadians(arm.getAngleDegrees()))));
    }
    counter++;

    // SmartDashboard.putNumber("Ideal Angle", limelight.getShooterFrontGoalAngle(arm.getAngleDegrees()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}