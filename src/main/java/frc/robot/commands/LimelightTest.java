/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Limelight;

public class LimelightTest extends CommandBase {
  private Limelight limelight;
  private Arm arm;
  /**
   * Creates a new LimelightTest.
   */
  public LimelightTest(Limelight limelight, Arm arm) {
    this.limelight = limelight;
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber("Limelight Distance", Units.metersToInches(limelight.getLimelightDistance(Math.toRadians(arm.getAngleDegrees()))));
    // SmartDashboard.putNumber("Shooter Distance", Units.metersToInches(limelight.getShooterDistance(Math.toRadians(arm.getAngleDegrees()))));
    // System.out.println(limelight.getShooterHeight(Math.toRadians(arm.getAngleDegrees())));
    // System.out.println(limelight.getLimelightHeight(Math.toRadians(arm.getAngleDegrees())));

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
