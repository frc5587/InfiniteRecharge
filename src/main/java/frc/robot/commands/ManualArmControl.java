/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ManualArmControl extends CommandBase {
  private final ArmSubsystem arm;
  private final DoubleSupplier throttleSupplier;

  /**
   * Creates a new ManualArmControl.
   */
  public ManualArmControl(ArmSubsystem arm, DoubleSupplier throttleSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);

    this.arm = arm;
    this.throttleSupplier = throttleSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.disable();
    arm.setThrottle(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setThrottle(throttleSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setThrottle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
