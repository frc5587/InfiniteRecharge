/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootUntilEmpty extends CommandBase {
  /**
   * Creates a new CycleShoot.
   */
  private Conveyor conveyor;
  private Shooter shooter;
  private Intake intake;

  public ShootUntilEmpty(Conveyor conveyor, Intake intake, Shooter shooter) {
    this.conveyor = conveyor;
    this.shooter = shooter;
    this.intake = intake;
    addRequirements(conveyor, shooter, intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    conveyor.moveConveyorForward();
    intake.moveIntakeForward();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.stopConveyorMovement();
    intake.stopIntakeMovement();
    shooter.setThrottle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return conveyor.getCurrentNumberOfBalls() == 0;
  }
}
