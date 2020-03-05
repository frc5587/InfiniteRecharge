/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

public class ConveyorStepBack extends CommandBase {
  private Conveyor conveyor;
  private Intake intake;
  private Timer timer;

  /**
   * Creates a new ConveyorStepBack.
   */
  public ConveyorStepBack(Conveyor conveyor, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(conveyor, intake);

    this.conveyor = conveyor;
    this.intake = intake;
    this.timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    conveyor.moveConveyorBackward();
    intake.moveIntakeBackward();
    timer.start();
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(0.25);
  }
}
