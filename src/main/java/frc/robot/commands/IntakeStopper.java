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
/**
 * This class records current amount of balls and issues a command in order to stop it when it gets to 5.
 */
public class IntakeStopper extends CommandBase {

  private final Intake intake;
  private final Conveyor conveyor;

  public IntakeStopper(Intake intake, Conveyor conveyor) {
    addRequirements(intake, conveyor);
  
    this.conveyor = conveyor;
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  // Sets the intake to zero if parameter is met.
  @Override
  public void execute() {
    // We can only hold 5 balls, therefore when the sensors detect more than that
    // the robot disables the intake motors.
    if (conveyor.getCurrentNumberOfBalls() >= 5) {
      intake.stopIntakeMovement();
      conveyor.stopConveyorMovement();
    }
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
