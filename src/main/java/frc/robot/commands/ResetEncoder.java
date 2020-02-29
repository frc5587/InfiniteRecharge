package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ResetEncoder extends CommandBase {
  private Arm arm;

  /**
   * This command **should** move the arm down at about half speed until it his the limit
   * switch, and then it stops it. It may grind the gears because of the arm's momentum
   * 
   * @param arm arm subsystem
   */
  public ResetEncoder(Arm arm) {
    this.arm = arm;

    addRequirements(arm);
  }

  /**
   * Sends the arm down at half speed
   */
  @Override
  public void initialize() {
    arm.setArm(-0.5);
  }

  /**
   * Ends when it hits the limits switch
   */
  @Override
  public boolean isFinished() {
    return arm.getLimitSwitchVal();
  }

/**
 * Stops the arm
 */
  @Override
  public void end(boolean interrupted) {
    arm.setArm(0);
  }
}