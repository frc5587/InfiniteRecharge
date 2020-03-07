package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Limelight;

public class FindTarget extends CommandBase {
  private Arm arm;
  private Limelight limelight;
  private int direction = -1;

  /**
   * A command to move the arm down until it reaches the limit switch, then it moves the arm 
   * up till it get to the upper limit. If at any point, it detects the target, it will end.
   * The goal of this is to find the target if the arm ever loses it.
   * 
   * @param arm the arm subsystem
   * @param limelight the limelight subsystem
   */
  public FindTarget(Arm arm, Limelight limelight) {
    this.arm = arm;
    this.limelight = limelight;

    addRequirements(arm);
  }

  /**
   * Moves the arm at half speed in `direction`, negative is down while positive is up
   */
  @Override
  public void execute() {
    if (arm.getLimitSwitchVal()) {
      direction = 1;
    }

    arm.setArm(0.75 * direction);
  }

  /**
   * True if it detects the target, OR it reaches the upper limit while traveling upwards
   */
  @Override
  public boolean isFinished() {
    return limelight.isTargetDetected() || (arm.getAngleDegrees() >= Constants.ArmConstants.UPPER_LIMIT_DEGREES && direction == 1);
  }

  /**
   * Stops the arm
   */
  @Override
  public void end(boolean interrupted) {
    arm.setArm(0);
  }
}