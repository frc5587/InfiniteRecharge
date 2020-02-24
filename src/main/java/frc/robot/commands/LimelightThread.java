package frc.robot.commands;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants;

public class LimelightThread extends CommandBase {
  private Limelight limelight;
  private Arm arm;
  private Notifier notifier = new Notifier(this::updateArm);

  /**
   * Creates a commands that schedules a thread for updating the arm based 
   * on the position that the limelight determines that the robot is in
   * 
   * @param arm arm subsystem
   * @param limelight limelight subsystem
   */
  public LimelightThread(Arm arm, Limelight limelight) {
    this.limelight = limelight;
    this.arm = arm;
  }
  
  public void updateArm() {
    double distanceMeters = limelight.getShooterDistance(arm.getAngleRadians());
    double angleToSetDegrees = calcArmAngleDegrees(distanceMeters);
    arm.setArmAngleDegrees(angleToSetDegrees);
  }

  /**
   * Calculates the angle to set the arm base on the distance the robot is away from the power port
   * This calculates the optimum distance
   * 
   * @param distanceMeters
   * @return
   */
  public static double calcArmAngleDegrees(double distanceMeters) {
    return Math.toDegrees(Math.atan(2 * Constants.ShooterConstants.GOAL_HEIGHT / distanceMeters));
  }

  @Override
  public void initialize() {
    notifier.startPeriodic(Constants.LimelightConstants.THREAD_PERIOD_TIME_SECONDS);
  }

  @Override
  public void end(boolean interrupted) {
    notifier.stop();
    System.out.println("LimelightThread ending - interrupted: " + interrupted);
  }
}