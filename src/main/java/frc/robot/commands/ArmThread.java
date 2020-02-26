package frc.robot.commands;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmThread extends CommandBase {
  private Limelight limelight;
  private Arm arm;
  private Notifier notifier = new Notifier(this::updateArm);
  private double lastAngle = 30;

  /**
   * Creates a commands that schedules a thread for updating the arm based 
   * on the position that the limelight determines that the robot is in
   * 
   * @param arm arm subsystem
   * @param limelight limelight subsystem
   */
  public ArmThread(Arm arm, Limelight limelight) {
    this.limelight = limelight;
    this.arm = arm;
    addRequirements(arm);
  }
  
  public void updateArm() {
    // angle of the arm
    double armAngleRad = arm.getAngleRadians();

    double heightOfWorkingTarget = Constants.LimelightConstants.GOAL_HEIGHT_METERS - limelight.getShooterHeight(armAngleRad);

    double distance = limelight.getShooterGoalHorizontalDifference(armAngleRad, Limelight.Target.FRONT);

    double angleToSetDegrees = limelight.isTargetDetected() ? calcArmAngleDegrees(distance, heightOfWorkingTarget) : this.lastAngle;
    
    this.lastAngle = angleToSetDegrees;

    SmartDashboard.putNumber("Setting Arm Angle - Thread", angleToSetDegrees);
    // System.out.println("Setting angle to " + angleToSetDegrees + " degrees");
    arm.setArmAngleDegrees(angleToSetDegrees);
  }

  /**
   * Calculates the angle to set the arm base on the distance the robot is away from the power port
   * This calculates the optimum distance
   * 
   * @param distanceMeters
   * @return
   */
  public static double calcArmAngleDegrees(double distanceMeters, double height) {
    return Math.toDegrees(Math.atan(2 * (height) / distanceMeters));
  }

  @Override
  public void initialize() {
    notifier.startPeriodic(Constants.LimelightConstants.THREAD_PERIOD_TIME_SECONDS);
  }

  @Override
  public void end(boolean interrupted) {
    notifier.stop();
    System.out.println("ArmThread ending - interrupted: " + interrupted);
  }
}