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
    this.lastAngle = arm.getAngleDegrees();
    addRequirements(arm);
  }
  
  public void updateArm() {
    // Get angle to set arm, if the limelight hasn't found the target, it just sets it to the previous angle
    double angleToSetDegrees = limelight.isTargetDetected() ? limelight.calculateArmMovement(arm.getAngleRadians(), Limelight.Target.FRONT) : this.lastAngle;  // alcArmAngleDegrees(distance, heightOfWorkingTarget) : this.lastAngle;
    
    // saves last angle
    this.lastAngle = angleToSetDegrees;
    SmartDashboard.putNumber("Horizontal", limelight.getShooterGoalHorizontalDifference(arm.getAngleRadians()));
    

    // debugging
    SmartDashboard.putNumber("Setting Arm Angle - Thread", angleToSetDegrees);
    arm.setArmAngleDegrees(angleToSetDegrees);
  }

  /**
   * Calculates the angle to set the arm base on the distance the robot is away from the power port
   * This calculates the optimum distance
   * 
   * @param distanceMeters
   * @return
   */
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