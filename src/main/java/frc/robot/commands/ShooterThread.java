package frc.robot.commands;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Conveyor;

public class ShooterThread extends CommandBase {
  private Arm arm;
  private Shooter shooter;
  private Limelight limelight;
  private Conveyor conveyor;
  private Notifier notifier = new Notifier(this::updateShooter);
  private int startBalls;

  public ShooterThread(Arm arm, Shooter shooter, Limelight limelight, Conveyor conveyor) {
    this.arm = arm;
    this.shooter = shooter;
    this.limelight = limelight;
    this.conveyor = conveyor;

    this.startBalls = this.conveyor.getCurrentNumberOfBalls();

    addRequirements(this.shooter, this.conveyor);
  }

  public void updateShooter() {
    double speedRPM = limelight.calculateShooterSpeed(arm.getAngleRadians(), Limelight.Target.FRONT);

    if (!limelight.isTargetDetected()) {
      speedRPM = 0;
      System.out.println("No target detected!");
    }

    SmartDashboard.putNumber("Shooter Speed - Thread", speedRPM);
    shooter.setVelocity(speedRPM);
  }

  @Override
  public void initialize() {
    notifier.startPeriodic(Constants.LimelightConstants.THREAD_PERIOD_TIME_SECONDS);
    
    conveyor.moveConveyorForward();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("ShooterThread ending - interrupted: " + interrupted);
    notifier.stop();
    conveyor.stopConveyorMovement();
    shooter.setThrottle(0);
  }

  @Override
  public boolean isFinished() {
    return conveyor.getCurrentNumberOfBalls() == 0 && startBalls != 0;
  }
}