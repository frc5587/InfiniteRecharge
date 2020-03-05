package frc.robot.commands;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterJRAD;
import frc.robot.subsystems.Conveyor;

public class ShooterThread extends CommandBase {
  private Arm arm;
  private ShooterJRAD shooter;
  private Limelight limelight;
  private Conveyor conveyor;
  private Notifier notifier = new Notifier(this::updateShooter);
  private boolean moveConveyor = false;
  // private int startBalls;

  public ShooterThread(Arm arm, ShooterJRAD shooter, Limelight limelight, Conveyor conveyor) {
    this.arm = arm;
    this.shooter = shooter;
    this.limelight = limelight;
    this.conveyor = conveyor;

    // this.startBalls = this.conveyor.getCurrentNumberOfBalls();

    addRequirements(this.shooter, this.conveyor);
  }

  public void updateShooter() {
    // TODO: adust coefficient
    // 1st power: 2.58
    // 2nd power: N/A
    // 1.3 power: 
    // double speedRPM = 2.58 * limelight.calculateShooterSpeed(arm.getAngleRadians(), Limelight.Target.FRONT) * Math.pow(arm.getAngleRadians(), 1);

    double speedRPM = limelight.calculateShooterSpeed(arm.getAngleRadians(), Limelight.Target.FRONT);

    SmartDashboard.putNumber("Actual Shooter Speed", shooter.getShooterSpeed());
    SmartDashboard.putNumber("Shooter Setpoint", speedRPM);
    shooter.setVelocity(speedRPM);

    if (shooter.atSetpoint()) {
      moveConveyor = true;
    } 
    if (moveConveyor) {
      conveyor.moveConveyorForward();
    }
  }

  @Override
  public void initialize() {
    shooter.enable();
    moveConveyor = false;
    notifier.startPeriodic(Constants.LimelightConstants.THREAD_PERIOD_TIME_SECONDS);
  }

  @Override
  public void end(boolean interrupted) {
    notifier.stop();
    conveyor.stopConveyorMovement();
    shooter.disable();
    shooter.setThrottle(0);
  }

  @Override
  public boolean isFinished() {
    return false;
    // return conveyor.getCurrentNumberOfBalls() == 0 && startBalls != 0;
  }
}