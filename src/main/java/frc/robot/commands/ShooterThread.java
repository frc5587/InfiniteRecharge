package frc.robot.commands;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
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
  private boolean isAuto;
  private Timer timer = new Timer(); 

  public ShooterThread(Arm arm, ShooterJRAD shooter, Limelight limelight, Conveyor conveyor, boolean isAuto) {
    this.arm = arm;
    this.shooter = shooter;
    this.limelight = limelight;
    this.conveyor = conveyor;
    this.isAuto = isAuto;

    addRequirements(this.shooter, this.conveyor);
  }

  public ShooterThread(Arm arm, ShooterJRAD shooter, Limelight limelight, Conveyor conveyor) {
    this(arm, shooter, limelight, conveyor, false);
  }

  public void updateShooter() {
    limelight.turnOn();

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
    timer.reset();
    timer.start();
    moveConveyor = false;
    shooter.enable();
    conveyor.moveConveyorBackward();
    try {
      TimeUnit.MILLISECONDS.sleep(100);  
    } catch (InterruptedException interruptedException) {}
    conveyor.stopConveyorMovement();

    notifier.startPeriodic(Constants.LimelightConstants.THREAD_PERIOD_TIME_SECONDS);
  }

  @Override
  public void end(boolean interrupted) {
    limelight.turnOff();
    notifier.stop();
    conveyor.stopConveyorMovement();
    shooter.disable();
    shooter.setThrottle(0);
    timer.stop();
    timer.reset();
  }

  @Override
  public boolean isFinished() {
    if (isAuto) {
      return timer.get() >= 5;
    }
    return false;
    
  }
}