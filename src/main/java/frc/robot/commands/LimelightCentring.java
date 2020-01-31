/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class LimelightCentring extends CommandBase {
  private static final NetworkTableEntry HORIZONTAL_ANGLE_DEG = NetworkTableInstance.getDefault().getTable("limelight")
      .getEntry("tx");
  private static final NetworkTableEntry LED_MODE = NetworkTableInstance.getDefault().getTable("limelight")
      .getEntry("ledMode");
  private static boolean trackingTarget = false;

  private final Drivetrain drivetrain;
  private final ScheduledExecutorService scheduledExecutorService;

  /**
   * Creates a new LimelightCentring.
   */
  public LimelightCentring(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
    this.scheduledExecutorService = Executors.newSingleThreadScheduledExecutor();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Run the update method every 10ms
    scheduledExecutorService.scheduleAtFixedRate(this::updatePID, 0, 10, TimeUnit.MILLISECONDS);

    drivetrain.arcadeDrive(0, 0);
    drivetrain.enable();
    enableLEDs(true);
    trackingTarget = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0);
    drivetrain.disable();
    enableLEDs(false);
    trackingTarget = false;

    scheduledExecutorService.shutdown();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Shouldn't ever return true, but just in case of an error:
    return scheduledExecutorService.isTerminated();
  }

  private void updatePID() {
    var newError = HORIZONTAL_ANGLE_DEG.getDouble(0);
    var currentHeading = drivetrain.getHeading(180.0);
    double desiredAngle = currentHeading + newError;
    drivetrain.setSetpoint(desiredAngle);
  }

  public static void enableLEDs(boolean enabled) {
    if (!trackingTarget) {
      if (enabled) {
        LED_MODE.setNumber(3);
      } else {
        LED_MODE.setNumber(1);
      }
    }
  }
}
