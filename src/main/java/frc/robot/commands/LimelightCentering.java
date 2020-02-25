/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class LimelightCentering extends CommandBase {
  private final Drivetrain drivetrain;
  private final Limelight limelight;
  private final Notifier notifier;

  /**
   * Creates a new LimelightCentring.
   */
  public LimelightCentering(Drivetrain drivetrain, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;

    // Limelight not a requirement because it is fine if multiple commands use it
    this.limelight = limelight;

    this.notifier = new Notifier(this::updatePID);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Run the update method based on the given period
    notifier.startPeriodic(Constants.DrivetrainConstants.TURN_PID_UPDATE_PERIOD_SEC);

    // Stop drivetrain and enable PID controller
    drivetrain.stop();
    drivetrain.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain and PID controller
    drivetrain.stop();
    drivetrain.disable();

    // Stop the notifier from updating again
    notifier.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO: add deadband sensing so that command will finish
    return false;
  }

  /**
   * Updates the setpoint of the drivetrain's angle PID loop based on the angle
   * from the target as determined by the Limelight and the drivetrain's current
   * angle.
   * 
   * <p>
   * Note that this method <b>does not</b> check that the angle PID is enabled or
   * that the Limelight's LEDs are currently on, even though this method will not
   * be able to do anything if these are not both set on.
   * 
   * @see Drivetrain#enable()
   */
  private void updatePID() {
    // Get the difference between centre and vision target (error)
    var angleError = limelight.getHorizontalAngleOffset();
    SmartDashboard.putNumber("Angle Error", angleError);

    // Calculate the desired angle using the error and current angle
    var currentHeading = drivetrain.getHeading180();
    double desiredAngle = currentHeading - angleError;
    SmartDashboard.putNumber("Current Angle", currentHeading);
    SmartDashboard.putNumber("Desired Angle", desiredAngle);

    // Set the angle PID controller to the desired angle
    drivetrain.setSetpoint(desiredAngle);
  }
}
