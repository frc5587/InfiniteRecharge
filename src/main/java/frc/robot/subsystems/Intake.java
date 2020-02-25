/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * this subsystem controls the collection and movement of balls for shooting.
 */
public class Intake extends SubsystemBase {
  private final TalonSRX intakeTalon = new TalonSRX(IntakeConstants.INTAKE_MOTOR);
  private final TalonSRX centeringTalon = new TalonSRX(IntakeConstants.CENTERING_MOTOR);

  
  public Intake() {
    intakeTalon.setInverted(true);
  }
  /**
   * Moves the intake forward
   */
  public void moveIntakeForward() {
    intakeTalon.set(ControlMode.PercentOutput, IntakeConstants.THROTTLE);
    centeringTalon.set(ControlMode.PercentOutput, IntakeConstants.THROTTLE / 2);
  }
  /**
   * Moves the intake backward
   */
  public void moveIntakeBackward() {
    intakeTalon.set(ControlMode.PercentOutput, -IntakeConstants.THROTTLE);
    centeringTalon.set(ControlMode.PercentOutput, -IntakeConstants.THROTTLE / 2);
  }
  /**
   * Stops all movement of the intake
   */
  public void stopIntakeMovement() {
    intakeTalon.set(ControlMode.PercentOutput, 0);
    centeringTalon.set(ControlMode.PercentOutput, 0);
  }
  @Override
  public void periodic() {
 
  }
}