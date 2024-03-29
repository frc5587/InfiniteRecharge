/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * this subsystem controls the collection and movement of balls for shooting.
 */
public class Intake extends SubsystemBase {
  private final TalonSRX conveyorBeltMotor = new TalonSRX(IntakeConstants.CONVEYOR_MOTOR);
  private final DigitalInput bottomLimit = new DigitalInput(IntakeConstants.BOTTOM_LIMIT);
  private final DigitalInput topLimit = new DigitalInput(IntakeConstants.TOP_LIMIT);
  private int currentNumberOfBalls = 0;
  private boolean previousSettingOfBottomSwitch = bottomLimit.get();
  private boolean previousSettingOfTopSwitch = topLimit.get();
  private boolean shoot = false;
  private final TalonSRX intakeTalon = new TalonSRX(IntakeConstants.INTAKE_MOTOR);
  private final TalonSRX centeringTalon = new TalonSRX(IntakeConstants.CENTERING_MOTOR);

  
  public Intake() {
    intakeTalon.setInverted(true);
    conveyorBeltMotor.setInverted(true);
  }

  /**
   * Moves the conveyer forward
   */
  public void moveConveyorForward() {
    conveyorBeltMotor.set(ControlMode.PercentOutput, IntakeConstants.CONVEYOR_THROTTLE);
  }

  /**
   * Moves the intake forward
   */
  public void moveIntakeForward() {
    intakeTalon.set(ControlMode.PercentOutput, IntakeConstants.THROTTLE);
    centeringTalon.set(ControlMode.PercentOutput, IntakeConstants.THROTTLE / 2);
  }

  /**
   * Moves the conveyer backward
   */
  public void moveConveyorBackward() {
    conveyorBeltMotor.set(ControlMode.PercentOutput, -IntakeConstants.CONVEYOR_THROTTLE);
  }

  /**
   * Moves the intake backward
   */
  public void moveIntakeBackward() {
    intakeTalon.set(ControlMode.PercentOutput, -IntakeConstants.THROTTLE);
    centeringTalon.set(ControlMode.PercentOutput, -IntakeConstants.THROTTLE / 2);
  }

  /**
   * Stops all movement of the conveyer
   */
  public void stopConveyorMovement() {
    conveyorBeltMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Stops all movement of the intake
   */
  public void stopIntakeMovement() {
    intakeTalon.set(ControlMode.PercentOutput, 0);
    centeringTalon.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Method that returns a value of current amount of balls.
   * 
   * @return amount of balls currently in the bot
   */
  public int getCurrentNumberOfBalls() {
    return currentNumberOfBalls;
  }

  /**
   * Method that resets current amount of balls.
   */
  public void reset() {
    currentNumberOfBalls = 0;
  }

  @Override
  public void periodic() {
 
  }
}
