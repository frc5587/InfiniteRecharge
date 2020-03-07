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
  private final TalonSRX conveyorBeltMotor = new TalonSRX(IntakeConstants.CONVEYOR_MOTOR);
  // private final DigitalInput bottomLimit = new DigitalInput(IntakeConstants.BOTTOM_LIMIT);
  // private final DigitalInput topLimit = new DigitalInput(IntakeConstants.TOP_LIMIT);
  private int currentNumberOfBalls = 0;
  // private boolean previousSettingOfBottomSwitch = bottomLimit.get();
  // private boolean previousSettingOfTopSwitch = topLimit.get();
  // private boolean shoot = false;
  private final TalonSRX intakeTalon = new TalonSRX(IntakeConstants.INTAKE_MOTOR);

  
  public Intake() {
    intakeTalon.setInverted(true);
  }

  /**
   * Moves the intake forward
   */
  public void moveIntakeForward() {
    intakeTalon.set(ControlMode.PercentOutput, IntakeConstants.THROTTLE);
  }

  /**
   * Moves the intake backward
   */
  public void moveIntakeBackward() {
    intakeTalon.set(ControlMode.PercentOutput, -IntakeConstants.THROTTLE);
  }

  /**
   * Stops all movement of the intake
   */
  public void stopIntakeMovement() {
    intakeTalon.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
 
  }
}
