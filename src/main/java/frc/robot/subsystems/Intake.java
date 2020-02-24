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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * this subsystem controls the collection and movement of balls for shooting.
 */
public class Intake extends SubsystemBase {
  
  private final DigitalInput bottomLimit = new DigitalInput(Constants.IntakeConstants.BOTTOM_LIMIT);
  private final DigitalInput topLimit = new DigitalInput(Constants.IntakeConstants.TOP_LIMIT);
  private int currentNumberOfBalls = 3;
  private boolean previousSettingOfBottomSwitch = bottomLimit.get();
  private boolean previousSettingOfTopSwitch = topLimit.get();
  private boolean shoot = false;
  private final TalonSRX intakeTalon = new TalonSRX(Constants.IntakeConstants.INTAKE_MOTOR);
  private final TalonSRX centeringTalon = new TalonSRX(Constants.IntakeConstants.CENTERING_MOTOR);

  /**
   * Creates a new Conveyor.
   */
  public Intake() {
    intakeTalon.setInverted(true);
  }
  /**
   * Moves the intake forward
   */
  public void moveIntakeForward(){
    intakeTalon.set(ControlMode.PercentOutput, Constants.IntakeConstants.THROTTLE);
    centeringTalon.set(ControlMode.PercentOutput,  Constants.IntakeConstants.THROTTLE/2);
  }
  /**
   * Moves the intake backward
   */
  public void moveIntakeBackward(){
    intakeTalon.set(ControlMode.PercentOutput, -Constants.IntakeConstants.THROTTLE);
    centeringTalon.set(ControlMode.PercentOutput, -Constants.IntakeConstants.THROTTLE/2);
    }
  /**
   * Stops all movement of the intake
   */
  public void stopIntakeMovement(){
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
    // updates the number of balls. Only works if limit switch was not previously
    // enabled.
    if (bottomLimit.get() && bottomLimit.get() != previousSettingOfBottomSwitch && currentNumberOfBalls <= 5) {
      currentNumberOfBalls += 1;
    }
    if (topLimit.get() && topLimit.get() != previousSettingOfTopSwitch && currentNumberOfBalls > 0) {
      currentNumberOfBalls -= 1;
    }
    // communicates to drive team about when to shoot.
    if (currentNumberOfBalls >= 5) {
      shoot = true;
      SmartDashboard.putBoolean("shoot?", shoot);
    } else {
      shoot = false;
      SmartDashboard.putBoolean("shoot?", shoot);
    }
    // this is done to update the value of the limit switch in order to establish
    // previous and current state of limit switch
    previousSettingOfBottomSwitch = bottomLimit.get();
    previousSettingOfTopSwitch = topLimit.get();
    SmartDashboard.putNumber("number_of_balls", currentNumberOfBalls);
  }
}
