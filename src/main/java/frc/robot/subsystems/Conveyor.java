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

public class Conveyor extends SubsystemBase {
  private final TalonSRX controlPanelTalon = new TalonSRX(Constants.ConveyorConstants.CONTROL_PANEL_MOTOR);
  private final TalonSRX conveyorBeltMotor = new TalonSRX(Constants.ConveyorConstants.CONVEYOR_MOTOR);
  private final DigitalInput bottomLimit = new DigitalInput(Constants.IntakeConstants.BOTTOM_LIMIT);
  private final DigitalInput topLimit = new DigitalInput(Constants.IntakeConstants.TOP_LIMIT);
  private boolean previousSettingOfBottomSwitch = tomLimit();
  private boolean previousSettingOfTopSwitch = timLimit();
  private boolean shoot = false;
  private int currentNumberOfBalls = 3;
  /**
   * Creates a new Conveyor.
   */
  public Conveyor() {
    conveyorBeltMotor.setInverted(true); 
    controlPanelTalon.setInverted(true);
  }
    /**
   * Moves the conveyer forward
   */
  public void moveConveyorForward( ){
    conveyorBeltMotor.set(ControlMode.PercentOutput, Constants.ConveyorConstants.CONVEYOR_THROTTLE);
    controlPanelTalon.set(ControlMode.PercentOutput, Constants.ConveyorConstants.CONTROL_PANEL_THROTTLE);
  }
  /**
   * Moves the conveyer backward 
   */
  public void moveConveyorBackward() {
    conveyorBeltMotor.set(ControlMode.PercentOutput, -Constants.ConveyorConstants.CONVEYOR_THROTTLE);
    controlPanelTalon.set(ControlMode.PercentOutput, -Constants.ConveyorConstants.CONTROL_PANEL_THROTTLE);
  }
  /**
   * Stops all movement of the conveyer 
   */
  public void stopConveyorMovement() {
    conveyorBeltMotor.set(ControlMode.PercentOutput, 0);
    controlPanelTalon.set(ControlMode.PercentOutput, 0);
  }
   /**
   * Method that resets current amount of balls.
   */
  public void reset() {
    currentNumberOfBalls = 0;
  }
   /**
   * Method that returns a value of current amount of balls.
   * 
   * @return amount of balls currently in the bot
   */
  public int getCurrentNumberOfBalls() {
    return currentNumberOfBalls;
  }
  public boolean tomLimit(){
    return !bottomLimit.get();
    
  }
  public boolean timLimit(){
    return !topLimit.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
       // updates the number of balls. Only works if limit switch was not previously
    // enabled.
    if (tomLimit() && tomLimit() != previousSettingOfBottomSwitch && currentNumberOfBalls < 5) {
      currentNumberOfBalls += 1;
    }
    if (timLimit() && timLimit() != previousSettingOfTopSwitch && currentNumberOfBalls > 0) {
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
    previousSettingOfBottomSwitch = tomLimit();
    previousSettingOfTopSwitch = topLimit.get();
    SmartDashboard.putNumber("number_of_balls", currentNumberOfBalls);
    SmartDashboard.putBoolean("bottom", tomLimit());
    SmartDashboard.putBoolean("top", timLimit());
  }
}
