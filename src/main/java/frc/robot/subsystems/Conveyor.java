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
import frc.robot.Constants;

public class Conveyor extends SubsystemBase {
  private final TalonSRX controlPanelTalon = new TalonSRX(Constants.ConveyorConstants.CONTROL_PANEL_TALON);
  private final TalonSRX conveyorBeltMotor = new TalonSRX(Constants.ConveyorConstants.CONVEYOR_MOTOR);
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
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
