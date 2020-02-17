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
import frc.robot.Constants.IntakeConstants;

public class Conveyor extends SubsystemBase {
  private final TalonSRX conveyorBeltMotor = new TalonSRX(Constants.ConveyorConstants.CONVEYOR_MOTOR);
  private final TalonSRX intakeTalon = new TalonSRX(IntakeConstants.INTAKE_MOTOR);

  public Conveyor() {
    intakeTalon.setInverted(true);
    conveyorBeltMotor.setInverted(true);

  }

  /**
   * Moves the conveyor forward
   */
  public void moveForward() {
    conveyorBeltMotor.set(ControlMode.PercentOutput, 0.75);
  }

  /**
   * Moves the conveyor backwards
   */
  public void moveBackward() {
    conveyorBeltMotor.set(ControlMode.PercentOutput, -0.75);
  }

  /**
   * Stops all movement of the conveyor
   */
  public void stopMovement() {
    conveyorBeltMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {

  }
}
