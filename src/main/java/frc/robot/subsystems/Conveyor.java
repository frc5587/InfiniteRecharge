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

public class Conveyor extends SubsystemBase {
  private final TalonSRX conveyorBeltMotor = new TalonSRX(10);

  /**
   * Creates a new Conveyor.
   */
  public Conveyor() {

  }

  public void moveForward() {
    conveyorBeltMotor.set(ControlMode.PercentOutput, 0.30);
  }

  public void moveBackward() {
    conveyorBeltMotor.set(ControlMode.PercentOutput, -0.30);
  }

  public void stopMovement() {
    conveyorBeltMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
