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
    private final TalonSRX conveyorBeltMotor = new TalonSRX(Constants.ConveyorConstants.CONVEYOR_MOTOR);

    /**
     * Creates a new Conveyor.
     */
    public Conveyor() {

    }

    /**
     * Moves the conveyer forward
     */
    public void moveForward() {
        conveyorBeltMotor.set(ControlMode.PercentOutput, 0.30);
    }

    /**
     * Moves the conveyer backwards
     */
    public void moveBackward() {
        conveyorBeltMotor.set(ControlMode.PercentOutput, -0.30);
    }

    /**
     * Stops all movement of the conveyer
     */
    public void stopMovement() {
        conveyorBeltMotor.set(ControlMode.PercentOutput, 0);
    }
}
