/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

/**
 * The subsystem for the climber
 */
public class Climber extends SubsystemBase {
    private final TalonFX climberMotor = new TalonFX(Constants.ClimberConstants.CLIMBER_MOTOR);

    /**
     * Inverts the motors
     */
    public Climber() {
        climberMotor.setInverted(true);
    }

    /**
     * Set the climber's motor to a certain speed
     * 
     * @param percent percent to set the motor to
     */
    public void set(double percent) {
        climberMotor.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
