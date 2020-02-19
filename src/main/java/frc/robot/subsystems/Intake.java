/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
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
 * The subsystem for the intake
 */
public class Intake extends SubsystemBase {
    private final TalonSRX intakeTalon = new TalonSRX(IntakeConstants.INTAKE_MOTOR);
    private final TalonSRX centeringTalon = new TalonSRX(IntakeConstants.CENTERING_MOTOR);

    /**
     * Set the intake to a particular speed
     * 
     * @param throttle the speed at which the intake takes in or removes power cells
     */
    public void set(double throttle) {
        intakeTalon.set(ControlMode.Velocity, -throttle);
        centeringTalon.set(ControlMode.Velocity, -throttle);
    }
}
