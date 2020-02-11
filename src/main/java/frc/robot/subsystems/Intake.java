/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * The subsystem for the intake
 */
public class Intake extends SubsystemBase {
  private final CANSparkMax intakeSparkMax = new CANSparkMax(IntakeConstants.INTAKE_SPARKMAX, MotorType.kBrushless);
  private final CANSparkMax intakeSparkMax2 = new CANSparkMax(IntakeConstants.INTAKE_SPARKMAX2, MotorType.kBrushless);

  /**
   * Set the intake to a particular speed
   * 
   * @param throttle the speed at which the intake takes in or removes power cells
   */
  public void set(double throttle) {
    intakeSparkMax.set(-throttle);
    intakeSparkMax2.set(-throttle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
