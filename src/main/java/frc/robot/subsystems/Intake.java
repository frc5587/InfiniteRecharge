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
 * Intake subsystem allows the intake to run by operating both SparkMaxes at
 * full speed. Constants are available in constants and other things are
 * initialized in the robot container.
 */
public class Intake extends SubsystemBase {
  private final CANSparkMax intakeSparkMax = new CANSparkMax(IntakeConstants.INTAKE_SPARKMAX, MotorType.kBrushless);
  private final CANSparkMax intakeSparkMax2 = new CANSparkMax(IntakeConstants.INTAKE_SPARKMAX2, MotorType.kBrushless);

  /**
   * public void set, sets the SparkMaxes to -throttle which is full speed in the
   * positive direction.
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
