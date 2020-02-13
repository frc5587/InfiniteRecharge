/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;

/**
 * A robot arm subsystem that moves with a motion profile.
 */
public class ArmSubsystem extends ProfiledPIDSubsystem {
  private final CANSparkMax motor = new CANSparkMax(ArmConstants.ARM_MOTOR, MotorType.kBrushless);
  private final CANEncoder encoder = motor.getAlternateEncoder(AlternateEncoderType.kQuadrature, 8192);

  /**
   * Create a new ArmSubsystem.
   */
  public ArmSubsystem() {
    super(new ProfiledPIDController(ArmConstants.ARM_PID.kP, ArmConstants.ARM_PID.kI, ArmConstants.ARM_PID.kD,
        new TrapezoidProfile.Constraints(ArmConstants.kV, ArmConstants.kA)), 0);

    // Start arm at rest in neutral position
    setGoal(ArmConstants.ARM_OFFSET_RADS);

    // Clear all previous constants
    motor.restoreFactoryDefaults();

    // Invert so counterclockwise is positive
    motor.setInverted(true);
    encoder.setInverted(true);

    // Use conversion factor to convert from rotations to radians
    encoder.setPositionConversionFactor(Math.PI);
    encoder.setVelocityConversionFactor(Math.PI);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the setpoint
    var feedforward = ArmConstants.FF.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    motor.setVoltage(output + feedforward);
  }

  @Override
  public double getMeasurement() {
    // The position is automatically scaled by the set conversion factor
    return encoder.getPosition() + ArmConstants.ARM_OFFSET_RADS;
  }

  public void setThrottle(double percent) {
    motor.set(percent);
  }
}