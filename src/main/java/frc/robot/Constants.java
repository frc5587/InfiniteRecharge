/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;
import org.frc5587.lib.pid.FPID;
import org.frc5587.lib.pid.PID;

import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class AutoConstants {
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 3;
    public static final double MAX_ACCEL_METERS_PER_SECOND_SQUARED = 3;

    // Reasonable values from WPILib docs (values are robot-agnostic)
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;
  }

  public static class DrivetrainConstants {
    public static final int LEFT_LEADER = 10;
    public static final int LEFT_FOLLOWER = 12;

    public static final int RIGHT_LEADER = 11;
    public static final int RIGHT_FOLLOWER = 13;

    public static final boolean LEFT_SIDE_INVERTED = true;
    public static final boolean RIGHT_SIDE_INVERTED = true;
    public static final boolean LEFT_ENCODER_INVERTED = false;
    public static final boolean RIGHT_ENCODER_INVERTED = true;

    public static final int SMART_CURRENT_LIMIT = 30;
    public static final int HARD_CURRENT_LIMIT = 40;

    // Make sure that paths with Pathfinder/WPILib respect the following as well:
    // Gyro angle value should be positive when turning counterclockwise
    public static final boolean INVERT_GYRO_DIRECTION = true;

    // Turn PID constants
    public static final FPID TURN_FPID = new FPID(0, 0.1, 0, 0.009);
    public static final double TURN_PID_TOLERANCE_DEG = 0.5;
    public static final double TURN_PID_FORWARD_THROTTLE = 0;
    public static final double TURN_PID_UPDATE_PERIOD_SEC = 0.010;

    // Values from characterisation
    public static final double KS_VOLTS = 0.153;
    public static final double KV_VOLT_SECONDS_PER_METER = 2.2;
    public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = 0.394;
    public static final double TRACK_WIDTH_METERS = 0.686863135; // Empirically determined fixed
    public static final double RAMSETE_KP_DRIVE_VEL = 14.4; // Raw from charact. fixed

    // Basic differential drivetrain kinematics constants
    public static final int TICKS_PER_REV = 8192;
    public static final double WHEEL_DIAMETER_METERS = 0.2032;
    public static final double WHEEL_RADIUS_METERS = WHEEL_DIAMETER_METERS / 2;
    public static final DifferentialDriveKinematics DRIVETRAIN_KINEMATICS = new DifferentialDriveKinematics(
        TRACK_WIDTH_METERS);

    // Lag compensation
    public static final int HISTORY_LIMIT = 32;
  }

  public static class MLConstants {
    // TODO: Replace with actual values
    // Distance from floor to the centre of the camera in meters
    public static final double CENTER_CAMERA_HEIGHT_METERS = 0.05;
    public static final double CAMERA_FOCAL_LENGTH_PIXELS = 500;
    public static final double CAMERA_WIDTH_PIXELS = 320;
    public static final double CAMERA_HEIGHT_PIXELS = 240;

    public static final double POWER_CELL_RADIUS_METERS = 0.1778 / 2;
  }

  /**
   * Constants used by the climber
   */
  public static final class ClimberConstants {
    public static final int CLIMBER_MOTOR = 1;
  }

  /**
   * Constants used by the intake
   */
  public static final class IntakeConstants {
    public static final int INTAKE_MOTOR = 41;
    public static final int CENTERING_MOTOR = 42;
    public static final int CONVEYOR_MOTOR = 50;
    public static final int BOTTOM_LIMIT = 0;
    public static final int TOP_LIMIT = 1;
    public static final double THROTTLE = 1.0;
    public static final double CONVEYOR_THROTTLE = 0.75;
  }

  public static final class ConveyorConstants {
    public static final double CONVEYOR_THROTTLE = 0.75;
    public static final int CONTROL_PANEL_MOTOR = 51;
    public static final double CONTROL_PANEL_THROTTLE = 0.75;
    public static final int CONVEYOR_MOTOR = 50;
}

  public static final class ArmConstants {
    public static final int ARM_MOTOR = 30;

    public static final int ARM_LIMIT_SWITCH = 9;

    public static final ArmFeedforward FF = new ArmFeedforward(.219, // kS
        .439, // kCos
        .169, // kV
        .0125 // kA
    );

    public static final PID ARM_PID = new PID(23, // kP
        0.0, // kI
        // 127.//0 //kD
        0);

    public static final double ARM_LENGTH_METERS = 0.9;

  }

  public static final class ShooterConstants {
    public static final int SHOOTER_MOTOR_ONE = 20; // top
    public static final int SHOOTER_MOTOR_TWO = 21; // bottom

    public static final float MIN_OUTPUT = 1;
    public static final float MAX_OUTPUT = 1;

    public static final double V_COMP_SATURATION = 12.0;

    public static final int SMART_MOTION_SLOT = 0;

    public static final int K_TIMEOUT_MS = 10;

    public static final double MAX_VELOCITY_ONE = 5074;
    public static final double MAX_VELOCITY_TWO = 5080;

    public static final FPID SHOOTER_ONE_FPID = new FPID((1 / MAX_VELOCITY_ONE), // kF
        ((.065 / (3000 - 2954)) * 1.5), // kP
        0, // kI
        8 // kD
    );

    public static final FPID SHOOTER_TWO_FPID = new FPID((1 / MAX_VELOCITY_TWO), // kF
        ((.057 / (3040 - 3000)) * 1.5), // kP
        0, // kI
        (8.5 * (.05 / (3040 - 3000)) * 12.5) // kD
    );

    public static final double FLYWHEEL_RADIUS = 0.0508; // radius in meters (2")

    public static final double CONVERSION_FACTOR = 30 / Math.PI; // RPM --> radians/second

    public static final double GOAL_HEIGHT = 2.495; // height in meters

    public static final double G = 9.8; // gravitational acceleration
  }

  /**
   * Constants used by the Limelight
   */
  public static class LimelightConstants {
    public static final double ARM_HEIGHT_METERS = Units.inchesToMeters(8.75);
    public static final double ARM_LENGTH_METERS = Units.inchesToMeters(4404471.0 / 125000);
    public static final double ARM_AXLE_HEIGHT_METERS = Units.inchesToMeters(1739459.0 / 250000);

    public static final double STANDOFF_METERS = Units.inchesToMeters(1.097);

    public static final double GOAL_HEIGHT_METERS = Units.inchesToMeters(98.25);
    public static final double INNER_OUTER_GOAL_DISTANCE_METERS = Units.inchesToMeters(29.25);

    public static final double G_METERS_PER_SECOND_SQUARED = 9.81;

    public static final double THREAD_PERIOD_TIME_SECONDS = 0.01;
  }
}
