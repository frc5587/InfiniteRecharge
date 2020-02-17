/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.frc5587.lib.pid.FPID;

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

    /**
     * Constants used by the Limelight.
     */
    public static class LimelightConstants {
        public static final double ARM_HEIGHT_INCHES = 8.75;
        public static final double STANDOFF_INCHES = 1.097;
        public static final double GOAL_HEIGHT_INCHES = 98.25;
        public static final double ARM_LENGTH_INCHES = 4404471.0 / 125000;
        public static final double ARM_AXLE_HEIGHT_INCHES = 1739459.0 / 250000;
    }

    public static class AutoConstants {
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 10;
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
        public static final boolean LEFT_ENCODER_INVERTED = true;
        public static final boolean RIGHT_ENCODER_INVERTED = true;

        public static final int SMART_CURRENT_LIMIT = 30;
        public static final int HARD_CURRENT_LIMIT = 40;

        // TODO: Figure out real values
        // Make sure that paths with Pathfinder/WPILib respect the following as well:
        // Gyro angle value should be positive when turning counterclockwise
        public static final boolean INVERT_GYRO_DIRECTION = false;

        // TODO: Verify that assumed constants are good
        // Turn PID constants
        public static final FPID TURN_FPID = new FPID(0, 0.1, 0, 0.009);
        public static final double TURN_PID_TOLERANCE_DEG = 0.5;
        public static final double TURN_PID_FORWARD_THROTTLE = 0;
        public static final double TURN_PID_UPDATE_PERIOD_SEC = 0.010;

        // Values from characterisation
        // TODO: Find actual values
        public static final double KS_VOLTS = 0;
        public static final double KV_VOLT_SECONDS_PER_METER = 0;
        public static final double KA_VOLT_SECONDS_PER_SQUARED_METER = 0;
        public static final double TRACK_WIDTH_METERS = 0.69;

        // TODO: Change to real values
        // Basic differential drivetrain kinematics constants
        public static final int TICKS_PER_REV = 8192;
        public static final double WHEEL_DIAMETER_METERS = 0.2032;
        public static final double WHEEL_RADIUS_METERS = WHEEL_DIAMETER_METERS / 2;
        public static final DifferentialDriveKinematics DRIVETRAIN_KINEMATICS = new DifferentialDriveKinematics(
                TRACK_WIDTH_METERS);

        // Ramsete constants
        public static final double RAMSETE_KP_DRIVE_VEL = 8.5;

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
}
