/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.frc5587.lib.pid.FPID;
import org.frc5587.lib.pid.PIDVA;

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
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    public static class DrivetrainConstants {
        public static final int LEFT_LEADER = 21;
        public static final int RIGHT_LEADER = 24;
        public static final int LEFT_FOLLOWER = 23;
        public static final int RIGHT_FOLLOWER = 22;

        // TODO: Figure out real values
        // Make sure that paths with Pathfinder/WPILib respect the following as well:
        // Gyro angle value should be positive when turning counterclockwise
        public static final boolean GYRO_POSITIVE_COUNTERCLOCKWISE = false;

        // TODO: Change to real values
        public static final int TICKS_PER_REV = 1024;
        public static final double WHEEL_DIAMETER = 8.0; // in
        public static final double MAX_VELOCITY = 60; // in/s

        // TODO: Verify that assumed constants are good
        public static final FPID TURN_FPID = new FPID(0, 0.03, 0, 0);
        public static final double TURN_PID_TOLERANCE_DEG = 2.0;
        public static final double TURN_PID_FORWARD_THROTTLE = 0.2;
        public static final double TURN_PID_UPDATE_PERIOD_SEC = 0.010;

        // TODO: tune PIDVA constants
        public static final boolean PATHFINDER_TUNING = false;
        public static final PIDVA LEFT_PATHFINDER_PIDVA = new PIDVA(0, 0, 0, 1 / MAX_VELOCITY, 0);
        public static final PIDVA RIGHT_PATHFINDER_PIDVA = new PIDVA(0, 0, 0, 1 / MAX_VELOCITY, 0);
        public static final double PATHFINDER_TURN_P = 0.01;

        // Characterisation values
        // TODO: Find actual values
        public static final double KS_VOLTS = 0;
        public static final double KV_VOLT_SECONDS_PER_METER = 0;
        public static final double KA_VOLT_SECONDS_PER_SQUARED_METER = 0;

        public static final double TRACK_WIDTH_METERS = 0.69; // TODO: Change to measured value
        public static final DifferentialDriveKinematics DRIVETRAIN_KINEMATICS = new DifferentialDriveKinematics(
                TRACK_WIDTH_METERS);

        public static final double RAMSETTE_KP_DRIVE_VEL = 8.5;
    }
}
