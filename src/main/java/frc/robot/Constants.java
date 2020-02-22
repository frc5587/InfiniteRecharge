/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

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
    }
}
