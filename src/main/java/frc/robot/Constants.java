/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class ShooterConstants {
        public static final int SHOOTER_MOTOR_ONE = 31;
        public static final int SHOOTER_MOTOR_TWO = 32; 
        
        public static final float MIN_OUTPUT = 1;
        public static final float MAX_OUTPUT = 1;
     
        public static final double V_COMP_SATURATION = 12.0;

        public static final int SMART_MOTION_SLOT = 0;

        public static final int K_TIMEOUT_MS = 10;

        public static final double MAX_VELOCITY_ONE = 4948;
        public static final double MAX_VELOCITY_TWO = 4650; // 4650 4530


        public static final FPID SHOOTER_FPID = new FPID (
            (1 / MAX_VELOCITY_ONE), // kF
            ((.065 / (3000 - 2954)) * 1.5), // kP // .1 / error // 2.01
            0, // kI
            // ((10 * ((.1 / (3000 - 2954)) / 2)) * 8) // kD
            // (10 * (.065 / (3000 - 2954)) * 1.5)
            .1 * 6
        );

        public static final FPID SHOOTER_TWO_FPID = new FPID (
            (1 / MAX_VELOCITY_TWO), // kF
            // ((.1 / (3040 - 3000)) * 1.5),
            ((.057 / (3040 - 3000)) * 1.5),
            // .0001,
            0, // kI
            // 0
            (8.5 * (.05 / (3040 - 3000)) * 12.5) // kD // first val was 10, 8.5, 8
            // 0
        );
    }
}
