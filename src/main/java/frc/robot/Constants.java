/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import org.frc5587.lib.pid.PID;
import org.frc5587.lib.pid.FPID;

import edu.wpi.first.wpilibj.controller.ArmFeedforward;


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

  public static final class ConveyorConstants {
    public static final int CONVEYOR_MOTOR = 10;
  }

    public static final class ArmConstants {
        public static final int ARM_MOTOR = 30;

        public static final double kV = .169;
        public static final double kA = .0125;
        public static final ArmFeedforward FF = new ArmFeedforward (
            .219, //kS 
            .439, //kCos
            kV, //kV
            kA //kA
            );

        public static final PID ARM_PID = new PID(
            .273, //kP
            0.0, //kI
            125.//0 //kD
            );
        public static final double ARM_OFFSET_RADS = 14 * Math.PI / 180.0;
    }
}