package frc.robot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorMatch;

public final class Constants {
    public static final class CPConstants {
        //public static final int CPMOTOR = 10;
        public static final I2C.Port i2cPort = I2C.Port.kOnboard;
        public static Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
        public static Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
        public static Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
        public static Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
    }
}