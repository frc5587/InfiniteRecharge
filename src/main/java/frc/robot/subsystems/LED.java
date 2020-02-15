package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

    private final I2C arduino = new I2C(I2C.Port.kMXP, 8);

    public void sendArmAngle(double angle) {
        byte[] angles = {(byte) angle};

        arduino.writeBulk(angles);
    }
}