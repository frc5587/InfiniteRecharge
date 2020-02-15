package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LED;

public class ControlLED extends CommandBase {

    private final LED leds = new LED();
    private final Arm arm;
    private final DoubleSupplier angle;

    public ControlLED(Arm arm, DoubleSupplier angle) {
        this.arm = arm;
        this.angle = angle;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        var currentAngle = angle.getAsDouble();

        leds.sendArmAngle(currentAngle);
    }
}