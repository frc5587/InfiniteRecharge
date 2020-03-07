package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class AutoConveyor extends CommandBase {
    private final Conveyor conveyor;

    public AutoConveyor(Conveyor conveyor) {
        addRequirements(conveyor);

        this.conveyor = conveyor;
    }

    @Override
    public void initialize() {
        conveyor.stopConveyorMovement();
    }

    @Override
    public void execute() {
        conveyor.moveConveyorForward();
    }

    @Override
    public void end(boolean isFinished) {
        conveyor.stopConveyorMovement();
    }

    @Override
    public boolean isFinished() {
        return conveyor.getCurrentNumberOfBalls() == 0;
    }
}