package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

public class AutoShooter extends CommandBase {
    private final Shooter shooter;
    private final Conveyor conveyor;

    public AutoShooter(Shooter shooter, Conveyor conveyor) {
        addRequirements(shooter);

        this.shooter = shooter;
        this.conveyor = conveyor;
    }

    @Override
    public void initialize() {
        shooter.setThrottle(0);
        conveyor.stopConveyorMovement();
    }

    @Override
    public void execute() {
        // set shooter by calling initialize on ShooterThread
        conveyor.moveConveyorForward();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setThrottle(0);
        conveyor.stopConveyorMovement();
    }

    @Override
    public boolean isFinished() {
        return conveyor.getCurrentNumberOfBalls() == 0;
    }
}