package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Timer;

public class DriveBackwards extends CommandBase {
    private Drivetrain drivetrain;
    private Timer timer = new Timer();

    public DriveBackwards(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        drivetrain.setDrive(.75);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 7;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        drivetrain.stopDrivetrain();
        timer.stop();
    }
}