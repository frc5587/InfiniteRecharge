package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class AutoShootPrep extends CommandBase {
    private final Arm arm;
    private final Limelight limelight;
    private final Shooter shooter;

    public AutoShootPrep(Arm arm, Limelight limelight, Shooter shooter) {
        addRequirements(arm, limelight, shooter);

        this.arm = arm;
        this.limelight = limelight;
        this.shooter = shooter;
    }

    // TODO: implement actual method (either limelight or math)

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        // does this need to be variable?
        shooter.setVelocity(3000);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }
}