/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A Shoot command that operates the shooter
 */
public class Shoot extends CommandBase {
    private Shooter shooter;
    private DoubleSupplier yAxis;
    private boolean velocityControl;

    /**
     * Creates a new Shoot command
     *
     * @param subsystem The subsystem used by this command.
     */
    public Shoot(Shooter shooter, DoubleSupplier y, boolean velocityControl) {
        this.shooter = shooter;
        yAxis = y;
        this.velocityControl = velocityControl;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooter);
    }

    public Shoot(Shooter shooter, DoubleSupplier y) {
        this.shooter = shooter;
        yAxis = y;
        this.velocityControl = false;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        SmartDashboard.putNumber("Setpoint", 0.0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (velocityControl) {
            shooter.setVelocity(yAxis.getAsDouble());
        } else {
            shooter.setThrottle(yAxis.getAsDouble());
        }
        // shooter.setVelocity(SmartDashboard.getNumber("Setpoint", 0.0));
        shooter.log();
    }

    /**
     * Stops the shooter from spinning
     */
    @Override
    public void end(boolean interrupted) {
        shooter.setThrottle(0);
    }
}
