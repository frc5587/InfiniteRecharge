package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class SetArmThenShoot extends SequentialCommandGroup {
    public SetArmThenShoot(Shooter m_shooter, Arm m_arm) {
        var distanceMeters = SmartDashboard.getNumber("distance", 2);
        var angleDegrees = Shooter.calcArmAngleDegrees(distanceMeters);

        addCommands(
            new InstantCommand(() -> m_arm.setArmAngleDegrees(angleDegrees), m_arm),
            new Shoot(m_shooter, (() -> m_shooter.calculateShooterSpeed(distanceMeters, angleDegrees)), true)
        );
    }
}