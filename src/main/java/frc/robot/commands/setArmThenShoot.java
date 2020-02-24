package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Limelight;

public class SetArmThenShoot extends SequentialCommandGroup {
    public SetArmThenShoot(Shooter m_shooter, Arm m_arm, Limelight m_limelight) {
        this(m_shooter, m_arm, m_limelight, false);
        
    }

    public SetArmThenShoot(Shooter m_shooter, Arm m_arm, Limelight m_limelight, boolean useLimelight) {
        double distanceMeters = useLimelight? m_limelight.getShooterDistance(m_arm.getAngleRadians()) : SmartDashboard.getNumber("distance", 2);
        double moveToAngleDegrees = Shooter.calcArmAngleDegrees(distanceMeters);

        addCommands(
            new InstantCommand(() -> m_arm.setArmAngleDegrees(moveToAngleDegrees), m_arm),
            new Shoot(m_shooter, (() -> m_shooter.calculateShooterSpeed(distanceMeters, moveToAngleDegrees)), true)
        );
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Ending SetArmThenShoot");
        super.end(interrupted);
    }
}