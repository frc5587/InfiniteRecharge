package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmThread;
import frc.robot.commands.AutoConveyor;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.FindTarget;
import frc.robot.commands.LimelightCentering;
import frc.robot.commands.RamseteCommandWrapper;
import frc.robot.commands.ShooterThread;
import frc.robot.commands.RamseteCommandWrapper.AutoPaths;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class CloseWall extends SequentialCommandGroup {
    public CloseWall(Arm arm, Conveyor conveyor, Drivetrain drivetrain, Intake intake, Limelight limelight, Shooter shooter, AutoPaths reverseToTrench, AutoPaths reverseUnderTrench, FindTarget findTarget, ArmThread armThread, ShooterThread shooterThread, AutoConveyor autoConveyor) {
        addCommands(
                new ParallelCommandGroup(new RamseteCommandWrapper(drivetrain, reverseToTrench),
                        new AutoIntake(intake, conveyor)),
                new ParallelRaceGroup(new LimelightCentering(drivetrain, limelight)),
                new SequentialCommandGroup(findTarget, new ParallelRaceGroup(armThread, shooterThread, autoConveyor)),
                new ParallelCommandGroup(new RamseteCommandWrapper(drivetrain, reverseUnderTrench),
                        new AutoIntake(intake, conveyor)));
    }
}