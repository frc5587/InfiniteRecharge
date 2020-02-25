package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoIntake;
// import frc.robot.commands.AutoShootPrep;
import frc.robot.commands.AutoShooter;
import frc.robot.commands.LimelightCentering;
import frc.robot.commands.RamseteCommandWrapper;
import frc.robot.commands.RamseteCommandWrapper.AutoPaths;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class CloseWall extends SequentialCommandGroup {
    public CloseWall(Arm arm, Conveyor conveyor, Drivetrain drivetrain, Intake intake, Limelight limelight, Shooter shooter, AutoPaths reverseToTrench, AutoPaths reverseUnderTrench) {
        addCommands(
                new ParallelCommandGroup(new RamseteCommandWrapper(drivetrain, reverseToTrench),
                        new AutoIntake(intake, conveyor)),
                new ParallelRaceGroup(new LimelightCentering(drivetrain, limelight)/*,*/ // again, max said this can happen separately
                        /*new AutoShootPrep(arm, limelight, shooter)*/),
                new AutoShooter(shooter, conveyor),
                new ParallelCommandGroup(new RamseteCommandWrapper(drivetrain, reverseUnderTrench),
                        new AutoIntake(intake, conveyor)));
    }
}