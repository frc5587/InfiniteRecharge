package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmThread;
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
import frc.robot.subsystems.ShooterJRAD;

public class CloseWall extends SequentialCommandGroup {
  public CloseWall(Arm arm, Conveyor conveyor, Drivetrain drivetrain, Intake intake, Limelight limelight,
      ShooterJRAD shooter) {
    addCommands(
        // new ParallelRaceGroup(new RamseteCommandWrapper(drivetrain, AutoPaths.ReverseToTrench), new AutoIntake(intake, conveyor)),
        // new ParallelRaceGroup(new LimelightCentering(drivetrain, limelight)),
        // new SequentialCommandGroup(new FindTarget(arm, limelight),
        //     new ParallelRaceGroup(new ArmThread(arm, limelight), new ShooterThread(arm, shooter, limelight, conveyor, true))),
        // new ParallelCommandGroup(new RamseteCommandWrapper(drivetrain, AutoPaths.ReverseUnderTrench),
        //     new AutoIntake(intake, conveyor)));
        new RamseteCommandWrapper(drivetrain, AutoPaths.ReverseToTrench));
  }
}