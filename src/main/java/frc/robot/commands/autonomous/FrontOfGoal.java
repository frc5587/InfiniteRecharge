package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.FindTarget;
import frc.robot.commands.ArmThread;
import frc.robot.commands.AutoConveyor;
import frc.robot.commands.LimelightCentering;
import frc.robot.commands.RamseteCommandWrapper;
import frc.robot.commands.ShooterThread;
import frc.robot.commands.RamseteCommandWrapper.AutoPaths;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
// import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterJRAD;

public class FrontOfGoal extends SequentialCommandGroup {
    // AutoPaths rightStartToPowerPort;

    public FrontOfGoal(Drivetrain drivetrain, ShooterJRAD shooter, Arm arm, Conveyor conveyor, Intake intake, Limelight limelight) {

        addCommands(new SequentialCommandGroup(new FindTarget(arm, limelight), new ParallelRaceGroup(new ArmThread(arm, limelight), new ShooterThread(arm, shooter, limelight, conveyor))),
                new ParallelCommandGroup(new RamseteCommandWrapper(drivetrain, RamseteCommandWrapper.AutoPaths.RightStartToPowerPort),
                    new AutoIntake(intake, conveyor)),
                new LimelightCentering(drivetrain, limelight),
                new SequentialCommandGroup(new FindTarget(arm, limelight), new ParallelRaceGroup(new ArmThread(arm, limelight), new ShooterThread(arm, shooter, limelight, conveyor))));
    }

}