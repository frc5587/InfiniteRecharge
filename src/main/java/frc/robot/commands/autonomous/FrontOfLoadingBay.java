package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

public class FrontOfLoadingBay extends SequentialCommandGroup {
    public FrontOfLoadingBay(Arm arm, Conveyor conveyor, Drivetrain drivetrain, Intake intake, Limelight limelight, Shooter shooter, AutoPaths reverseToRendezvous, AutoPaths sCurveFromRendezvous) {
        addCommands(
                new ParallelCommandGroup(new RamseteCommandWrapper(drivetrain, reverseToRendezvous),
                        new AutoIntake(intake, conveyor)),
                new RamseteCommandWrapper(drivetrain, sCurveFromRendezvous), 
                new ParallelCommandGroup(new LimelightCentering(drivetrain, limelight)/*,*/ // max again
                        /*new AutoShootPrep(arm, limelight, shooter)*/),
                new AutoShooter(shooter, conveyor));
    }
}