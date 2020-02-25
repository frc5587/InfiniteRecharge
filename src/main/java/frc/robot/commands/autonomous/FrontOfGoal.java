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

public class FrontOfGoal extends SequentialCommandGroup {

    public FrontOfGoal(Drivetrain drivetrain, Shooter shooter, Arm arm, Conveyor conveyor, Intake intake, Limelight limelight, AutoPaths rightStartToPowerPort) {

        addCommands(new AutoShooter(shooter, conveyor),
                new ParallelCommandGroup(new RamseteCommandWrapper(drivetrain, rightStartToPowerPort),
                    new AutoIntake(intake, conveyor)),
                // new ParallelCommandGroup(new AutoShootPrep(arm, limelight, shooter), // this is commented out bc max's multithreading will allow this to run in the background
                    new LimelightCentering(drivetrain, limelight)/*)*/,
                new AutoShooter(shooter, conveyor));
    }

}