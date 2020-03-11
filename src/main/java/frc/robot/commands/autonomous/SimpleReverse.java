package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmThread;
import frc.robot.commands.DriveBackwards;
import frc.robot.commands.FindTarget;
import frc.robot.commands.LimelightCentering;
import frc.robot.commands.ResetEncoder;
import frc.robot.commands.ShooterThread;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterJRAD;

public class SimpleReverse extends SequentialCommandGroup {
    public SimpleReverse(Arm arm, Conveyor conveyor, Drivetrain drivetrain, Intake intake, Limelight limelight, ShooterJRAD shooter) {
        addCommands(
            new DriveBackwards(drivetrain),
            new SequentialCommandGroup(new ResetEncoder(arm), 
                new FindTarget(arm, limelight), 
                new SequentialCommandGroup(new ParallelRaceGroup(new ArmThread(arm, limelight), 
                                                        new LimelightCentering(drivetrain, limelight)), 
                    new ShooterThread(arm, shooter, limelight, conveyor))));
    }
}