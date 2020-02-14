package frc.robot.auto;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.OI;
import frc.robot.commands.bottomintake.SpinBottomIntake;
import frc.robot.commands.drivetrain.SwerveDriveWithOdometryProfiling;
import frc.robot.commands.drivetrain.auton.SwerveAutonAlignWithLimelight;
import frc.robot.commands.indexer.MoveBallsToShooter;
import frc.robot.commands.indexer.SpinIndexer;
import frc.robot.commands.shooter.SpinShooterLimelight;
import frc.robot.commands.shooter.SpinShooterVelocity;

/**
 * Stores and selects an autonomous routine to run.
 * 
 * @author Chirag Kaushik
 * @author Jatin Kohli
 * @author Shahzeb Lakhani
 * @since February 13, 2020
 */
public class Autons {
    private static AutonCommands curAuton = AutonCommands.THREE;
    private static Trajectory firstTrajectory = Trajectories.Three.centerStartingAutonThree;
    private static Trajectory fiveStartTrajectory = Trajectories.Five.leftStarting;
    private static Trajectory secondTrajectory = Trajectories.Eight.leftStartingAutonEight;
    private static Trajectory thirdTrajectory = Trajectories.Eight.leftStartingAutonEight;
    private static Rotation2d rendevousTwoBallPickupHeading = Rotation2d.fromDegrees(120);
    private static final double REV_SPEED = 90;
    private static final double SHOOTER_SHOOT_TIME = 4.0; // how long it takes to shoot all the balls

    private static final SequentialCommandGroup threeBallAuton = new SequentialCommandGroup(
            new SwerveDriveWithOdometryProfiling(firstTrajectory, OI.forwardHeading)
                    .raceWith(new SpinShooterVelocity(REV_SPEED)),
            new SwerveAutonAlignWithLimelight(),
            new ParallelRaceGroup(new SpinShooterLimelight(), new WaitCommand(SHOOTER_SHOOT_TIME)));
    // Start on right wall, move to rendevous zone and pick up two balls,
    // move to the other side of rendevous, shoot, pickup those three and shoot

    private static final SequentialCommandGroup fiveBallAuton = new SequentialCommandGroup(
            new SwerveDriveWithOdometryProfiling(fiveStartTrajectory, OI.forwardHeading)
    );

    private static final SequentialCommandGroup rendevousEightBallAuton = new SequentialCommandGroup(
            new SwerveDriveWithOdometryProfiling(firstTrajectory, rendevousTwoBallPickupHeading)
                    .raceWith(new SpinBottomIntake(1), new SpinIndexer(false)),
            new ParallelRaceGroup(new SpinBottomIntake(1), new SpinIndexer(false), new WaitCommand(0.5)),
            new SwerveDriveWithOdometryProfiling(secondTrajectory, OI.forwardHeading)
                    .raceWith(new SpinShooterVelocity(REV_SPEED)),
            new SwerveAutonAlignWithLimelight().raceWith(new SpinShooterVelocity(REV_SPEED)),
            new SpinShooterLimelight().raceWith(new WaitCommand(2), new MoveBallsToShooter(false)),
            new SwerveDriveWithOdometryProfiling(thirdTrajectory, OI.forwardHeading).raceWith(new SpinBottomIntake(1),
                    new SpinIndexer(false)),
            new ParallelRaceGroup(new SpinBottomIntake(1), new SpinIndexer(false), new WaitCommand(0.5)),
            new SwerveAutonAlignWithLimelight().raceWith(new SpinShooterVelocity(REV_SPEED)),
            new SpinShooterVelocity(REV_SPEED).raceWith(new WaitCommand(1)),
            new SpinShooterLimelight().raceWith(new WaitCommand(2), new MoveBallsToShooter(false))
    );

    // private static Trajectory trenchEightBallStart = Trajectories.
    private static final SequentialCommandGroup trenchEightBallAuton = new SequentialCommandGroup(
    // new SwerveDriveWithOdometryProfiling(Trajectories,, OI.forwardHeading)
    );

    public static Command getAutonCommand() {
        return threeBallAuton;
    }

    public enum AutonCommands {
        THREE(threeBallAuton);

        public SequentialCommandGroup value;

        private AutonCommands(SequentialCommandGroup value) {
            this.value = value;
        }
    }
}