package frc.robot.auto;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import frc.robot.commands.shooter.SpinShooterLimelightAuton;
import frc.robot.commands.shooter.SpinShooterVelocity;
import frc.robot.subsystems.BottomIntake;

/**
 * Stores and selects an autonomous routine to run.
 * 
 * @author Chirag Kaushik
 * @author Jatin Kohli
 * @author Shahzeb Lakhani
 * @author Anirudh Kotamraju
 * @author Angela Jia
 * @since February 13, 2020
 */
public class Autons {
    private enum StartingPosition {
        LEFT, MIDDLE, RIGHT
    }
    
    //Change this before the match.
    private static StartingPosition startingPosition = StartingPosition.LEFT;
    private static AutonCommands curAuton = AutonCommands.THREE; 
    private static boolean trenchChosen = false;

    //Brings the intake out, required at the start of every autonomous configuration
    private static InstantCommand intakeOut = new InstantCommand(
            () -> BottomIntake.getInstance().getSolenoid().set(BottomIntake.OUT), BottomIntake.getInstance());
    
    private static Trajectory threeStartTrajectory;
    private static Trajectory fiveStartTrajectory;
    private static Trajectory eightStartTrajectory;


    /**
     * Switch between the various autonomous paths.
     * NOTE: //trenchChosen doesn't matter for 3 ball auto
     */
    static {
        if(trenchChosen) {
            if(startingPosition == StartingPosition.LEFT) {
                threeStartTrajectory = Trajectories.Three.leftStarting;
                fiveStartTrajectory = Trajectories.FiveTrench.leftStarting;
                eightStartTrajectory = Trajectories.EightTrench.leftStarting;
            } else if(startingPosition == StartingPosition.MIDDLE) {
                threeStartTrajectory = Trajectories.Three.middleStarting;
                fiveStartTrajectory = Trajectories.FiveTrench.middleStarting;
                eightStartTrajectory = Trajectories.EightTrench.middleStarting;
            } else {
                threeStartTrajectory = Trajectories.Three.rightStarting;
                fiveStartTrajectory = Trajectories.FiveTrench.rightStarting;
                eightStartTrajectory = Trajectories.EightTrench.rightStarting;
            }
        } else {
            if(startingPosition == StartingPosition.LEFT) {
                threeStartTrajectory = Trajectories.Three.leftStarting;
                fiveStartTrajectory = Trajectories.FiveRendezvous.leftStarting;
                eightStartTrajectory = Trajectories.EightRendezvous.leftStarting;
            } else if(startingPosition == StartingPosition.MIDDLE) {
                threeStartTrajectory = Trajectories.Three.middleStarting;
                fiveStartTrajectory = Trajectories.FiveRendezvous.middleStarting;
                eightStartTrajectory = Trajectories.EightRendezvous.middleStarting;
            } else {
                threeStartTrajectory = Trajectories.Three.rightStarting;
                fiveStartTrajectory = Trajectories.FiveRendezvous.rightStarting;
                eightStartTrajectory = Trajectories.EightRendezvous.rightStarting;
            }
        }
    }

    private static Rotation2d rendezvousTwoBallPickupHeading = Rotation2d.fromDegrees(120);
    private static final double REV_SPEED = 90;
    private static final double SHOOTER_SHOOT_TIME = 4.0; // how long it takes to shoot all the balls

    private static final SequentialCommandGroup baselineAuto = new SequentialCommandGroup(
        new SwerveDriveWithOdometryProfiling(Trajectories.Baseline.moveForward, OI.forwardHeading)
    );

    //Shoot three pre-loaded balls
    private static final SequentialCommandGroup three = new SequentialCommandGroup(
        new SwerveDriveWithOdometryProfiling(threeStartTrajectory, OI.forwardHeading)
            .raceWith(new SpinShooterVelocity(REV_SPEED)),
        new SwerveAutonAlignWithLimelight(),
        new ParallelRaceGroup(new SpinShooterLimelight(), new WaitCommand(SHOOTER_SHOOT_TIME)));
   
    //pick up two balls from opponents trench and shoot all five
    private static final SequentialCommandGroup trenchFiveAuton = new SequentialCommandGroup(
        intakeOut,
        new SwerveDriveWithOdometryProfiling(fiveStartTrajectory, OI.forwardHeading).raceWith(new SpinBottomIntake(1)),
        new SwerveDriveWithOdometryProfiling(Trajectories.FiveTrench.pickupToShoot, OI.forwardHeading).raceWith(new MoveBallsToShooter(false)),
        new SpinShooterLimelightAuton(SHOOTER_SHOOT_TIME).raceWith(new MoveBallsToShooter(false)));

    //pick up two balls from rendezvous point and shoot all five
    private static final SequentialCommandGroup rendezvousFiveAuton = new SequentialCommandGroup(
        intakeOut,
        new SwerveDriveWithOdometryProfiling(fiveStartTrajectory, OI.forwardHeading).raceWith(new SpinBottomIntake(1)),
        new SwerveDriveWithOdometryProfiling(Trajectories.FiveRendezvous.pickupToShoot, OI.forwardHeading).raceWith(new MoveBallsToShooter(false)),
        new SpinShooterLimelightAuton(SHOOTER_SHOOT_TIME).raceWith(new MoveBallsToShooter(false)));

    private static Trajectory rendevousEightStart = Trajectories.EightRendezvous.middleStarting;
    
    //shoot all three pre-loaded balls, pick up five from the rendevous point, and shoot those five
    private static final SequentialCommandGroup rendezvousEightAuton = new SequentialCommandGroup(
        intakeOut,
        new SwerveDriveWithOdometryProfiling(eightStartTrajectory, rendezvousTwoBallPickupHeading)
             .raceWith(new SpinBottomIntake(1), new SpinIndexer(false)),
        new ParallelRaceGroup(new SpinBottomIntake(1), new SpinIndexer(false), new WaitCommand(0.5)),
        new SwerveDriveWithOdometryProfiling(Trajectories.EightRendezvous.fromTwoBallRendezvousToThreeBallRendezvous, OI.forwardHeading)
             .raceWith(new SpinShooterVelocity(REV_SPEED)),
        new SwerveAutonAlignWithLimelight().raceWith(new SpinShooterVelocity(REV_SPEED)),
        new SpinShooterLimelight().raceWith(new WaitCommand(2), new MoveBallsToShooter(false)),
        new SwerveDriveWithOdometryProfiling(Trajectories.EightRendezvous.fromTwoBallRendezvousToThreeBallRendezvous, OI.forwardHeading).raceWith(new SpinBottomIntake(1),
             new SpinIndexer(false)),
        new ParallelRaceGroup(new SpinBottomIntake(1), new SpinIndexer(false), new WaitCommand(0.5)),
        new SwerveAutonAlignWithLimelight().raceWith(new SpinShooterVelocity(REV_SPEED)),
        new SpinShooterVelocity(REV_SPEED).raceWith(new WaitCommand(1)), 
        new SpinShooterLimelight().raceWith(new WaitCommand(2), new MoveBallsToShooter(false))
    );

    private static Trajectory trenchEightStart = Trajectories.EightTrench.middleStarting;

    //shoot all three pre-loaded balls, pick up five from the alliance trench, and shoot those five
    private static final SequentialCommandGroup trenchEightAuton = new SequentialCommandGroup(
        intakeOut, 
        new SwerveDriveWithOdometryProfiling(trenchEightStart, OI.forwardHeading)  // move to shooting location
            .raceWith(new SpinShooterVelocity(REV_SPEED)),
        new SwerveAutonAlignWithLimelight() // align with target
            .raceWith(new SpinShooterVelocity(REV_SPEED)),
        new SpinShooterLimelight().raceWith(new WaitCommand(SHOOTER_SHOOT_TIME)), // shoot
        new SwerveDriveWithOdometryProfiling(Trajectories.EightTrench.pickUpTrenchBalls, OI.forwardHeading) // pick up trench balls
            .raceWith(new SpinBottomIntake(1)),
        new SwerveDriveWithOdometryProfiling(Trajectories.EightTrench.alignFromTrench, OI.forwardHeading) // move to viable space and align with target from trench
            .raceWith(new SpinShooterVelocity(REV_SPEED)),
        new SpinShooterLimelight().raceWith(new WaitCommand(SHOOTER_SHOOT_TIME)) // shoot
    );

    public static Command getAutonCommand() {
        return baselineAuto;
        // return three;
        // return trenchFiveAuton;
        // return rendezvousFiveAuton;
        // return rendezvousEightAuton;
        // return trenchEightAuton;
    }

    public enum AutonCommands {
        THREE(three), 
        FIVE_TRENCH(trenchFiveAuton), 
        FIVE_REND(rendezvousFiveAuton),
        EIGHT_TRENCH(trenchEightAuton),
        EIGHT_REND(rendezvousEightAuton);
        
        public SequentialCommandGroup value;

        private AutonCommands(SequentialCommandGroup value) {
            this.value = value;
        }
    }
}