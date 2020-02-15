package frc.robot.auto;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.OI;
import frc.robot.commands.bottomintake.SpinBottomIntake;
import frc.robot.commands.drivetrain.SwerveAutonAlignWithLimelight;
import frc.robot.commands.drivetrain.SwerveDriveWithOdometryProfiling;
import frc.robot.commands.drivetrain.SwerveAlignWithLimelight;
import frc.robot.commands.indexer.MoveBallsToShooter;
import frc.robot.commands.indexer.SpinIndexer;
import frc.robot.commands.shooter.SpinShooterLimelight;
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
    public enum StartingPosition {
        LEFT, MIDDLE, RIGHT
    }
    
    //Change this before the match.
    public static StartingPosition startingPosition = StartingPosition.LEFT;
    private static AutonCommands curAuton = AutonCommands.THREE; 

    //Brings the intake out, required at the start of every autonomous configuration
    private static InstantCommand intakeOut = new InstantCommand(
            () -> BottomIntake.getInstance().getSolenoid().set(BottomIntake.OUT), BottomIntake.getInstance());

    private static Rotation2d rendezvousTwoBallPickupHeading = Rotation2d.fromDegrees(120);
    private static final double REV_SPEED = 90;
    private static final double SHOOTER_REV_TIME = 2; // how long it takes to shoot all the balls
    private static final double SHOOTER_SHOOT_TIME = 4.0; // how long it takes to shoot all the balls

    private static final SequentialCommandGroup baselineAuto = new SequentialCommandGroup(
        new SwerveDriveWithOdometryProfiling(Trajectories.Baseline.moveForward, OI.forwardHeading)
    );

    //Shoot three pre-loaded balls
    private static final SequentialCommandGroup three = new SequentialCommandGroup(
        new SwerveDriveWithOdometryProfiling(Trajectories.Three.getStart(), OI.forwardHeading)
            .raceWith(new SpinShooterVelocity(REV_SPEED)),
        new SpinShooterLimelight().raceWith(new WaitCommand(SHOOTER_REV_TIME), new SwerveAlignWithLimelight()),
        new ParallelRaceGroup(new SpinShooterLimelight(), new MoveBallsToShooter(false), new WaitCommand(SHOOTER_SHOOT_TIME)));
   
    //pick up two balls from opponents trench and shoot all five
    private static final SequentialCommandGroup trenchFiveAuton = new SequentialCommandGroup(
        intakeOut,
        new SwerveDriveWithOdometryProfiling(Trajectories.FiveTrench.getStart(), OI.forwardHeading).raceWith(new SpinBottomIntake(1), new SpinIndexer(false)),
        new SwerveDriveWithOdometryProfiling(Trajectories.FiveTrench.pickupToShoot, OI.forwardHeading).raceWith(new SpinShooterVelocity(REV_SPEED)),
        new SpinShooterLimelight().raceWith(new WaitCommand(SHOOTER_REV_TIME), new SwerveAlignWithLimelight()),
        new SpinShooterLimelight().raceWith(new MoveBallsToShooter(false), new WaitCommand(SHOOTER_SHOOT_TIME)));

    //pick up two balls from rendezvous point and shoot all five
    private static final SequentialCommandGroup rendezvousFiveAuton = new SequentialCommandGroup(
        intakeOut,
        new SwerveDriveWithOdometryProfiling(Trajectories.FiveRendezvous.getStart(), OI.forwardHeading).raceWith(new SpinBottomIntake(1), new SpinIndexer(false)),
        new SpinBottomIntake(1).raceWith(new WaitCommand(0.5), new SpinIndexer(false)),
        new SwerveDriveWithOdometryProfiling(Trajectories.FiveRendezvous.pickupToShoot, OI.forwardHeading).raceWith(new SpinShooterVelocity(REV_SPEED)),
        new SpinShooterLimelight().raceWith(new WaitCommand(SHOOTER_REV_TIME), new SwerveAlignWithLimelight()),
        new SpinShooterLimelight().raceWith(new MoveBallsToShooter(false), new WaitCommand(SHOOTER_SHOOT_TIME)));
    
    //shoot all three pre-loaded balls, pick up five from the rendevous point, and shoot those five
    private static final SequentialCommandGroup rendezvousEightAuton = new SequentialCommandGroup(
        intakeOut,
        new SwerveDriveWithOdometryProfiling(Trajectories.EightRendezvous.getStart(), rendezvousTwoBallPickupHeading)
             .raceWith(new SpinBottomIntake(1), new SpinIndexer(false)),
        new ParallelRaceGroup(new SpinBottomIntake(1), new SpinIndexer(false), new WaitCommand(0.5)),
        new SwerveDriveWithOdometryProfiling(Trajectories.EightRendezvous.fromTwoBallRendezvousToThreeBallRendezvous, OI.forwardHeading)
             .raceWith(new SpinShooterVelocity(REV_SPEED)),
        new SwerveAutonAlignWithLimelight().raceWith(new SpinShooterVelocity(REV_SPEED)),
        new SpinShooterLimelight().raceWith(new WaitCommand(2), new MoveBallsToShooter(false)),
        new SwerveDriveWithOdometryProfiling(Trajectories.EightRendezvous.fromTwoBallRendezvousToThreeBallRendezvous, OI.forwardHeading).raceWith(new SpinBottomIntake(1),
             new SpinIndexer(false)),
        new ParallelRaceGroup(new SpinBottomIntake(1), new SpinIndexer(false), new WaitCommand(0.5)),
        new SpinShooterLimelight().raceWith(new WaitCommand(SHOOTER_REV_TIME), new SwerveAlignWithLimelight()),
        new SpinShooterLimelight().raceWith(new WaitCommand(2), new MoveBallsToShooter(false))
    );

    //shoot all three pre-loaded balls, pick up five from the alliance trench, and shoot those five
    private static final SequentialCommandGroup trenchEightAuton = new SequentialCommandGroup(
        intakeOut, 
        new SwerveDriveWithOdometryProfiling(Trajectories.EightTrench.getStart(), OI.forwardHeading)  // move to shooting location
            .raceWith(new SpinShooterVelocity(REV_SPEED)),
        new SpinShooterLimelight().raceWith(new WaitCommand(SHOOTER_REV_TIME), new SwerveAlignWithLimelight()),
        new SwerveDriveWithOdometryProfiling(Trajectories.EightTrench.pickUpTrenchBalls, OI.forwardHeading) // pick up trench balls
            .raceWith(new SpinBottomIntake(1), new SpinIndexer(false)),
        new SpinBottomIntake(1).raceWith(new SpinIndexer(false), new WaitCommand(0.5)),
        new SwerveDriveWithOdometryProfiling(Trajectories.EightTrench.alignFromTrench, OI.forwardHeading) // move to viable space and align with target from trench
            .raceWith(new SpinShooterVelocity(REV_SPEED)),
        new SpinShooterLimelight().raceWith(new WaitCommand(SHOOTER_REV_TIME), new SwerveAlignWithLimelight())
    );

    private static final SequentialCommandGroup trenchToRendevousTenBallAuton = new SequentialCommandGroup(
      //Move out the intake
      intakeOut,
      
      //Move to the opponent's trench and intake two balls
      new SwerveDriveWithOdometryProfiling(Trajectories.TenTwoTrenchRendezvous.getStart(), OI.forwardHeading
      ).raceWith(new SpinBottomIntake(1), new SpinIndexer(false)),
      //Spin for extra 0.5 seconds for safety
      new SpinBottomIntake(1).raceWith(new SpinIndexer(false), new WaitCommand(0.5)),
      //Move to the shooting position and align
      new SwerveDriveWithOdometryProfiling(Trajectories.TenTwoTrenchRendezvous.firstPickupToShootingPosition, OI.forwardHeading).raceWith(new SpinShooterVelocity(REV_SPEED)),
      //Shoot the five balls
      new SpinShooterLimelight().raceWith(new SwerveAlignWithLimelight(), new WaitCommand(SHOOTER_REV_TIME)),
      new SpinShooterLimelight().raceWith(new MoveBallsToShooter(false), new WaitCommand(SHOOTER_SHOOT_TIME)),
      //Move to the rendezvous and intake two balls
      new SwerveDriveWithOdometryProfiling(Trajectories.TenTwoTrenchRendezvous.shootingToRendezvous, OI.forwardHeading)
        .raceWith(new SpinBottomIntake(1), new SpinIndexer(false)),
      //Spin intake for 0.5 s for safety
        new SpinBottomIntake(1).raceWith(new SpinIndexer(false), new WaitCommand(0.5)),
      //Move from the rendezvous to right next to the opponent trench to get ready to intake balls
      new SwerveDriveWithOdometryProfiling(Trajectories.TenTwoTrenchRendezvous.rendezvousToPreTrench, OI.forwardHeading)
        .raceWith(new SpinBottomIntake(1), new SpinIndexer(false)),
      //Spin intake for 0.5s for safety
      new SpinBottomIntake(1).raceWith(new SpinIndexer(false), new WaitCommand(0.5)),
      //Move from the opponent trench to the shooting position and rev the shooter in the meantime
      new SwerveDriveWithOdometryProfiling(Trajectories.TenTwoTrenchRendezvous.secondTrenchToShooting, OI.forwardHeading)
        .raceWith(new SpinShooterVelocity(REV_SPEED)),
      //Shoot and index the balls
      new SpinShooterLimelight().raceWith(new SwerveAlignWithLimelight(), new WaitCommand(SHOOTER_REV_TIME)),
      new SpinShooterLimelight().raceWith(new MoveBallsToShooter(false), new WaitCommand(SHOOTER_SHOOT_TIME)) 
    );

    private static final SequentialCommandGroup speedRendezvousToTrenchTen = new SequentialCommandGroup(
      //Move out the intake
      intakeOut,
      
      //Move to the opponent's trench and intake two balls
      new SwerveDriveWithOdometryProfiling(Trajectories.SpeedTenBallAuton.getStart(), OI.forwardHeading
      ).raceWith(new SpinBottomIntake(1), new SpinIndexer(false)),
      //Spin for extra 0.5 seconds for safety
      new SpinBottomIntake(1).raceWith(new SpinIndexer(false), new WaitCommand(0.5)),
      //Move to the shooting position and align
      new SwerveDriveWithOdometryProfiling(Trajectories.SpeedTenBallAuton.rendezvousToShooting, OI.forwardHeading).raceWith(new SpinShooterVelocity(REV_SPEED)),
      //Shoot the five balls
      new SpinShooterLimelight().raceWith(new SwerveAlignWithLimelight(), new WaitCommand(SHOOTER_REV_TIME)),
      new SpinShooterLimelight().raceWith(new MoveBallsToShooter(false), new WaitCommand(SHOOTER_SHOOT_TIME)),
      //Move to the rendezvous and intake two balls
      new SwerveDriveWithOdometryProfiling(Trajectories.SpeedTenBallAuton.shootingToTrench, OI.forwardHeading)
        .raceWith(new SpinBottomIntake(1), new SpinIndexer(false)),
      //Spin intake for 0.5 s for safety
        new SpinBottomIntake(1).raceWith(new SpinIndexer(false), new WaitCommand(0.5)),
      //Move from the opponent trench to the shooting position and rev the shooter in the meantime
      new SwerveDriveWithOdometryProfiling(Trajectories.SpeedTenBallAuton.trenchToShooting, OI.forwardHeading)
        .raceWith(new SpinShooterVelocity(REV_SPEED)),
      //Shoot and index the balls
      new SpinShooterLimelight().raceWith(new SwerveAlignWithLimelight(), new WaitCommand(SHOOTER_REV_TIME)),
      new SpinShooterLimelight().raceWith(new MoveBallsToShooter(false), new WaitCommand(SHOOTER_SHOOT_TIME)) 
    );

    public static CommandBase getAutonCommand() {
        return curAuton.value;
    }

    public enum AutonCommands {
        BASELINE(baselineAuto),
        THREE(three), 
        FIVE_TRENCH(trenchFiveAuton), 
        FIVE_REND(rendezvousFiveAuton),
        EIGHT_TRENCH(trenchEightAuton),
        EIGHT_REND(rendezvousEightAuton),
        TEN_TWO_TRENCH(trenchToRendevousTenBallAuton),
        TEN_SPEED(speedRendezvousToTrenchTen);
        
        public SequentialCommandGroup value;

        private AutonCommands(SequentialCommandGroup value) {
            this.value = value;
        }
    }
}