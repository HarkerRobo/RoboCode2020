package frc.robot.auto;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.OI;
import frc.robot.commands.bottomintake.SpinIntakeVelocity;
import frc.robot.commands.drivetrain.SwerveAutonAlignWithLimelight;
import frc.robot.commands.drivetrain.SwerveDriveWithOdometryProfiling;
import frc.robot.commands.drivetrain.SwerveAlignWithLimelight;
import frc.robot.commands.indexer.MoveBallsToShooter;
import frc.robot.commands.indexer.SpinIndexer;
import frc.robot.commands.shooter.SpinShooterLimelight;
import frc.robot.commands.shooter.SpinShooterVelocity;
import frc.robot.subsystems.BottomIntake;
import frc.robot.subsystems.Shooter;

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
    public static StartingPosition startingPosition = StartingPosition.RIGHT;
    // public static AutonCommands curAuton = AutonCommands.EIGHT_TRENCH; 

    //Brings the intake out, required at the start of every autonomous configuration
    // private static InstantCommand intakeOut = new InstantCommand(
    //         () -> BottomIntake.getInstance().getSolenoid().set(BottomIntake.OUT), BottomIntake.getInstance());

    private static Rotation2d rendezvousTwoBallPickupHeading = Rotation2d.fromDegrees(120);
    private static final double REV_SPEED = 90;
    private static final double SHOOTER_REV_TIME = 0.7;
    private static final double SHOOTER_SHOOT_TIME = 4.0; // how long it takes to shoot all the balls
    private static final Rotation2d leftRendezvousHeading= Rotation2d.fromDegrees(-26.5);
    private static final Rotation2d rightRendezvousHeading= Rotation2d.fromDegrees(65);

    private static final SequentialCommandGroup baselineAuto = new SequentialCommandGroup(
        new SwerveDriveWithOdometryProfiling(Trajectories.Baseline.moveForward, OI.forwardHeading)
    );

    private static final SequentialCommandGroup test = new SequentialCommandGroup(
        new SwerveDriveWithOdometryProfiling(Trajectories.Test.verticalTrajectory, OI.forwardHeading)
            .raceWith(new SpinIntakeVelocity(0.6))
    );

    //Shoot three pre-loaded balls
    private static final SequentialCommandGroup three = new SequentialCommandGroup(
        new InstantCommand(() -> Shooter.getInstance().getSolenoid().set(Shooter.HIGH_ANGLE)),    
        new SwerveDriveWithOdometryProfiling(Trajectories.Three.getStart(), OI.forwardHeading)
             .raceWith(new SpinShooterVelocity(58)),
         new SpinShooterVelocity(58).raceWith(new WaitCommand(SHOOTER_REV_TIME)),
         new ParallelRaceGroup(new SpinShooterVelocity(58), new MoveBallsToShooter(false), new WaitCommand(SHOOTER_SHOOT_TIME))
    );
   
    // private static final SequentialCommandGroup threeFlex = new SequentialCommandGroup(
    //     new InstantCommand(() -> Shooter.getInstance().getSolenoid().set(Shooter.HIGH_ANGLE)),    
    //     new SwerveDriveWithOdometryProfiling(Trajectories.Three.flexStarting, OI.forwardHeading)
    //        .raceWith(new SpinShooterVelocity(58)),
    //     new SpinShooterVelocity(58).raceWith(new WaitCommand(SHOOTER_REV_TIME), new SwerveAutonAlignWithLimelight()),
    //     new ParallelRaceGroup(new SpinShooterLimelight(), new MoveBallsToShooter(false), new WaitCommand(SHOOTER_SHOOT_TIME))
    // );
    
    //pick up two balls from opponents trench and shoot all five
    private static final SequentialCommandGroup trenchFiveAuton = new SequentialCommandGroup(
        new InstantCommand(() -> Shooter.getInstance().getSolenoid().set(Shooter.LOW_ANGLE)),
        new InstantCommand(() -> BottomIntake.getInstance().getSolenoid().set(BottomIntake.OUT), BottomIntake.getInstance()),
        new SwerveDriveWithOdometryProfiling(Trajectories.FiveTrench.getStart(), OI.forwardHeading)
            .raceWith(new SpinIntakeVelocity(0.6), new SpinIndexer(0.8, false)),
        new SwerveDriveWithOdometryProfiling(Trajectories.FiveTrench.pickupToShoot, OI.forwardHeading)
            .raceWith(new SpinShooterVelocity(REV_SPEED), new SpinIndexer(0.6, false)),
        new SpinShooterLimelight().raceWith(new WaitCommand(SHOOTER_REV_TIME), new SwerveAlignWithLimelight()),
        new SpinShooterLimelight().raceWith(new MoveBallsToShooter(false), new WaitCommand(SHOOTER_SHOOT_TIME)));

    //pick up two balls from rendezvous point and shoot all five
    private static final SequentialCommandGroup rendezvousFiveAuton = new SequentialCommandGroup(
        // new InstantCommand(() -> BottomIntake.getInstance().getSolenoid().set(BottomIntake.OUT), BottomIntake.getInstance()),
        // new InstantCommand(() -> Shooter.getInstance().getSolenoid().set(Shooter.LOW_ANGLE), Shooter.getInstance()),
        new SwerveDriveWithOdometryProfiling(Trajectories.FiveRendezvous.getStart(), leftRendezvousHeading)
            .raceWith(new SpinIntakeVelocity(0.6), new SpinIndexer(0.6, false)),
        // new SpinIntakeVelocity(0.6).raceWith(new WaitCommand(0.5), new SpinIndexer(0.8, false)),
        new WaitCommand(1),
        new SwerveDriveWithOdometryProfiling(Trajectories.FiveRendezvous.pickupToShoot, OI.forwardHeading)
            .raceWith(new SpinShooterVelocity(REV_SPEED), new SpinIndexer(0.6, false)
        ));
        // new SpinShooterLimelight().raceWith(new WaitCommand(SHOOTER_REV_TIME), new SwerveAlignWithLimelight()),
        // new SpinShooterLimelight().raceWith(new MoveBallsToShooter(false), new WaitCommand(SHOOTER_SHOOT_TIME)));
    
    //shoot all three pre-loaded balls, pick up five from the rendevous point, and shoot those five
    private static final SequentialCommandGroup rendezvousEightAuton = new SequentialCommandGroup(
        new InstantCommand(
            () -> BottomIntake.getInstance().getSolenoid().set(BottomIntake.OUT), BottomIntake.getInstance()),
        new SwerveDriveWithOdometryProfiling(Trajectories.EightRendezvous.getStart(), rendezvousTwoBallPickupHeading)
             .raceWith(new SpinIntakeVelocity(1), new SpinIndexer(1, false)),
        new ParallelRaceGroup(new SpinIntakeVelocity(1), new SpinIndexer(1, false), new WaitCommand(0.5)),
        new SwerveDriveWithOdometryProfiling(Trajectories.EightRendezvous.fromTwoBallRendezvousToThreeBallRendezvous, OI.forwardHeading)
             .raceWith(new SpinShooterVelocity(REV_SPEED)),
        new SwerveAutonAlignWithLimelight().raceWith(new SpinShooterVelocity(REV_SPEED)),
        new SpinShooterLimelight().raceWith(new WaitCommand(2), new MoveBallsToShooter(false)),
        new SwerveDriveWithOdometryProfiling(Trajectories.EightRendezvous.fromTwoBallRendezvousToThreeBallRendezvous, OI.forwardHeading).raceWith(new SpinIntakeVelocity(1),
             new SpinIndexer(1, false)),
        new ParallelRaceGroup(new SpinIntakeVelocity(1), new SpinIndexer(1, false), new WaitCommand(0.5)),
        new SpinShooterLimelight().raceWith(new WaitCommand(SHOOTER_REV_TIME), new SwerveAlignWithLimelight()),
        new SpinShooterLimelight().raceWith(new WaitCommand(2), new MoveBallsToShooter(false))
    );

    //shoot all three pre-loaded balls, pick up five from the alliance trench, and shoot those five
    private static final SequentialCommandGroup trenchEightAuton = new SequentialCommandGroup(
        new InstantCommand(() -> Shooter.getInstance().getSolenoid().set(Shooter.LOW_ANGLE), Shooter.getInstance()),
        new InstantCommand(() -> BottomIntake.getInstance().getSolenoid().set(BottomIntake.OUT), BottomIntake.getInstance()), 
        new SwerveDriveWithOdometryProfiling(Trajectories.EightTrench.getStart(), OI.forwardHeading)  // move to shooting location
            .raceWith(new SpinShooterVelocity(REV_SPEED)),
        new SpinShooterLimelight().raceWith(new WaitCommand(SHOOTER_SHOOT_TIME), new SwerveAlignWithLimelight(), new MoveBallsToShooter(false)),
        new SwerveDriveWithOdometryProfiling(Trajectories.EightTrench.pickUpTrenchBalls, OI.forwardHeading) // pick up trench balls
            .raceWith(new SpinIntakeVelocity(0.6), new SpinIndexer(0.6, false)),
        new SwerveDriveWithOdometryProfiling(Trajectories.EightTrench.alignFromTrench, OI.forwardHeading) // move to viable space and align with target from trench
            .raceWith(new SpinShooterVelocity(REV_SPEED), new SpinIndexer(0.6, false)),
        new SpinShooterLimelight().raceWith(new WaitCommand(SHOOTER_SHOOT_TIME), new SwerveAlignWithLimelight(), new MoveBallsToShooter(false))
    );

    private static final SequentialCommandGroup trenchToRendevousTenBallAuton = new SequentialCommandGroup(
        //Move out the intake
        new InstantCommand(() -> BottomIntake.getInstance().getSolenoid().set(BottomIntake.OUT), BottomIntake.getInstance()),
        //Move to the opponent's trench and intake two balls
        new SwerveDriveWithOdometryProfiling(Trajectories.TenTwoTrenchRendezvous.getStart(), OI.forwardHeading)
            .raceWith(new SpinIntakeVelocity(1), new SpinIndexer(1, false)),
        //Spin for extra 0.5 seconds for safety
        new SpinIntakeVelocity(1).raceWith(new SpinIndexer(1, false), new WaitCommand(0.5)),
        //Move to the shooting position and align
        new SwerveDriveWithOdometryProfiling(Trajectories.TenTwoTrenchRendezvous.firstPickupToShootingPosition, OI.forwardHeading).raceWith(new SpinShooterVelocity(REV_SPEED)),
        //Shoot the five balls
        new SpinShooterLimelight().raceWith(new SwerveAlignWithLimelight(), new WaitCommand(SHOOTER_REV_TIME)),
        new SpinShooterLimelight().raceWith(new MoveBallsToShooter(false), new WaitCommand(SHOOTER_SHOOT_TIME)),
        //Move to the rendezvous and intake two balls
        new SwerveDriveWithOdometryProfiling(Trajectories.TenTwoTrenchRendezvous.shootingToRendezvous, OI.forwardHeading)
            .raceWith(new SpinIntakeVelocity(1), new SpinIndexer(1, false)),
        //Spin intake for 0.5 s for safety
        new SpinIntakeVelocity(1).raceWith(new SpinIndexer(1, false), new WaitCommand(0.5)),
        //Move from the rendezvous to right next to the opponent trench to get ready to intake balls
        new SwerveDriveWithOdometryProfiling(Trajectories.TenTwoTrenchRendezvous.rendezvousToPreTrench, OI.forwardHeading)
            .raceWith(new SpinIntakeVelocity(1), new SpinIndexer(1, false)),
        //Spin intake for 0.5s for safety
        new SpinIntakeVelocity(1).raceWith(new SpinIndexer(1, false), new WaitCommand(0.5)),
        //Move from the opponent trench to the shooting position and rev the shooter in the meantime
        new SwerveDriveWithOdometryProfiling(Trajectories.TenTwoTrenchRendezvous.secondTrenchToShooting, OI.forwardHeading)
            .raceWith(new SpinShooterVelocity(REV_SPEED)),
        //Shoot and index the balls
        new SpinShooterLimelight().raceWith(new SwerveAlignWithLimelight(), new WaitCommand(SHOOTER_REV_TIME)),
        new SpinShooterLimelight().raceWith(new MoveBallsToShooter(false), new WaitCommand(SHOOTER_SHOOT_TIME)) 
    );

    private static final SequentialCommandGroup speedRendezvousToTrenchTen = new SequentialCommandGroup(
        //Move out the intake
        new InstantCommand(() -> BottomIntake.getInstance().getSolenoid().set(BottomIntake.OUT), BottomIntake.getInstance()),
        //Move to the opponent's trench and intake two balls
        new SwerveDriveWithOdometryProfiling(Trajectories.SpeedTenBallAuton.getStart(), rightRendezvousHeading)
            .raceWith(new SpinIntakeVelocity(0.6), new SpinIndexer(0.6, false)),
        //Spin for extra 0.5 seconds for safety
        new SpinIntakeVelocity(0.6).raceWith(new SpinIndexer(0.6, false), new WaitCommand(0.5)),
        //Move to the shooting position and align
        new SwerveDriveWithOdometryProfiling(Trajectories.SpeedTenBallAuton.rendezvousToShooting, OI.forwardHeading).raceWith(new SpinShooterVelocity(REV_SPEED)),
        //Shoot the five balls
        new SpinShooterLimelight().raceWith(new SwerveAlignWithLimelight(), new WaitCommand(SHOOTER_REV_TIME)),
        new SpinShooterLimelight().raceWith(new MoveBallsToShooter(false), new WaitCommand(SHOOTER_SHOOT_TIME)),
        //Move to the rendezvous and intake two balls
        new SwerveDriveWithOdometryProfiling(Trajectories.SpeedTenBallAuton.shootingToTrench, OI.forwardHeading)
            .raceWith(new SpinIntakeVelocity(1), new SpinIndexer(1, false)),
        //Spin intake for 0.5 s for safety
        new SpinIntakeVelocity(1).raceWith(new SpinIndexer(1, false), new WaitCommand(0.5)),
        //Move from the opponent trench to the shooting position and rev the shooter in the meantime
        new SwerveDriveWithOdometryProfiling(Trajectories.SpeedTenBallAuton.trenchToShooting, OI.forwardHeading)
            .raceWith(new SpinShooterVelocity(REV_SPEED)),
        //Shoot and index the balls
        new SpinShooterLimelight().raceWith(new SwerveAlignWithLimelight(), new WaitCommand(SHOOTER_REV_TIME)),
        new SpinShooterLimelight().raceWith(new MoveBallsToShooter(false), new WaitCommand(SHOOTER_SHOOT_TIME)) 
    );

    public static CommandBase getAutonCommand() {
        return trenchEightAuton/*curAuton.value*/;
    }

    public static enum AutonCommands {
        BASELINE(baselineAuto),
        THREE(three), 
        FIVE_TRENCH(trenchFiveAuton), 
        FIVE_REND(rendezvousFiveAuton),
        EIGHT_TRENCH(trenchEightAuton),
        EIGHT_REND(rendezvousEightAuton),
        TEN_TWO_TRENCH(trenchToRendevousTenBallAuton),
        TEN_SPEED(speedRendezvousToTrenchTen);
        
        public SequentialCommandGroup value;

        private AutonCommands(SequentialCommandGroup val) {
            System.out.println(val);
            this.value = val;
        }
    }
}