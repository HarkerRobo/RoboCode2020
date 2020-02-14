package frc.robot.auto;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.Drivetrain;

/**
 * Contains all Trajectories used for various autonomous configurations.
 * 
 * @author Jatin Kohli
 * @author Shahzeb Lakhani
 * @author Ada Praun-Petrovic
 * @author Chirag Kaushik
 * @author Angela Jia
 * @since February 12, 2020
 */
public class Trajectories {
    public static TrajectoryConfig config = new TrajectoryConfig(Drivetrain.MP_MAX_DRIVE_VELOCITY,
            Drivetrain.MP_MAX_DRIVE_ACCELERATION).setKinematics(Drivetrain.getInstance().getKinematics());

    public static TrajectoryConfig slowConfig = new TrajectoryConfig(Drivetrain.MP_MAX_DRIVE_VELOCITY/2,
            Drivetrain.MP_MAX_DRIVE_ACCELERATION/2).setKinematics(Drivetrain.getInstance().getKinematics());
    // .addConstraint(constraint);

    public static class Baseline {
        public static Trajectory moveForward = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(90)), 
                new Pose2d(0, 2, Rotation2d.fromDegrees(90))),
            config);
    }

    /**
     * Trajectories used in three ball configurations
     */
    public static class Three {
        public static Trajectory leftStarting = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
                    new Pose2d(4, 1, Rotation2d.fromDegrees(0))),
            config);

        public static Trajectory middleStarting = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
                    new Pose2d(0, 1, Rotation2d.fromDegrees(0))),
            config);

        public static Trajectory rightStarting = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
                    new Pose2d(1, -5, Rotation2d.fromDegrees(0))),
            config);
    }

    /**
     * Trajectories used in five ball configurations that pickup two balls from the opponent's trench
     */
    public static class FiveTrench {
        public static Trajectory leftStarting = TrajectoryGenerator.generateTrajectory( 
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(270)),
                new Pose2d(0, -3, Rotation2d.fromDegrees(270))),
            config);
        
        public static Trajectory middleStarting = TrajectoryGenerator.generateTrajectory( 
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(270)),
                new Pose2d(-5, -3, Rotation2d.fromDegrees(270))), 
            config);
                
        public static Trajectory rightStarting = TrajectoryGenerator.generateTrajectory( 
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(90)),
                new Pose2d(-5, 1.5, Rotation2d.fromDegrees(0)),
                new Pose2d(-10, -3, Rotation2d.fromDegrees(270))),
            config);

        public static Trajectory pickupToShoot = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(90)),
                new Pose2d(7, 3, Rotation2d.fromDegrees(90))),
            config);
    }

    /**
     * Trajectories used in five ball configurations that pickup two balls from the rendevous point
     */
    public static class FiveRendezvous {
        public static Trajectory leftStarting = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(90)), 
                new Pose2d(5, 1, Rotation2d.fromDegrees(0)),
                new Pose2d(5, -3, Rotation2d.fromDegrees(270)),
                new Pose2d(4.5, -3.5, Rotation2d.fromDegrees(252))),
            config);

        public static Trajectory middleStarting = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(270)), 
                new Pose2d(2, -2, Rotation2d.fromDegrees(0)),
                new Pose2d(5, -3, Rotation2d.fromDegrees(270)),
                new Pose2d(4.5, -3.5, Rotation2d.fromDegrees(252))),
            config);
            public static Trajectory rightStarting = TrajectoryGenerator.generateTrajectory( 
                List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(90)),
                    new Pose2d(-5, 1.5, Rotation2d.fromDegrees(0)),
                    new Pose2d(-10, -3, Rotation2d.fromDegrees(270))),
                config);
                public static Trajectory pickupToShoot = TrajectoryGenerator.generateTrajectory(
                    List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(90)),
                        new Pose2d(7, 3, Rotation2d.fromDegrees(90))),
                    config);
    }

    /**
     * Trajectories used in eight ball configurations that pickup five balls from the alliance's trench
     */
    public static class EightTrench {
        public static Trajectory leftStarting = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(270)), 
                new Pose2d(4, -5, Rotation2d.fromDegrees(0))),
            config);

        public static Trajectory middleStarting = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(180)), 
                new Pose2d(3, -2, Rotation2d.fromDegrees(0))),
            config);

        public static Trajectory rightStarting = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(180)), 
                new Pose2d(-2, -2, Rotation2d.fromDegrees(0))),
            config);
        
        public static Trajectory pickUpTrenchBalls = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
                    new Pose2d(0, -4, Rotation2d.fromDegrees(180))),
            config);
        
        public static Trajectory alignFromTrench = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
                    new Pose2d(0, 1, Rotation2d.fromDegrees(180))), 
            config);
    }

    /**
     * Trajectories used in eight ball configurations that pickup five balls from the rendevous point, 
     * starting to the left of the rendezvous point (from driver station perspective)
     */
    public static class EightRendezvous {
        public static Trajectory leftStarting = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(270)), 
                    new Pose2d(4, -5, Rotation2d.fromDegrees(0))),
                config);
                
        public static Trajectory middleStarting = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(180)), 
                new Pose2d(2, -2, Rotation2d.fromDegrees(180)),
                new Pose2d(0, -4, Rotation2d.fromDegrees(120))),
            config);

        public static Trajectory rightStarting = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(135)), 
                new Pose2d(-3, -3, Rotation2d.fromDegrees(135))),
            config);
    
        public static Trajectory fromTwoBallRendezvousToThreeBallRendezvous = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(90)), 
                new Pose2d(-1, 2, Rotation2d.fromDegrees(180)),
                new Pose2d(-2, 0, Rotation2d.fromDegrees(270))),
            slowConfig);
    }
    
    /**
     * Trajectories used in eight ball configurations that pickup five balls from the rendevous point, 
     * starting to the right of the rendezvous point (from driver station perspective)
     */
    public static class EightReverseRendezvous {
        public static Trajectory leftStart = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(-21.77)), 
                new Pose2d(5, -2, Rotation2d.fromDegrees(-21.77))
           ),  config);
    
        public static Trajectory fromThreeBallRendezvousToTwoBallRendezvous = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(90)), 
                new Pose2d(1, 2, Rotation2d.fromDegrees(0)),
                new Pose2d(2, 0, Rotation2d.fromDegrees(270))),
            slowConfig);
    }

    /**
     * Trajectories used in ten ball configurations
     */
    public static class Ten {
        public static Trajectory leftStartingAutonTen = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(90)), 
                new Pose2d(5, 0, Rotation2d.fromDegrees(270))),
            config);
    
        public static Trajectory middleStartingAutonTen = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(90)), 
                new Pose2d(3, 0, Rotation2d.fromDegrees(270))),
            config);
    
        public static Trajectory rightStartingAutonTen = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(90)), 
                new Pose2d(1, 0, Rotation2d.fromDegrees(270))),
            config);
    
        public static Trajectory firstPickupToShootingPositionTen = TrajectoryGenerator.generateTrajectory(
           List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
                new Pose2d(-2, 5, Rotation2d.fromDegrees(180))),
           config);
    }
    
    /**
     * Trajectories used in all autonomous configurations
     */
    public static class Common {
        public static Trajectory pickupFive = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(270)), 
                new Pose2d(1, -2, Rotation2d.fromDegrees(270))),
            config);
                    
        public static Trajectory fromPickupFiveToShootingPosition = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(153.43)), 
                new Pose2d(-1, 2, Rotation2d.fromDegrees(153.43))),
            config);
    }
    
    /**
     * Trajectories used for testing
     */
    public static class Test {
        public static Trajectory horizontalTrajectory = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
                new Pose2d(3, 0, Rotation2d.fromDegrees(0))),
            config);

        public static Trajectory verticalTrajectory = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(90)), 
                new Pose2d(0, 1, Rotation2d.fromDegrees(90))),
            config);

        public static Trajectory initiationToBackTest = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
                new Pose2d(2, 0, Rotation2d.fromDegrees(0))),
            config);

        public static Trajectory circle = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(2, 2, Rotation2d.fromDegrees(90)), 
                new Pose2d(0, 4, Rotation2d.fromDegrees(180)),
                new Pose2d(-2, 2, Rotation2d.fromDegrees(270)), 
                new Pose2d(0, 0, Rotation2d.fromDegrees(0))), 
            config);

        public static Trajectory heart = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(2, 2.5, Rotation2d.fromDegrees(90)),
                new Pose2d(1, 4, Rotation2d.fromDegrees(210)),
                new Pose2d(0, 3, Rotation2d.fromDegrees(180)),
                new Pose2d(-1, 4, Rotation2d.fromDegrees(150)),
                new Pose2d(-2, 2.5, Rotation2d.fromDegrees(270)),
                new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
            config);
    }
}