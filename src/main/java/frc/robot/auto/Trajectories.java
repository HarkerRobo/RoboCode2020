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
 */
public class Trajectories {
    // private static SwerveDriveKinematicsConstraint constraint = new SwerveDriveKinematicsConstraint(
    //         Drivetrain.getInstance().getKinematics(), Drivetrain.MAX_DRIVE_VELOCITY);

    public static TrajectoryConfig config = new TrajectoryConfig(Drivetrain.MP_MAX_DRIVE_VELOCITY,
        Drivetrain.MP_MAX_DRIVE_ACCELERATION).setKinematics(Drivetrain.getInstance().getKinematics());
    // .addConstraint(constraint);

    /**
     * Trajectories used in various three ball autonomous configurations
     */
    public static class Three {
        public static Trajectory leftStartingAutonThree = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
                        new Pose2d(4, 1, Rotation2d.fromDegrees(0))),
                config);

        public static Trajectory centerStartingAutonThree = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
                        new Pose2d(0, 1, Rotation2d.fromDegrees(0))),
                config);

        public static Trajectory rightStartingAutonThree = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
                        new Pose2d(1, -5, Rotation2d.fromDegrees(0))),
                config);
    }

    public static class Five {
        public static Trajectory leftStarting = TrajectoryGenerator.generateTrajectory( 
                List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
                        new Pose2d(-3, 0, Rotation2d.fromDegrees(180))),
                config);
        

        public static Trajectory pickupToShoot = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                        new Pose2d(5, 3, Rotation2d.fromDegrees(0))),
                config();
                
    }

    /**
     * Trajectories used in various eight ball autonomous configurations
     */
    public static class Eight {
        public static Trajectory leftStartingAutonEight = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(270)), 
                        new Pose2d(4, -5, Rotation2d.fromDegrees(0))),
                config);

        public static Trajectory middleStartingAutonEight = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(270)), 
                        new Pose2d(2, -5, Rotation2d.fromDegrees(0))),
                config);

        public static Trajectory rightStartingAutonEight = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(270)), 
                        new Pose2d(1, -5, Rotation2d.fromDegrees(270))),
                config);
    }
    
    /**
     * Trajectories used in our eight ball trench auton
     */
    public static class EightTrench {
        public static Trajectory middleStart = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(180)), 
                        new Pose2d(3, -2, Rotation2d.fromDegrees(0))),
                config);

        public static Trajectory rightStart = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(180)), 
                        new Pose2d(-2, -2, Rotation2d.fromDegrees(0))),
                config);
    }

    /**
     * Trajectories used in various ten ball autonomous configurations
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