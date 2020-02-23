package frc.robot.auto;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.auto.Autons.StartingPosition;

/**
 * Contains all Trajectories used for various autonomous configurations.
 * 
 * @author Jatin Kohli
 * @author Shahzeb Lakhani
 * @author Ada Praun-Petrovic
 * @author Chirag Kaushik
 * @author Angela Jia
 * @author Anirudh Kotamraju
 * @since February 12, 2020
 */
public class Trajectories {
    public static TrajectoryConfig config = new TrajectoryConfig(Drivetrain.MP_MAX_DRIVE_VELOCITY,
            Drivetrain.MP_MAX_DRIVE_ACCELERATION).setKinematics(Drivetrain.getInstance().getKinematics());

    public static TrajectoryConfig slowConfig = new TrajectoryConfig(Drivetrain.MP_MAX_DRIVE_VELOCITY/2,
            Drivetrain.MP_MAX_DRIVE_ACCELERATION/2).setKinematics(Drivetrain.getInstance().getKinematics());

    /**
     * Trajectory used for moving off of the initiation line
     */
    public static class Baseline {
        /**
         * https://imgur.com/a/EYvAJjR
         * 
         * (Can start anywhere on initiation line with 2 meters of space in front of robot)
         */
        public static Trajectory moveForward = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(90)), 
                    new Pose2d(0, 2, Rotation2d.fromDegrees(90))),
            config);

        /**
         * https://imgur.com/a/Gzz3fAL
         * 
         * (Can start anywhere on initiation line with 2 meters of space behind robot)
         */
        public static Trajectory moveBackward = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(270)),
                    new Pose2d(0, -2, Rotation2d.fromDegrees(270))),
            config);
    }

    /*
     * Trajectories used in three ball configurations
     */
   public static class Three extends AutonTrajectory {
       /**
        * https://imgur.com/a/rUSiKb4
        */
       public static Trajectory leftStarting = TrajectoryGenerator.generateTrajectory(
           List.of(new Pose2d(1.60 * 0.3048, 42.00 * 0.3048, Rotation2d.fromDegrees(30)), 
                   new Pose2d(19.00 * 0.3048, 50.20 * 0.3048, Rotation2d.fromDegrees(90))),
           config);

       /**
        * https://imgur.com/a/KnwF104
        */
       public static Trajectory middleStarting = TrajectoryGenerator.generateTrajectory(
           List.of(new Pose2d(8.50 * 0.3048, 42.00 * 0.3048, Rotation2d.fromDegrees(45)), 
                   new Pose2d(19.00 * 0.3048, 50.20 * 0.3048, Rotation2d.fromDegrees(90))), 
           config);

       /**
        * https://imgur.com/a/eLtqCyb
        */
       public static Trajectory rightStarting = TrajectoryGenerator.generateTrajectory(
           List.of(new Pose2d(25.40 * 0.3048, 42.00 * 0.3048, Rotation2d.fromDegrees(-45)), 
                   new Pose2d(19.00 * 0.3048, 50.20 * 0.3048, Rotation2d.fromDegrees(90))),
           config);
       
       public static Trajectory getLeft() { return leftStarting; }
       public static Trajectory getMiddle() { return middleStarting; }
       public static Trajectory getRight() { return rightStarting; }
   }

    /**
     * Trajectories used in five ball configurations that pickup two balls from the opponent's trench
     */
    public static class FiveTrench extends AutonTrajectory {
        /**
         * https://imgur.com/a/Cr3jWYP
         */
        public static Trajectory leftStarting = TrajectoryGenerator.generateTrajectory( 
            List.of(new Pose2d(1.60 * 0.3048, 42.00 * 0.3048, Rotation2d.fromDegrees(180)),
                    new Pose2d(2.49 * 0.3048, 32.50 * 0.3048, Rotation2d.fromDegrees(180))),
            config);
        
        /**
         * https://imgur.com/a/xulkyU1
         */
        public static Trajectory middleStarting = TrajectoryGenerator.generateTrajectory( 
            List.of(new Pose2d(8.50 * 0.3048, 42.00 * 0.3048, Rotation2d.fromDegrees(180)),
                    new Pose2d(2.49 * 0.3048, 32.50 * 0.3048, Rotation2d.fromDegrees(180))),
            config);

        /**
         * https://imgur.com/a/wSmQIoM
         */
        public static Trajectory pickupToShoot = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(2.49 * 0.3048, 32.50 * 0.3048, Rotation2d.fromDegrees(0)),
                    new Pose2d(19.00 * 0.3048, 50.20 * 0.3048, Rotation2d.fromDegrees(0))),
            config);

        public static Trajectory getLeft() { return leftStarting; }
        public static Trajectory getMiddle() { return middleStarting; }
        // public static Trajectory getRight() { return middleStarting; }
    }

    /**
     * Trajectories used in five ball configurations that pickup two balls from the rendevous point
     */
    public static class FiveRendezvous extends AutonTrajectory {
        /**
         * https://imgur.com/a/QC43cB7
         */
        public static Trajectory middleStarting = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(8.50 * 0.3048, 42.00 * 0.3048, Rotation2d.fromDegrees(270)),
                    new Pose2d(19.35 * 0.3048, 36.72 * 0.3048, Rotation2d.fromDegrees(314)), //136.47
                    new Pose2d(18.41 * 0.3048, 31.95 * 0.3048, Rotation2d.fromDegrees(201))), //-110.556
            config);
            
        /**
         * https://imgur.com/a/NiTftF8
         */
        public static Trajectory rightStarting = TrajectoryGenerator.generateTrajectory( 
            List.of(new Pose2d(25.40 * 0.3048, 42.00 * 0.3048, Rotation2d.fromDegrees(270)),
                    new Pose2d(18.41 * 0.3048, 31.95 * 0.3048, Rotation2d.fromDegrees(201))), //-110.556
            config);
        
        /**
         * https://imgur.com/a/mHWSoxw
         */
        public static Trajectory pickupToShoot = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(18.41 * 0.3048, 31.95 * 0.3048, Rotation2d.fromDegrees(21)), //69.444
                    new Pose2d(19.00 * 0.3048, 50.20 * 0.3048, Rotation2d.fromDegrees(90))), 
            config);
            
        // public static Trajectory getLeft() { return middleStarting; }
        public static Trajectory getMiddle() { return middleStarting; }
        public static Trajectory getRight() { return rightStarting; }
    }

    /**
     * Trajectories used in eight ball configurations that pickup five balls from the alliance's trench
     */
    public static class EightTrench extends AutonTrajectory {
        public static Trajectory middleStarting = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(19.0074 * 0.3048, 42 * 0.3048, Rotation2d.fromDegrees(180)), 
                    new Pose2d(18.9578 * 0.3048, 37.7593 * 0.3048, Rotation2d.fromDegrees(270))),
            config);

        public static Trajectory rightStarting = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(25.5087 * 0.3048, 42 * 0.3048, Rotation2d.fromDegrees(180)), 
                    new Pose2d(18.9578 * 0.3048, 37.7593 * 0.3048, Rotation2d.fromDegrees(270))),
            config);
        
        public static Trajectory pickUpTrenchBalls = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(18.9578 * 0.3048, 37.7593 * 0.3048, Rotation2d.fromDegrees(90)),
                    new Pose2d(22.4739 * 0.3048, 22.4739 * 0.3048, Rotation2d.fromDegrees(180))),
            config);
        
        public static Trajectory alignFromTrench = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(22.4739 * 0.3048, 22.4739 * 0.3048, Rotation2d.fromDegrees(0)),
                    new Pose2d(18.9578 * 0.3048, 37.7593 * 0.3048, Rotation2d.fromDegrees(270))), 
            config);

        // public static Trajectory getLeft() { return middleStarting; }
        public static Trajectory getMiddle() { return middleStarting; }
        public static Trajectory getRight() { return rightStarting; }
    }

    /**
     * Trajectories used in eight ball configurations that pickup five balls from the rendevous point, 
     * starting to the left of the rendezvous point (from driver station perspective)
     */
    public static class EightRendezvous extends AutonTrajectory {
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
            config);
    }
    
    /**
     * Trajectories used in eight ball configurations that pickup five balls from the rendevous point, 
     * starting to the right of the rendezvous point (from driver station perspective)
     */
    public static class EightReverseRendezvous extends AutonTrajectory {
        public static Trajectory leftStart = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(-21.77)), 
                    new Pose2d(5, -2, Rotation2d.fromDegrees(-21.77))
           ),  config);
    
        public static Trajectory fromThreeBallRendezvousToTwoBallRendezvous = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(90)), 
                    new Pose2d(1, 2, Rotation2d.fromDegrees(0)),
                    new Pose2d(2, 0, Rotation2d.fromDegrees(270))),
            slowConfig);
        
        public static Trajectory getLeft() { return leftStart; }
        public static Trajectory getMiddle() { return leftStart; }
        public static Trajectory getRight() { return leftStart; }
    }

    /**
     * Trajectories used in ten ball configurations
     */
    public static class Ten extends AutonTrajectory {
        public static Trajectory leftStart = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(270)), 
                    new Pose2d(-1, -3, Rotation2d.fromDegrees(270))),
            config);
    
        public static Trajectory middleStart = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(270)),
                    new Pose2d(-5, -3, Rotation2d.fromDegrees(270))),
            config);
    
        // public static Trajectory rightStart = TrajectoryGenerator.generateTrajectory(
        //     List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(90)), 
        //             new Pose2d(1, 0, Rotation2d.fromDegrees(270))),
        //     config);
    
        public static Trajectory firstPickupToShootingPosition = TrajectoryGenerator.generateTrajectory(
           List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(90)), 
                   new Pose2d(2, 3, Rotation2d.fromDegrees(30)),
                   new Pose2d(6, 5, Rotation2d.fromDegrees(90))),
           config);

        public static Trajectory shootingToTrench = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(270)),
                    new Pose2d(2, -3, Rotation2d.fromDegrees(270)),
                    new Pose2d(2, -6, Rotation2d.fromDegrees(270))),
            config);
        
        public static Trajectory getLeft() { return leftStart; }
        public static Trajectory getMiddle() { return middleStart; }
        public static Trajectory getRight() { return middleStart; }
    }
    
    /**
     * 1678 Auto:
     * 1. Go to opponent trench and pickup two balls.
     * 2. Shoot
     * 3. Pickup two balls from rendezvous
     * 4. Pickup three balls from our trench
     * 5. Shoot
     */
    public static class TenTwoTrenchRendezvous extends AutonTrajectory {
        public static Trajectory leftStarting = TrajectoryGenerator.generateTrajectory( 
            List.of(new Pose2d(1.59 * 0.3048, 12.80, Rotation2d.fromDegrees(180)),
                    new Pose2d(2.48 * 0.3048, 32.45 * 0.3048, Rotation2d.fromDegrees(180))),
            config);
        
        public static Trajectory middleStarting = TrajectoryGenerator.generateTrajectory( 
            List.of(new Pose2d(13.4491 * 0.3048, 12.80, Rotation2d.fromDegrees(180)),
                    new Pose2d(2.48 * 0.3048, 32.45 * 0.3048, Rotation2d.fromDegrees(180))),
            config);
    
        // public static Trajectory rightStart = TrajectoryGenerator.generateTrajectory(
        //     List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(90)), 
        //             new Pose2d(1, 0, Rotation2d.fromDegrees(270))),
        //     config);
    
        //From first trench to shooting
        public static Trajectory firstPickupToShootingPosition = TrajectoryGenerator.generateTrajectory(
           List.of(new Pose2d(2.48 * 0.3048, 32.45 * 0.3048, Rotation2d.fromDegrees(90)), 
                   new Pose2d(18.9578 * 0.3048, 37.7593 * 0.3048, Rotation2d.fromDegrees(90))),
           config); 

        //Move from the shooting area to the rendezvous to pickup 2 balls.
        public static Trajectory shootingToRendezvous = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(18.9578 * 0.3048, 37.7593 * 0.3048, Rotation2d.fromDegrees(90)),
                    new Pose2d(20.4963 * 0.3048, 38.1067 * 0.3048, Rotation2d.fromDegrees(242.986))),
            config);

        //Move from the rendezvous and prepare to go to the other trench
        public static Trajectory rendezvousToPreTrench = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                    new Pose2d(2, 5, Rotation2d.fromDegrees(0))),
            config);

        //Move from this "pre trench" position to the actual part of the trench to intake three balls
        //The pre trench is the line right where the opponent's trench starts
         public static Trajectory preTrenchToSecondTrench = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, -5, Rotation2d.fromDegrees(270))
                    ),
            config);
        
        //Move from this second trench to the shooting area
        public static Trajectory secondTrenchToShooting = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 5, Rotation2d.fromDegrees(90))
                    ),
            config);    
        
        public static Trajectory getLeft() { return leftStarting; }
        public static Trajectory getMiddle() { return middleStarting; }
        // public static Trajectory getRight() { return middleStarting; }
    }

    /**
     * Reverse 1678 Auto:
     * 1. Go to rendezvous and pickup two balls
     * 2. Shoot
     * 3. Pickup three other balls from rendezvous
     * 4. Pickup two balls from opponent trench
     * 5. Shoot five balls
     */
    public static class RendezvousToTrenchTen extends AutonTrajectory {
        public static Trajectory leftStart = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(270)), 
                    new Pose2d(-1, -3, Rotation2d.fromDegrees(270))),
            config);
    
        public static Trajectory middleStart = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(270)),
                    new Pose2d(-5, -3, Rotation2d.fromDegrees(270))),
            config);
    
        // public static Trajectory rightStart = TrajectoryGenerator.generateTrajectory(
        //     List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(90)), 
        //             new Pose2d(1, 0, Rotation2d.fromDegrees(270))),
        //     config);
    
        //From first trench to shooting
        public static Trajectory firstPickupToShootingPosition = TrajectoryGenerator.generateTrajectory(
           List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(90)), 
                   new Pose2d(2, 3, Rotation2d.fromDegrees(30)),
                   new Pose2d(6, 5, Rotation2d.fromDegrees(90))),
           config); 

        //Move from the shooting area to the rendezvous to pickup 2 balls.
        public static Trajectory shootingToTrench = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(270)),
                    new Pose2d(-5, -2, Rotation2d.fromDegrees(270))),
            config);

        //Move from the rendezvous and prepare to go to the other trench
        public static Trajectory rendezvousToPreTrench = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                    new Pose2d(2, 5, Rotation2d.fromDegrees(0))),
            config);

        //Move from this "pre trench" position to the actual part of the trench to intake three balls
        //The pre trench is the line right where the opponent's trench starts
         public static Trajectory preTrenchToSecondTrench = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, -5, Rotation2d.fromDegrees(270))
                    ),
            config);
        
        //Move from this second trench to the shooting area
        public static Trajectory trenchToShooting = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 5, Rotation2d.fromDegrees(90))
                    ),
            config);    
        
        public static Trajectory getLeft() { return leftStart; }
        public static Trajectory getMiddle() { return middleStart; }
        public static Trajectory getRight() { return Baseline.moveForward; }
    }

    /** 
     * A very fast yet risky autonomous plan.
     * 1. Move to rendezvous and take in two balls.
     * 2. Shoot
     * 3. Move to trench and pickup 5 balls (in our trench)
     * 4. Shoot 
     * @precondition Starts on the right
     */
    public static class SpeedTenBallAuton extends AutonTrajectory {

        public static Trajectory rightStart = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(270)), 
                    new Pose2d(0, -3, Rotation2d.fromDegrees(225))),
            config);

        public static Trajectory rendezvousToShooting = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(45)), 
                        new Pose2d(2, 5, Rotation2d.fromDegrees(270))),
                config);
            
        public static Trajectory shootingToTrench = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(270)), 
                    new Pose2d(0, -5, Rotation2d.fromDegrees(270))),
            config);
        
        public static Trajectory trenchToShooting = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(90)), 
                    new Pose2d(0, 5, Rotation2d.fromDegrees(90))),
            config);

        public static Trajectory getLeft() { return Baseline.moveForward; }
        public static Trajectory getMiddle() { return Baseline.moveForward; }
        public static Trajectory getRight() { return rightStart; }
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
    
    /**
     * Super class for all Trajectory Categories to make selecting Trajectories
     * based on starting position easier
     */
    public static class AutonTrajectory {
        public static Trajectory getLeftStart() { return null; };
        public static Trajectory getMiddleStart() { return null; };
        public static Trajectory getRightStart() { return null; };

        public static Trajectory getStart() {
            StartingPosition s = Autons.startingPosition;
            if(s == StartingPosition.LEFT) return getLeftStart();
            if(s == StartingPosition.MIDDLE) return getMiddleStart();
            else return getRightStart();
        }
    }
}